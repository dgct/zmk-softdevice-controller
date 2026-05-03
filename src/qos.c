/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * BLE QoS channel map filter for ZMK + SoftDevice Controller.
 * Ported from nRF Desktop ble_qos module.
 *
 * Uses SDC's per-connection-event CRC statistics and Nordic's chmap_filter
 * library to dynamically blacklist interference-heavy BLE channels.
 * Only runs on the central (the side that can call bt_le_set_chan_map).
 *
 * Adaptive interval: when CRC errors are detected the processing interval
 * drops to QOS_INTERVAL_FAST_MS and then doubles each cycle (exponential
 * back-off) until it returns to QOS_INTERVAL (the baseline).  This lets
 * the filter react quickly to interference bursts while staying cheap
 * during quiet periods.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include <sdc_hci_vs.h>

#include "chmap_filter.h"

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_qos, CONFIG_ZMK_BLE_QOS_LOG_LEVEL);

#define QOS_THREAD_PRIORITY K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO)
#define QOS_INTERVAL_BASE   CONFIG_ZMK_BLE_QOS_INTERVAL
#define QOS_INTERVAL_FAST   CONFIG_ZMK_BLE_QOS_INTERVAL_FAST

static K_THREAD_STACK_DEFINE(qos_stack, CONFIG_ZMK_BLE_QOS_STACK_SIZE);
static struct k_thread qos_thread;

static uint8_t chmap_inst_buf[CHMAP_FILTER_INST_SIZE]
	__aligned(CHMAP_FILTER_INST_ALIGN);
static struct chmap_instance *chmap_inst;

static atomic_t processing;
static atomic_t crc_errors_seen;
static atomic_t burst_requested;
static atomic_t reinit_requested;
static bool reporting_enabled;
static uint32_t current_interval_ms = QOS_INTERVAL_FAST;
static enum zmk_activity_state last_activity_state = ZMK_ACTIVITY_SLEEP;

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
static int8_t survey_energy[40];
static atomic_t survey_data_ready;
static uint8_t survey_blocked_map[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
static uint16_t survey_keepout[37];
static bool survey_enabled;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
static struct bt_conn *host_conn;
static uint8_t last_applied_map[5];
#endif

static int apply_filter_params(void);

/*
 * VS HCI event callback — called from the BT RX thread whenever the
 * SoftDevice Controller generates a vendor-specific event.
 *
 * We handle SDC_HCI_SUBEVENT_VS_QOS_CONN_EVENT_REPORT here:
 * it carries per-connection-event CRC ok/error counts per channel.
 *
 * Uses an atomic flag to cheaply skip updates while the QoS thread
 * is running chmap_filter_process() — avoids any lock contention
 * on the hot path. Dropping occasional CRC samples is harmless.
 *
 * When CRC errors are observed the crc_errors_seen flag is set so the
 * QoS thread can tighten its processing interval on the next wake.
 */
static bool on_vs_evt(struct net_buf_simple *buf)
{
	uint8_t *subevent_code;
	sdc_hci_subevent_vs_qos_conn_event_report_t *evt;

	subevent_code = net_buf_simple_pull_mem(buf, sizeof(*subevent_code));

	switch (*subevent_code) {
	case SDC_HCI_SUBEVENT_VS_QOS_CONN_EVENT_REPORT:
		if (atomic_get(&processing)) {
			return true;
		}
		evt = (void *)buf->data;
		chmap_filter_crc_update(chmap_inst,
			evt->channel_index,
			evt->crc_ok_count,
			evt->crc_error_count);
		if (evt->crc_error_count > 0) {
			atomic_set(&crc_errors_seen, true);
		}
		return true;
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
	case SDC_HCI_SUBEVENT_VS_QOS_CHANNEL_SURVEY_REPORT: {
		sdc_hci_subevent_vs_qos_channel_survey_report_t *srv =
			(void *)buf->data;
		memcpy(survey_energy, srv->channel_energy, sizeof(survey_energy));
		atomic_set(&survey_data_ready, true);
		return true;
	}
#endif
	default:
		return false;
	}
}

/*
 * Enable QoS connection event reporting via VS HCI command.
 * This tells the SDC to generate per-connection-event CRC stats.
 * The command is routed through hci_internal.c → sdc_hci_cmd_vs_qos_conn_event_report_enable().
 */
static int enable_conn_event_reporting(void)
{
	struct net_buf *buf;
	int err;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_QOS_CONN_EVENT_REPORT_ENABLE,
				sizeof(sdc_hci_cmd_vs_qos_conn_event_report_enable_t));
	if (!buf) {
		LOG_ERR("Failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_vs_qos_conn_event_report_enable_t *cmd = net_buf_add(buf,
		sizeof(sdc_hci_cmd_vs_qos_conn_event_report_enable_t));
	cmd->enable = 1;

	err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_QOS_CONN_EVENT_REPORT_ENABLE,
				   buf, NULL);
	if (err) {
		LOG_ERR("Failed to enable QoS conn event reporting: %d", err);
	} else {
		LOG_INF("QoS conn event reporting enabled");
	}

	return err;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
/*
 * Enable QoS channel survey via VS HCI command.
 * The survey measures RF energy on each BLE data channel and generates
 * periodic reports at approximately the configured interval.
 */
static int enable_channel_survey(void)
{
	struct net_buf *buf;
	int err;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_QOS_CHANNEL_SURVEY_ENABLE,
				sizeof(sdc_hci_cmd_vs_qos_channel_survey_enable_t));
	if (!buf) {
		LOG_ERR("Failed to allocate HCI command buffer for survey");
		return -ENOMEM;
	}

	sdc_hci_cmd_vs_qos_channel_survey_enable_t *cmd = net_buf_add(buf,
		sizeof(sdc_hci_cmd_vs_qos_channel_survey_enable_t));
	cmd->enable = 1;
	cmd->interval_us = CONFIG_ZMK_BLE_QOS_SURVEY_INTERVAL_US;

	err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_QOS_CHANNEL_SURVEY_ENABLE,
				   buf, NULL);
	if (err) {
		LOG_ERR("Failed to enable channel survey: %d", err);
	} else {
		LOG_INF("Channel survey enabled (interval %u us)",
			CONFIG_ZMK_BLE_QOS_SURVEY_INTERVAL_US);
	}

	return err;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE) || \
    IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
/*
 * Count set bits in a 5-byte channel map (37 data channels).
 */
static int chmap_popcount(const uint8_t map[5])
{
	int count = 0;

	for (int i = 0; i < 5; i++) {
		uint8_t b = map[i];

		while (b) {
			count++;
			b &= b - 1;
		}
	}
	/* Only the lower 5 bits of map[4] are data channels. */
	return count;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
/*
 * Read the channel map of the host (peripheral-role) connection via HCI.
 * Returns the map macOS/Windows/Linux assigned via its AFH algorithm.
 */
static int read_host_chan_map(uint8_t out[5])
{
	struct bt_conn *conn = host_conn;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	if (!conn) {
		return -ENOTCONN;
	}

	/* Take a ref to prevent concurrent disconnected_cb from
	 * releasing the conn object while we use it. */
	conn = bt_conn_ref(conn);
	if (!conn) {
		return -ENOTCONN;
	}

	uint16_t handle;

	err = bt_hci_get_conn_handle(conn, &handle);
	if (err) {
		goto out;
	}

	buf = bt_hci_cmd_create(BT_HCI_OP_LE_READ_CHAN_MAP,
				sizeof(struct bt_hci_cp_le_read_chan_map));
	if (!buf) {
		err = -ENOMEM;
		goto out;
	}

	struct bt_hci_cp_le_read_chan_map *cp = net_buf_add(buf,
		sizeof(struct bt_hci_cp_le_read_chan_map));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_CHAN_MAP, buf, &rsp);
	if (err) {
		goto out;
	}

	struct bt_hci_rp_le_read_chan_map *rp = (void *)rsp->data;

	if (rp->status) {
		err = -EIO;
	} else {
		memcpy(out, rp->ch_map, 5);
	}

	net_buf_unref(rsp);
out:
	bt_conn_unref(conn);
	return err;
}

/*
 * Merge the QoS filter's suggested map with the host-link channel map.
 * Always populates merged[5]. Falls back to qos_map if the host map
 * is unavailable or the intersection is too narrow.
 */
static void merge_host_map(const uint8_t *qos_map, uint8_t merged[5])
{
	uint8_t host_map[5];

	if (read_host_chan_map(host_map)) {
		memcpy(merged, qos_map, 5);
		return;
	}

	for (int i = 0; i < 5; i++) {
		merged[i] = qos_map[i] & host_map[i];
	}

	int count = chmap_popcount(merged);

	if (count < CONFIG_ZMK_BLE_QOS_HOST_MAP_MIN_CHANNELS) {
		LOG_WRN("Host map merge would leave %d channels (min %d), skipping",
			count, CONFIG_ZMK_BLE_QOS_HOST_MAP_MIN_CHANNELS);
		memcpy(merged, qos_map, 5);
		return;
	}

	LOG_INF("Host map merged: %d channels", count);
}

/*
 * Check if the host channel map changed since the last applied map.
 * Populates merged[5] with the current merge result.
 */
static bool host_map_changed(const uint8_t *qos_map, uint8_t merged[5])
{
	if (!host_conn) {
		return false;
	}

	merge_host_map(qos_map, merged);

	return memcmp(merged, last_applied_map, 5) != 0;
}
#endif /* CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE */

/*
 * QoS thread — runs at the lowest application priority.
 *
 * Adaptive interval logic:
 * - Baseline: sleeps for QOS_INTERVAL_BASE (default 1000ms)
 * - On CRC errors: drops to QOS_INTERVAL_FAST (default 100ms)
 * - Each clean cycle (no new errors): interval doubles
 * - Capped at QOS_INTERVAL_BASE
 *
 * Steps each wake:
 * 1. Check if CRC errors were seen since last wake
 * 2. If yes, reset interval to FAST; if no, double toward BASE
 * 3. Run chmap_filter_process() to evaluate channel quality
 * 4. If the filter recommends a channel map change, apply it
 * 5. If host map merge is enabled, also check for host map changes
 */
static void qos_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		bool update_channel_map;
		int err;

		k_sleep(K_MSEC(current_interval_ms));

		if (!reporting_enabled) {
			continue;
		}

		/* Adapt interval: burst on CRC errors or wake-from-sleep. */
		bool burst = atomic_cas(&burst_requested, true, false);
		bool errors = atomic_cas(&crc_errors_seen, true, false);
		if (burst || errors) {
			if (current_interval_ms > QOS_INTERVAL_FAST) {
				LOG_INF("QoS: %s, interval %u -> %u ms",
					errors ? "CRC errors" : "wake burst",
					current_interval_ms, QOS_INTERVAL_FAST);
			}
			current_interval_ms = QOS_INTERVAL_FAST;
		} else if (current_interval_ms < QOS_INTERVAL_BASE) {
			uint32_t next = current_interval_ms * 2;
			if (next > QOS_INTERVAL_BASE) {
				next = QOS_INTERVAL_BASE;
			}
			current_interval_ms = next;
		}

		atomic_set(&processing, true);

		/* Handle reinit request from wake listener — must happen
		 * inside the processing gate so chmap_filter_process and
		 * chmap_filter_instance_init can't run concurrently. */
		if (atomic_cas(&reinit_requested, true, false)) {
			int rinit_err = chmap_filter_instance_init(chmap_inst,
								   sizeof(chmap_inst_buf));
			if (rinit_err) {
				LOG_ERR("chmap_filter re-init failed: %d", rinit_err);
			} else {
				apply_filter_params();
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
				memset(last_applied_map, 0, sizeof(last_applied_map));
#endif
			}
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
			memset(survey_keepout, 0, sizeof(survey_keepout));
			memset(survey_blocked_map, 0xFF, 4);
			survey_blocked_map[4] = 0x1F;
			atomic_set(&survey_data_ready, false);
#endif
		}

		update_channel_map = chmap_filter_process(chmap_inst);
		atomic_set(&processing, false);

		uint8_t *chmap = chmap_filter_suggested_map_get(chmap_inst);
		uint8_t final_map[5];

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
		/*
		 * Survey post-filter: apply energy-based blocks on top of
		 * the chmap_filter output. Uses a separate short keepout so
		 * false positives self-correct in ~60s while true positives
		 * are independently promoted by the CRC filter long-term.
		 */
		uint8_t pre_merge_map[5];
		bool survey_changed = false;

		/* Process new survey data if available. */
		if (atomic_cas(&survey_data_ready, true, false)) {
			int8_t local_energy[40];

			memcpy(local_energy, survey_energy, sizeof(local_energy));

			for (int ch = 0; ch < 37; ch++) {
				if (local_energy[ch] == 127) {
					/* No measurement for this channel. */
					continue;
				}
				if (local_energy[ch] >
				    CONFIG_ZMK_BLE_QOS_SURVEY_ENERGY_THRESHOLD) {
					if (survey_keepout[ch] == 0) {
						LOG_INF("Survey: block ch %d "
							"(energy %d dBm)",
							ch, local_energy[ch]);
					}
					survey_keepout[ch] =
						CONFIG_ZMK_BLE_QOS_SURVEY_KEEPOUT_CYCLES;
				}
			}

			LOG_DBG("Survey energy received");
		}

		/* Decrement keepout counters and rebuild blocked map. */
		uint8_t new_blocked_map[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};

		for (int ch = 0; ch < 37; ch++) {
			if (survey_keepout[ch] > 0) {
				survey_keepout[ch]--;
				/* Clear bit = blocked. */
				new_blocked_map[ch / 8] &=
					~(1U << (ch % 8));
				if (survey_keepout[ch] == 0) {
					LOG_INF("Survey: unblock ch %d", ch);
				}
			}
		}

		if (memcmp(new_blocked_map, survey_blocked_map, 5) != 0) {
			memcpy(survey_blocked_map, new_blocked_map, 5);
			survey_changed = true;
		}

		/* Apply survey subtraction to filter output. */
		for (int i = 0; i < 5; i++) {
			pre_merge_map[i] = chmap[i] & survey_blocked_map[i];
		}

		/* Floor check: don't let survey starve the channel map. */
		if (chmap_popcount(pre_merge_map) <
		    CONFIG_ZMK_BLE_QOS_MIN_CHANNEL_COUNT) {
			LOG_WRN("Survey would leave %d channels (min %d), "
				"using filter-only map",
				chmap_popcount(pre_merge_map),
				CONFIG_ZMK_BLE_QOS_MIN_CHANNEL_COUNT);
			memcpy(pre_merge_map, chmap, 5);
			survey_changed = false;
		}

		if (survey_changed) {
			update_channel_map = true;
		}

		const uint8_t *effective_map = pre_merge_map;
#else
		const uint8_t *effective_map = chmap;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
		if (update_channel_map) {
			merge_host_map(effective_map, final_map);
		} else if (host_map_changed(effective_map, final_map)) {
			update_channel_map = true;
		}
#else
		memcpy(final_map, effective_map, 5);
#endif

		if (!update_channel_map) {
			continue;
		}

		err = bt_le_set_chan_map(final_map);
		if (err) {
			LOG_WRN("bt_le_set_chan_map failed: %d", err);
		} else {
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
			memcpy(last_applied_map, final_map, 5);
#endif
			LOG_INF("BLE channel map updated (interval=%u ms)",
				current_interval_ms);
		}

		chmap_filter_suggested_map_confirm(chmap_inst);
	}
}

/*
 * Connection callback — enable QoS reporting on first connection.
 * We only need to send the VS HCI enable command once; it persists
 * across connections.
 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
	struct bt_conn_info info;

	if (!bt_conn_get_info(conn, &info) &&
	    info.role == BT_CONN_ROLE_PERIPHERAL && !host_conn) {
		host_conn = bt_conn_ref(conn);
		LOG_INF("QoS: host connection tracked for channel map merge");
	}
#endif

	if (!reporting_enabled) {
		int ret = enable_conn_event_reporting();
		if (!ret) {
			reporting_enabled = true;
		}
	}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
	if (!survey_enabled) {
		int ret = enable_channel_survey();
		if (!ret) {
			survey_enabled = true;
		}
	}
#endif
}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (conn == host_conn) {
		LOG_INF("QoS: host connection lost, disabling channel map merge");
		bt_conn_unref(host_conn);
		host_conn = NULL;
	}
}
#endif

BT_CONN_CB_DEFINE(qos_conn_cb) = {
	.connected = connected_cb,
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
	.disconnected = disconnected_cb,
#endif
};

/*
 * Module init — called at system startup.
 * Initializes the chmap_filter library and starts the QoS thread.
 * VS event callback registration also happens here (before any connection).
 */
static int apply_filter_params(void)
{
	struct chmap_filter_params params;

	chmap_filter_params_get(chmap_inst, &params);
	params.min_channel_count = CONFIG_ZMK_BLE_QOS_MIN_CHANNEL_COUNT;
	params.eval_keepout_duration = CONFIG_ZMK_BLE_QOS_EVAL_KEEPOUT_DURATION;

	int err = chmap_filter_params_set(chmap_inst, &params);
	if (err) {
		LOG_ERR("chmap_filter_params_set failed: %d", err);
	} else {
		LOG_INF("QoS filter params: min_ch=%u keepout=%u",
			params.min_channel_count, params.eval_keepout_duration);
	}
	return err;
}

static int qos_init(void)
{
	int err;

	chmap_filter_init();
	chmap_inst = (struct chmap_instance *)chmap_inst_buf;

	err = chmap_filter_instance_init(chmap_inst, sizeof(chmap_inst_buf));
	if (err) {
		LOG_ERR("chmap_filter_instance_init failed: %d", err);
		return err;
	}

	apply_filter_params();

	err = bt_hci_register_vnd_evt_cb(on_vs_evt);
	if (err) {
		LOG_ERR("Failed to register VS event callback: %d", err);
		return err;
	}

	k_thread_create(&qos_thread, qos_stack,
			K_THREAD_STACK_SIZEOF(qos_stack),
			qos_thread_fn, NULL, NULL, NULL,
			QOS_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&qos_thread, "ble_qos");

	LOG_INF("BLE QoS channel map filter initialized (fast burst on boot)");

	return 0;
}

SYS_INIT(qos_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Activity state listener — triggers a QoS burst on wake from sleep.
 * Idle→active transitions (frequent with 1s idle timeout) are ignored.
 * Sleep→active means the RF environment may have changed significantly.
 *
 * With ZMK_BLE_QOS_REINIT_ON_WAKE enabled, the chmap_filter instance is
 * fully re-initialized on sleep→active. This clears all stale channel
 * ratings and blocked-channel history built up before sleep. The CRC
 * update callback is gated by the `processing` atomic during re-init
 * to avoid feeding data into a half-initialized instance.
 */
static int qos_activity_listener(const zmk_event_t *eh)
{
	struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
	if (ev == NULL) {
		return -ENOTSUP;
	}

	enum zmk_activity_state prev = last_activity_state;
	last_activity_state = ev->state;

	if (ev->state == ZMK_ACTIVITY_ACTIVE && prev == ZMK_ACTIVITY_SLEEP) {
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_REINIT_ON_WAKE)
		LOG_INF("QoS: wake from sleep, requesting filter re-init");
		/* Signal the QoS thread to re-init inside its own
		 * processing gate, avoiding concurrent access to
		 * chmap_inst from the listener (sysworkq) and the
		 * QoS thread. */
		atomic_set(&reinit_requested, true);
#else
		LOG_INF("QoS: wake from sleep, requesting burst");
#endif
		atomic_set(&burst_requested, true);
	}

	return 0;
}

ZMK_LISTENER(ble_qos_activity, qos_activity_listener);
ZMK_SUBSCRIPTION(ble_qos_activity, zmk_activity_state_changed);
