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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_qos, CONFIG_ZMK_BLE_QOS_LOG_LEVEL);

#define QOS_THREAD_PRIORITY K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO)

static K_THREAD_STACK_DEFINE(qos_stack, CONFIG_ZMK_BLE_QOS_STACK_SIZE);
static struct k_thread qos_thread;

static uint8_t chmap_inst_buf[CHMAP_FILTER_INST_SIZE]
	__aligned(CHMAP_FILTER_INST_ALIGN);
static struct chmap_instance *chmap_inst;

static atomic_t processing;
static bool reporting_enabled;

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
		return true;
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

/*
 * QoS thread — runs at the lowest application priority.
 *
 * Periodically:
 * 1. Sets the "processing" atomic to prevent CRC updates during processing
 * 2. Calls chmap_filter_process() to evaluate channel quality
 * 3. If the filter recommends a channel map change, applies it via
 *    bt_le_set_chan_map() (only effective for connections where we are central)
 * 4. Confirms the map so the library's internal state stays consistent
 */
static void qos_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		bool update_channel_map;
		int err;

		k_sleep(K_MSEC(CONFIG_ZMK_BLE_QOS_INTERVAL));

		if (!reporting_enabled) {
			continue;
		}

		atomic_set(&processing, true);
		update_channel_map = chmap_filter_process(chmap_inst);
		atomic_set(&processing, false);

		if (!update_channel_map) {
			continue;
		}

		uint8_t *chmap = chmap_filter_suggested_map_get(chmap_inst);

		err = bt_le_set_chan_map(chmap);
		if (err) {
			LOG_WRN("bt_le_set_chan_map failed: %d", err);
		} else {
			LOG_INF("BLE channel map updated");
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
	if (err || reporting_enabled) {
		return;
	}

	int ret = enable_conn_event_reporting();
	if (!ret) {
		reporting_enabled = true;
	}
}

BT_CONN_CB_DEFINE(qos_conn_cb) = {
	.connected = connected_cb,
};

/*
 * Module init — called at system startup.
 * Initializes the chmap_filter library and starts the QoS thread.
 * VS event callback registration also happens here (before any connection).
 */
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

	LOG_INF("BLE QoS channel map filter initialized");

	return 0;
}

SYS_INIT(qos_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
