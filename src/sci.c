/*
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * BLE SCI — Shorter Connection Intervals (BLE 6.2) for ZMK split link.
 *
 * After the central connects to a split peripheral and the link
 * upgrades to LE 2M PHY, negotiates a sub-7.5ms connection interval
 * using standard BLE 6.2 Shorter Connection Intervals procedures:
 *   1. Frame Space Update — negotiate tighter inter-frame spacing
 *   2. Connection Rate Request — request the target sub-7.5ms CI
 *
 * Replaces the vendor-specific LLPM approach (SDC_HCI_OPCODE_CMD_VS_CONN_UPDATE)
 * which required both sides to independently enable the Nordic proprietary mode.
 * SCI is negotiated via standard LL_CONNECTION_RATE_REQUEST PDUs.
 *
 * Only applies to connections where the local device is the BLE central
 * (i.e. the split keyboard link).  The host link is peripheral-role and
 * is unaffected.
 *
 * Limitations on ZMK's Zephyr fork:
 *   - No bt_conn_le_frame_space_update() / bt_conn_le_conn_rate_request()
 *     host APIs — uses raw SDC HCI commands instead.
 *   - No conn_rate_changed / frame_space_updated bt_conn_cb callbacks —
 *     uses fixed delays between steps and optimistic state tracking.
 *
 * Reference: dgct-ble-dongle/app/src/sci.c (NCS implementation)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>

#include <sdc_hci_cmd_le.h>
#include <sdc_hci_cmd_controller_baseband.h>

#include <zmk/sdc/sci.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_sci, CONFIG_ZMK_BLE_SCI_LOG_LEVEL);

/* Target SCI connection interval in microseconds */
#define SCI_TARGET_US          CONFIG_ZMK_BLE_SCI_INTERVAL_US

/* Supervision timeout (×10ms) */
#define SCI_SUPERVISION_TO     CONFIG_ZMK_BLE_SCI_SUPERVISION_TIMEOUT

/* Delay after PHY 2M before starting SCI procedures (ms) */
#define SCI_SWITCH_DELAY_MS    CONFIG_ZMK_BLE_SCI_SWITCH_DELAY_MS

/* Delay before re-triggering SCI after split reconnection (ms).
 * Shorter than initial delay since the link is already established. */
#define SCI_RETRIGGER_DELAY_MS  CONFIG_ZMK_BLE_SCI_RETRIGGER_DELAY_MS

/*
 * Delay between frame space update and conn rate request (ms).
 * Gives the controller time to complete the LL frame space negotiation
 * before we request the shorter connection interval.
 */
#define SCI_FSU_TO_CRR_DELAY_MS  200

/* Minimum flush timeout in baseband slots (0.625ms each).
 * 8 slots = 5ms — covers ~2 retransmit attempts at 2ms CI.
 * Actual timeout is scaled by CI: max(MIN_FLUSH_SLOTS, interval * 6)
 * to allow ~3 connection events for delivery at any CI. */
#define MIN_FLUSH_SLOTS        CONFIG_ZMK_BLE_SCI_FLUSH_TIMEOUT_MIN_SLOTS

#define SCI_MAX_RETRIES        10

/* Frame space update PHY and spacing type values (BLE 6.2 Core Spec) */
#define FSU_PHY_2M             0x02    /* bit 1 = LE 2M PHY */
#define FSU_SPACING_ACL_IFS    0x0001  /* bit 0 = ACL IFS */

/*
 * State machine for SCI switch.
 *
 * The dongle uses conn_rate_changed / frame_space_updated callbacks to
 * advance between states.  ZMK's Zephyr fork lacks those callbacks, so
 * we use fixed delays between steps instead.
 *
 *   IDLE ──▶ FRAME_SPACE_UPDATE ──(delay)──▶ CONN_RATE_REQUESTED ──(delay)──▶ ACTIVE
 */
enum sci_state {
	SCI_IDLE,
	SCI_FRAME_SPACE_UPDATE,   /* FSU sent, waiting before conn rate req */
	SCI_CONN_RATE_REQUESTED,  /* Conn rate request sent */
	SCI_ACTIVE,               /* SCI assumed active */
};

static struct bt_conn *pending_conn;
static struct k_work_delayable sci_switch_work;
static enum sci_state sci_state;
static int sci_retries;

/*
 * Send LE Frame Space Update HCI command.
 * Requests the lowest possible frame space on 2M PHY so the controller
 * can pack more retransmit attempts into each connection event.
 */
static int send_frame_space_update(struct bt_conn *conn)
{
	uint16_t conn_handle;
	int err;

	err = bt_hci_get_conn_handle(conn, &conn_handle);
	if (err) {
		LOG_ERR("FSU: failed to get conn handle: %d", err);
		return err;
	}

	struct net_buf *buf;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_LE_FRAME_SPACE_UPDATE,
				sizeof(sdc_hci_cmd_le_frame_space_update_t));
	if (!buf) {
		LOG_ERR("FSU: failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_le_frame_space_update_t *cmd =
		net_buf_add(buf, sizeof(*cmd));
	cmd->conn_handle = conn_handle;
	cmd->frame_space_min = 0;
	cmd->frame_space_max = 150;
	cmd->phys = FSU_PHY_2M;
	cmd->spacing_types = FSU_SPACING_ACL_IFS;

	err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_FRAME_SPACE_UPDATE, buf);
	if (err) {
		LOG_ERR("FSU failed: %d", err);
	} else {
		LOG_INF("Frame space update requested (min=0, max=150, 2M PHY)");
	}

	return err;
}

/*
 * Set the automatic flush timeout on a connection.
 * Activates the Flushable ACL Data feature (BLE 6.2) so stale
 * trackpoint/mouse packets are discarded instead of retransmitted.
 */
static int send_flush_timeout(struct bt_conn *conn, uint16_t flush_slots)
{
	uint16_t conn_handle;
	int err;

	err = bt_hci_get_conn_handle(conn, &conn_handle);
	if (err) {
		LOG_ERR("Flush timeout: failed to get conn handle: %d", err);
		return err;
	}

	struct net_buf *buf;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_CB_WRITE_AUTOMATIC_FLUSH_TIMEOUT,
				sizeof(sdc_hci_cmd_cb_write_automatic_flush_timeout_t));
	if (!buf) {
		LOG_ERR("Flush timeout: failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_cb_write_automatic_flush_timeout_t *cmd =
		net_buf_add(buf, sizeof(*cmd));
	cmd->conn_handle = conn_handle;
	cmd->flush_timeout = flush_slots;

	err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_CB_WRITE_AUTOMATIC_FLUSH_TIMEOUT, buf);
	if (err) {
		LOG_ERR("Write flush timeout failed: %d", err);
	} else {
		LOG_INF("Flush timeout set: %u slots (%.1f ms)",
			flush_slots, flush_slots * 0.625f);
	}

	return err;
}

/*
 * Send Connection Rate Request HCI command.
 * interval_us is the target CI in microseconds.
 * BLE 6.2 Shorter CI uses 0.125ms (125µs) units.
 */
static int send_conn_rate_request(struct bt_conn *conn, uint32_t interval_us,
				  uint16_t latency, uint16_t timeout)
{
	uint16_t conn_handle;
	int err;

	err = bt_hci_get_conn_handle(conn, &conn_handle);
	if (err) {
		LOG_ERR("Conn Rate Request: failed to get conn handle: %d", err);
		return err;
	}

	/* Convert µs to 0.125ms units (BLE 6.2 Shorter CI encoding) */
	uint16_t ci_val = (uint16_t)(interval_us / 125);

	struct net_buf *buf;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_LE_CONN_RATE_REQUEST,
				sizeof(sdc_hci_cmd_le_conn_rate_request_t));
	if (!buf) {
		LOG_ERR("Conn Rate Request: failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_le_conn_rate_request_t *cmd =
		net_buf_add(buf, sizeof(*cmd));
	cmd->conn_handle = conn_handle;
	cmd->conn_interval_min = ci_val;
	cmd->conn_interval_max = ci_val;
#if IS_ENABLED(CONFIG_BT_SUBRATING)
	cmd->subrate_min = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN;
	cmd->subrate_max = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX;
#else
	cmd->subrate_min = 1;
	cmd->subrate_max = 1;
#endif
	cmd->max_latency = latency;
	cmd->continuation_number = 0;
	cmd->supervision_timeout = timeout;
	cmd->min_ce_length = 0;
	cmd->max_ce_length = 0;

	err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_CONN_RATE_REQUEST, buf);
	if (err) {
		LOG_ERR("Conn Rate Request failed: %d", err);
	} else {
		LOG_INF("Conn Rate Request sent: CI=%u (0.125ms units, %u us)",
			ci_val, interval_us);
	}

	return err;
}

static void sci_switch_work_fn(struct k_work *work)
{
	struct bt_conn *conn = pending_conn;
	int err;

	if (!conn) {
		return;
	}

	switch (sci_state) {
	case SCI_IDLE:
		/*
		 * Step 1: Frame Space Update.
		 * Negotiate tighter inter-frame spacing on 2M PHY so the
		 * controller can support shorter connection intervals.
		 */
		sci_state = SCI_FRAME_SPACE_UPDATE;
		err = send_frame_space_update(conn);
		if (err) {
			LOG_WRN("FSU failed: %d — skipping to conn rate request",
				err);
			/* FSU failure is not fatal; the controller uses
			 * whatever frame space it can.  Proceed to step 2. */
		}
		/* Wait for the LL frame space negotiation to complete */
		k_work_reschedule(&sci_switch_work,
				  K_MSEC(SCI_FSU_TO_CRR_DELAY_MS));
		return;

	case SCI_FRAME_SPACE_UPDATE:
		/*
		 * Step 2: Connection Rate Request.
		 * Request the target sub-7.5ms CI via standard
		 * LL_CONNECTION_RATE_REQUEST PDUs.
		 */
		sci_state = SCI_CONN_RATE_REQUESTED;
		err = send_conn_rate_request(conn, SCI_TARGET_US, 0,
					     SCI_SUPERVISION_TO);
		if (err) {
			goto retry;
		}
		/*
		 * ZMK's Zephyr lacks conn_rate_changed callback, so we
		 * wait a short period then optimistically declare active.
		 */
		k_work_reschedule(&sci_switch_work,
				  K_MSEC(SCI_FSU_TO_CRR_DELAY_MS));
		return;

	case SCI_CONN_RATE_REQUESTED:
		/*
		 * Step 3: SCI assumed active.  Set flush timeout so stale
		 * trackpoint packets are discarded instead of retransmitted.
		 */
		LOG_INF("SCI switch complete: target CI=%u us", SCI_TARGET_US);
		sci_state = SCI_ACTIVE;
		pending_conn = NULL;

		if (IS_ENABLED(CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA)) {
			/* Use SCI target CI to compute flush slots.
			 * SCI_TARGET_US / 625 * 3 = ~3 connection events. */
			uint16_t ci_slots = (uint16_t)(SCI_TARGET_US / 625);
			uint16_t slots = MAX(MIN_FLUSH_SLOTS, ci_slots * 6);

			send_flush_timeout(conn, slots);
		}
		return;

	default:
		return;
	}

retry:
	sci_retries++;
	if (sci_retries >= SCI_MAX_RETRIES) {
		LOG_ERR("SCI switch failed after %d retries", SCI_MAX_RETRIES);
		sci_state = SCI_IDLE;
		pending_conn = NULL;
		return;
	}
	LOG_WRN("SCI retry %d/%d", sci_retries, SCI_MAX_RETRIES);
	k_work_reschedule(&sci_switch_work, K_MSEC(SCI_SWITCH_DELAY_MS));
}

/*
 * Trigger SCI switch when PHY reaches 2M.
 * SCI on nRF52840 requires LE 2M PHY for the shortest intervals.
 */
static void le_phy_updated_cb(struct bt_conn *conn,
			      struct bt_conn_le_phy_info *param)
{
	if (param->tx_phy != BT_GAP_LE_PHY_2M) {
		return;
	}

	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info)) {
		return;
	}

	/* Only target central-role connections (split link) */
	if (info.role != BT_CONN_ROLE_CENTRAL) {
		return;
	}

	if (conn == pending_conn && sci_state != SCI_IDLE) {
		return; /* switch already in progress */
	}

	LOG_INF("PHY 2M ready — scheduling SCI switch in %d ms",
		SCI_SWITCH_DELAY_MS);
	pending_conn = conn;
	sci_retries = 0;
	sci_state = SCI_IDLE;
	k_work_reschedule(&sci_switch_work, K_MSEC(SCI_SWITCH_DELAY_MS));
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (conn == pending_conn) {
		pending_conn = NULL;
		sci_state = SCI_IDLE;
		k_work_cancel_delayable(&sci_switch_work);
	}
}

/*
 * Reject standard L2CAP conn param update requests on the split link
 * while SCI is active.  Without this, the split peripheral's PPCP
 * (CI=7.5ms) overrides the SCI interval a few seconds after connection.
 *
 * Only applies to central-role connections (split link).  Host link
 * (peripheral role) param updates are always accepted — the host
 * needs to adjust CI for its own power management.
 */
static bool le_param_req_cb(struct bt_conn *conn,
			    struct bt_le_conn_param *param)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
		return true; /* always accept host link param updates */
	}

	if (sci_state == SCI_ACTIVE || sci_state == SCI_CONN_RATE_REQUESTED) {
		LOG_INF("Rejecting split conn param update (CI=%u-%u) — SCI active",
			param->interval_min, param->interval_max);
		return false;
	}
	return true;
}

static void le_param_updated_cb(struct bt_conn *conn, uint16_t interval,
				uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info)) {
		return;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		return;
	}

	LOG_INF("Split link params: CI=%u (%u.%02u ms), lat=%u, to=%u",
		interval, (interval * 125) / 100, (interval * 125) % 100,
		latency, timeout);

	/*
	 * Re-trigger SCI when keyboard returns from deep idle.
	 * Guard: lat=0 (active mode), CI ≤ 10ms, not already switching.
	 */
	if (latency == 0 && sci_state == SCI_IDLE &&
	    interval * 1250 <= 10000) {
		LOG_INF("Keyboard active (CI=%u) — scheduling SCI switch",
			interval);
		pending_conn = conn;
		sci_retries = 0;
		k_work_reschedule(&sci_switch_work,
				  K_MSEC(SCI_RETRIGGER_DELAY_MS));
	}
}

BT_CONN_CB_DEFINE(sci_conn_cb) = {
	.le_phy_updated = le_phy_updated_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req_cb,
	.le_param_updated = le_param_updated_cb,
};

/*
 * Set default connection rate parameters so the SDC proposes
 * Shorter CI automatically for future central connections.
 */
static void set_default_rate_params(void)
{
	uint16_t ci_val = (uint16_t)(SCI_TARGET_US / 125);

	struct net_buf *buf;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_RATE_PARAMS,
				sizeof(sdc_hci_cmd_le_set_default_rate_params_t));
	if (!buf) {
		LOG_ERR("Set Default Rate Params: alloc failed");
		return;
	}

	sdc_hci_cmd_le_set_default_rate_params_t *cmd =
		net_buf_add(buf, sizeof(*cmd));
	cmd->conn_interval_min = ci_val;
	cmd->conn_interval_max = ci_val;
#if IS_ENABLED(CONFIG_BT_SUBRATING)
	cmd->subrate_min = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN;
	cmd->subrate_max = CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX;
#else
	cmd->subrate_min = 1;
	cmd->subrate_max = 1;
#endif
	cmd->max_latency = 0;
	cmd->continuation_number = 0;
	cmd->supervision_timeout = SCI_SUPERVISION_TO;
	cmd->min_ce_length = 0;
	cmd->max_ce_length = 0;

	int err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_RATE_PARAMS,
				  buf);
	if (err) {
		LOG_ERR("Set Default Rate Params failed: %d", err);
	} else {
		LOG_INF("Default Rate Params set: CI=%u (0.125ms units, %u us)",
			ci_val, SCI_TARGET_US);
	}
}

/*
 * Query and log the minimum supported connection interval and
 * supported interval groups from the SDC.  One-time diagnostic.
 */
static void log_min_supported_conn_interval(void)
{
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(
		SDC_HCI_OPCODE_CMD_LE_READ_MIN_SUPPORTED_CONN_INTERVAL, 0);
	if (!buf) {
		LOG_ERR("Read Min Supported CI: alloc failed");
		return;
	}

	err = bt_hci_cmd_send_sync(
		SDC_HCI_OPCODE_CMD_LE_READ_MIN_SUPPORTED_CONN_INTERVAL,
		buf, &rsp);
	if (err || !rsp) {
		LOG_ERR("Read Min Supported CI failed: %d", err);
		return;
	}

	/* Skip Command Complete header (3 bytes: ncmd, opcode) + status (1) */
	const uint8_t *data = rsp->data + sizeof(struct bt_hci_evt_cmd_complete)
			      + sizeof(struct bt_hci_evt_cc_status);
	uint8_t min_ci = data[0];
	uint8_t num_groups = data[1];

	LOG_INF("Min supported CI: %u (0.125ms units = %u us), %u group(s)",
		min_ci, min_ci * 125, num_groups);

	const uint8_t *gp = &data[2];

	for (uint8_t i = 0; i < num_groups && i < 8; i++) {
		uint16_t gmin = sys_get_le16(&gp[i * 6]);
		uint16_t gmax = sys_get_le16(&gp[i * 6 + 2]);
		uint16_t gstride = sys_get_le16(&gp[i * 6 + 4]);

		LOG_INF("  Group %u: min=%u max=%u stride=%u "
			"(%.3f-%.3f ms, step %.3f ms)",
			i, gmin, gmax, gstride,
			gmin * 0.125f, gmax * 0.125f, gstride * 0.125f);
	}

	net_buf_unref(rsp);
}

/*
 * Set flush timeout on a host-link (peripheral role) connection.
 * Scales timeout relative to the current connection interval so stale
 * HID reports are discarded after ~3 connection events at any CI.
 *
 * Called from ble_latency when the host connection is secured or when
 * the connection interval changes.  Safe to call multiple times.
 */
int sci_set_flush_timeout(struct bt_conn *conn)
{
	if (!IS_ENABLED(CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA)) {
		return 0;
	}

	if (MIN_FLUSH_SLOTS == 0) {
		return 0; /* disabled by config */
	}

	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info)) {
		return -EINVAL;
	}

	/*
	 * interval is in 1.25ms units.  Convert to baseband slots (0.625ms)
	 * and allow ~3 connection events for delivery.
	 *   ci_slots = interval * (1.25 / 0.625) = interval * 2
	 *   flush = max(MIN_FLUSH_SLOTS, ci_slots * 3)
	 */
	uint16_t ci_slots = info.le.interval * 2;
	uint16_t slots = MAX(MIN_FLUSH_SLOTS, ci_slots * 3);

	return send_flush_timeout(conn, slots);
}

static int sci_init(void)
{
	k_work_init_delayable(&sci_switch_work, sci_switch_work_fn);

	set_default_rate_params();
	log_min_supported_conn_interval();

	LOG_INF("SCI module initialized (target CI=%u us, delay=%u ms)",
		SCI_TARGET_US, SCI_SWITCH_DELAY_MS);

	return 0;
}

SYS_INIT(sci_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
