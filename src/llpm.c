/*
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * BLE LLPM (Low Latency Packet Mode) for ZMK split link.
 *
 * After the central connects to a split peripheral, this module issues
 * a vendor-specific connection update to switch the split link to a
 * sub-7.5ms connection interval (1-7ms in 1ms steps).
 *
 * Only applies to connections where the local device is the BLE central
 * (i.e. the split keyboard link).  The host link is peripheral-role and
 * is unaffected.
 *
 * LLPM mode is enabled in hci_driver.c init; this module handles the
 * per-connection VS conn update after connection establishment.
 *
 * Reference: nRF Desktop ble_conn_params.c (sdk-nrf)
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>

#include <sdc_hci_vs.h>
#include <sdc_hci_cmd_controller_baseband.h>
#if defined(CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE)
#include <sdc_hci_cmd_le.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_llpm, CONFIG_ZMK_BLE_LLPM_LOG_LEVEL);

/*
 * SDC limitation DRGN-11297: the current connection interval must be
 * ≤ 10ms before switching to an LLPM interval.  ZMK default split CI
 * is 7.5ms (PREF_INT=6), so this is satisfied.  If idle conn params
 * widen the CI, the standard bt_conn_le_param_update must narrow it
 * first before the VS conn update will succeed.
 */
#define LLPM_PRE_SWITCH_MAX_US  10000

/* Delay before issuing VS conn update after connection.
 * Lets pairing / encryption handshake complete first.
 */
#define LLPM_SWITCH_DELAY_MS    CONFIG_ZMK_BLE_LLPM_SWITCH_DELAY_MS

/* Flush timeout in baseband slots (0.625ms each).
 * 8 slots = 5ms — covers ~2 retransmit attempts at 2ms CI.
 * Stale trackpoint data older than this is discarded.
 */
#define FLUSH_TIMEOUT_SLOTS     8

static struct bt_conn *pending_conn;
static struct k_work_delayable llpm_switch_work;

/*
 * Send the vendor-specific connection update via HCI.
 * This is a Command Status command (not Command Complete).
 */
static int send_vs_conn_update(struct bt_conn *conn, uint32_t interval_us,
			       uint16_t latency, uint16_t timeout)
{
	uint16_t conn_handle;
	int err;

	err = bt_hci_get_conn_handle(conn, &conn_handle);
	if (err) {
		LOG_ERR("Failed to get conn handle: %d", err);
		return err;
	}

	struct net_buf *buf;

	buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_CONN_UPDATE,
				sizeof(sdc_hci_cmd_vs_conn_update_t));
	if (!buf) {
		LOG_ERR("Failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_vs_conn_update_t *cmd = net_buf_add(buf, sizeof(*cmd));
	cmd->conn_handle = conn_handle;
	cmd->conn_interval_us = interval_us;
	cmd->conn_latency = latency;
	cmd->supervision_timeout = timeout;

	err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_VS_CONN_UPDATE, buf);
	if (err) {
		LOG_ERR("VS conn update failed: %d", err);
	} else {
		LOG_INF("VS conn update sent: CI=%u us, lat=%u, to=%u",
			interval_us, latency, timeout);
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
		LOG_INF("Flush timeout set: %u slots (%.1f ms) on handle %u",
			flush_slots, flush_slots * 0.625f, conn_handle);
	}

	return err;
}

#if defined(CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE)
/*
 * Send a standard BLE 6.2 Connection Rate Request to negotiate a
 * sub-7.5ms connection interval via LL_CONNECTION_RATE_REQUEST PDUs.
 *
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
	cmd->subrate_min = 1;
	cmd->subrate_max = 1;
	cmd->max_latency = latency;
	cmd->continuation_number = 0;
	cmd->supervision_timeout = timeout;
	cmd->min_ce_length = 0;
	cmd->max_ce_length = 0;

	err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_CONN_RATE_REQUEST, buf);
	if (err) {
		LOG_ERR("Conn Rate Request failed: %d", err);
	} else {
		LOG_INF("Conn Rate Request sent: CI=%u (0.125ms units, %u us), lat=%u, to=%u",
			ci_val, interval_us, latency, timeout);
	}

	return err;
}
#endif /* CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE */

static void llpm_switch_work_fn(struct k_work *work)
{
	struct bt_conn *conn = pending_conn;

	if (!conn) {
		return;
	}

	struct bt_conn_info info;
	int err = bt_conn_get_info(conn, &info);

	if (err) {
		LOG_WRN("Cannot get conn info: %d", err);
		goto done;
	}

	/* Only apply to central-role connections (split link) */
	if (info.role != BT_CONN_ROLE_CENTRAL) {
		goto done;
	}

	/* Check SDC pre-condition: current CI must be ≤ 10ms */
	if (info.le.interval * 1250 > LLPM_PRE_SWITCH_MAX_US) {
		LOG_WRN("Current CI %u us too high for LLPM switch, "
			"narrow first", info.le.interval * 1250);
		goto done;
	}

	uint32_t target_us = CONFIG_ZMK_BLE_LLPM_INTERVAL_US;

	/* Already at target? */
	if (info.le.interval * 1250 == target_us ||
	    (info.le.interval < 6 && info.le.interval * 1000 == target_us)) {
		LOG_INF("Split link already at %u us", target_us);
		goto done;
	}

	err = send_vs_conn_update(conn, target_us, 0,
				  CONFIG_ZMK_BLE_LLPM_SUPERVISION_TIMEOUT);
	if (err) {
		LOG_WRN("LLPM switch failed: %d — will retry", err);
		k_work_reschedule(&llpm_switch_work,
				  K_MSEC(LLPM_SWITCH_DELAY_MS));
		return;
	}

#if defined(CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE)
	/*
	 * Also issue a standard Connection Rate Request so the link
	 * negotiates via LL_CONNECTION_RATE_REQUEST PDUs.  If the peer
	 * supports BLE 6.2 Shorter CI, this will take over; otherwise
	 * the VS path already succeeded above and this is a harmless
	 * no-op that the controller will reject gracefully.
	 */
	send_conn_rate_request(conn, target_us, 0,
			       CONFIG_ZMK_BLE_LLPM_SUPERVISION_TIMEOUT);
#endif

	if (IS_ENABLED(CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA)) {
		send_flush_timeout(conn, FLUSH_TIMEOUT_SLOTS);
	}

done:
	pending_conn = NULL;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
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

	LOG_INF("Central connection established — scheduling LLPM switch");
	pending_conn = conn;
	k_work_reschedule(&llpm_switch_work, K_MSEC(LLPM_SWITCH_DELAY_MS));
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (conn == pending_conn) {
		pending_conn = NULL;
		k_work_cancel_delayable(&llpm_switch_work);
	}
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

	/*
	 * LLPM intervals are encoded with the 0x0d00 magic mask in the
	 * interval field.  Standard BLE intervals are in 1.25ms units.
	 * If the interval has the LLPM mask, report in ms; otherwise
	 * in 1.25ms units.
	 */
	if (interval & 0x0d00) {
		uint16_t llpm_ms = interval & 0xFF;
		LOG_INF("Split link LLPM active: CI=%u ms, lat=%u, to=%u",
			llpm_ms, latency, timeout);
	} else {
		LOG_INF("Split link params updated: CI=%u (×1.25ms), lat=%u, to=%u",
			interval, latency, timeout);
	}
}

BT_CONN_CB_DEFINE(llpm_conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_updated = le_param_updated_cb,
};

#if defined(CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE)
/*
 * Set default rate parameters so the SDC proposes Shorter CI
 * automatically for every future central connection.
 */
static void set_default_rate_params(void)
{
	uint16_t ci_val = (uint16_t)(CONFIG_ZMK_BLE_LLPM_INTERVAL_US / 125);

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
	cmd->subrate_min = 1;
	cmd->subrate_max = 1;
	cmd->max_latency = 0;
	cmd->continuation_number = 0;
	cmd->supervision_timeout = CONFIG_ZMK_BLE_LLPM_SUPERVISION_TIMEOUT;
	cmd->min_ce_length = 0;
	cmd->max_ce_length = 0;

	int err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_RATE_PARAMS,
				  buf);
	if (err) {
		LOG_ERR("Set Default Rate Params failed: %d", err);
	} else {
		LOG_INF("Default Rate Params set: CI=%u (0.125ms units, %u us)",
			ci_val, CONFIG_ZMK_BLE_LLPM_INTERVAL_US);
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
	struct net_buf *evt = rsp;
	const uint8_t *data = evt->data + sizeof(struct bt_hci_evt_cmd_complete)
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
			"(%.3f–%.3f ms, step %.3f ms)",
			i, gmin, gmax, gstride,
			gmin * 0.125f, gmax * 0.125f, gstride * 0.125f);
	}

	net_buf_unref(rsp);
}
#endif /* CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE */

static int llpm_init(void)
{
	k_work_init_delayable(&llpm_switch_work, llpm_switch_work_fn);

	LOG_INF("LLPM module initialized (target CI=%u us, delay=%u ms)",
		CONFIG_ZMK_BLE_LLPM_INTERVAL_US,
		CONFIG_ZMK_BLE_LLPM_SWITCH_DELAY_MS);

#if defined(CONFIG_BT_CTLR_SDC_CONNECTION_RATE_UPDATE)
	set_default_rate_params();
	log_min_supported_conn_interval();
#endif

	return 0;
}

SYS_INIT(llpm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
