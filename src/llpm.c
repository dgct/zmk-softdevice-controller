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

static int llpm_init(void)
{
	k_work_init_delayable(&llpm_switch_work, llpm_switch_work_fn);

	LOG_INF("LLPM module initialized (target CI=%u us, delay=%u ms)",
		CONFIG_ZMK_BLE_LLPM_INTERVAL_US,
		CONFIG_ZMK_BLE_LLPM_SWITCH_DELAY_MS);

	return 0;
}

SYS_INIT(llpm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
