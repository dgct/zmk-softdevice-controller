/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * Dedicated TX power for the split link.
 *
 * The board-level TX power (BT_CTLR_TX_PWR_*) sizes advertising and the host
 * link. The split link is a few tens of centimetres long, so central-role
 * connections get their own, lower level through the Zephyr vendor command
 * HCI_VS_Write_Tx_Power_Level (opcode 0xFC0E), which the SoftDevice
 * Controller implements. LE Power Control, when enabled, adjusts from here.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_ble_split_tx_power, LOG_LEVEL_INF);

static int write_conn_tx_power(struct bt_conn *conn, int8_t dbm)
{
	uint16_t handle;
	int err = bt_hci_get_conn_handle(conn, &handle);

	if (err) {
		return err;
	}

	struct net_buf *buf = bt_hci_cmd_alloc(K_FOREVER);

	if (!buf) {
		return -ENOMEM;
	}

	struct bt_hci_cp_vs_write_tx_power_level *cp = net_buf_add(buf, sizeof(*cp));

	cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_CONN;
	cp->handle = sys_cpu_to_le16(handle);
	cp->tx_power_level = dbm;

	struct net_buf *rsp;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
	if (err) {
		return err;
	}

	const struct bt_hci_rp_vs_write_tx_power_level *rp = (const void *)rsp->data;
	int8_t selected = rp->selected_tx_power;

	net_buf_unref(rsp);
	LOG_INF("Split link TX power: requested %d dBm, controller selected %d dBm", dbm,
		selected);
	return 0;
}

static void split_tx_power_connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}

	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
		return;
	}

	int ret = write_conn_tx_power(conn, CONFIG_ZMK_BLE_SPLIT_LINK_TX_POWER);

	if (ret) {
		LOG_WRN("Split link TX power write failed: %d", ret);
	}
}

BT_CONN_CB_DEFINE(split_tx_power_conn_cb) = {
	.connected = split_tx_power_connected_cb,
};
