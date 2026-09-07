/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * SoftDevice peripheral latency mode on every peripheral-role link.
 *
 * With peripheral latency the peripheral skips connection events, so a packet
 * that misses its event waits a whole latency window for the next attempt. In
 * "wait for ack" mode the SoftDevice keeps listening every event until the
 * central has acknowledged what the peripheral sent, then goes back to
 * skipping: idle power of latency, retransmit delay of no latency. Applied to
 * the split link on the left half and to the HID host link on the right.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <sdc_hci_vs.h>

LOG_MODULE_REGISTER(zmk_ble_periph_latency, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_ZMK_BLE_PERIPHERAL_LATENCY_WAIT_FOR_ACK)
#define LATENCY_MODE SDC_HCI_VS_PERIPHERAL_LATENCY_MODE_WAIT_FOR_ACK
#define LATENCY_MODE_NAME "wait for ack"
#elif IS_ENABLED(CONFIG_ZMK_BLE_PERIPHERAL_LATENCY_DISABLE)
#define LATENCY_MODE SDC_HCI_VS_PERIPHERAL_LATENCY_MODE_DISABLE
#define LATENCY_MODE_NAME "disabled"
#else
#define LATENCY_MODE SDC_HCI_VS_PERIPHERAL_LATENCY_MODE_ENABLE
#define LATENCY_MODE_NAME "enabled"
#endif

static ATOMIC_DEFINE(applied, CONFIG_BT_MAX_CONN);

static int set_mode(struct bt_conn *conn) {
    uint16_t handle;
    int err = bt_hci_get_conn_handle(conn, &handle);

    if (err) {
        return err;
    }

    struct net_buf *buf = bt_hci_cmd_alloc(K_MSEC(100));

    if (!buf) {
        return -ENOMEM;
    }

    sdc_hci_cmd_vs_peripheral_latency_mode_set_t *cp = net_buf_add(buf, sizeof(*cp));

    cp->conn_handle = sys_cpu_to_le16(handle);
    cp->mode = LATENCY_MODE;

    return bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_PERIPHERAL_LATENCY_MODE_SET, buf, NULL);
}

static void apply_to_conn(struct bt_conn *conn, void *data) {
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) || info.type != BT_CONN_TYPE_LE ||
        info.role != BT_CONN_ROLE_PERIPHERAL || info.state != BT_CONN_STATE_CONNECTED) {
        return;
    }
    if (atomic_test_and_set_bit(applied, bt_conn_index(conn))) {
        return;
    }

    int err = set_mode(conn);

    if (err) {
        atomic_clear_bit(applied, bt_conn_index(conn));
        LOG_WRN("Peripheral latency mode failed: %d", err);
        return;
    }
    LOG_INF("Peripheral latency mode: %s", LATENCY_MODE_NAME);
}

/* The vendor command is synchronous; run it off the connection callback. */
static void apply_work_handler(struct k_work *work) {
    bt_conn_foreach(BT_CONN_TYPE_LE, apply_to_conn, NULL);
}

static K_WORK_DEFINE(apply_work, apply_work_handler);

static void connected_cb(struct bt_conn *conn, uint8_t err) {
    if (err == 0) {
        k_work_submit(&apply_work);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
    atomic_clear_bit(applied, bt_conn_index(conn));
}

BT_CONN_CB_DEFINE(zmk_ble_periph_latency) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};
