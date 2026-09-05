/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * LE Power Control autonomous mode configuration for SoftDevice Controller.
 * Enables automatic TX power adjustments based on RSSI feedback from the peer.
 * Configured once per boot on every half (the parameters are controller-global).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include <zmk/sdc/hci_compat.h>

#include <sdc_hci_vs.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_power_control, LOG_LEVEL_INF);

static bool power_control_configured;

static int configure_power_control(void)
{
        struct net_buf *buf;
        int err;

        buf = bt_hci_cmd_create(
                SDC_HCI_OPCODE_CMD_VS_SET_POWER_CONTROL_REQUEST_PARAMS,
                sizeof(sdc_hci_cmd_vs_set_power_control_request_params_t));
        if (!buf) {
                LOG_ERR("Failed to allocate HCI buf for power control params");
                return -ENOMEM;
        }

        sdc_hci_cmd_vs_set_power_control_request_params_t *cmd = net_buf_add(
                buf, sizeof(sdc_hci_cmd_vs_set_power_control_request_params_t));

        cmd->auto_enable = 1;
        cmd->apr_enable = 1;
        cmd->beta = CONFIG_ZMK_BLE_POWER_CONTROL_BETA;
        cmd->lower_limit = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_LIMIT;
        cmd->upper_limit = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_LIMIT;
        cmd->lower_target_rssi = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_TARGET;
        cmd->upper_target_rssi = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_TARGET;
        cmd->wait_period_ms = CONFIG_ZMK_BLE_POWER_CONTROL_WAIT_PERIOD_MS;
        cmd->apr_margin = CONFIG_ZMK_BLE_POWER_CONTROL_APR_MARGIN;

        err = bt_hci_cmd_send_sync(
                SDC_HCI_OPCODE_CMD_VS_SET_POWER_CONTROL_REQUEST_PARAMS,
                buf, NULL);
        if (err) {
                LOG_ERR("Failed to set power control params: %d", err);
        } else {
                LOG_INF("LE Power Control autonomous mode enabled "
                        "(RSSI range [%d, %d] dBm, wait %u ms)",
                        CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_LIMIT,
                        CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_LIMIT,
                        CONFIG_ZMK_BLE_POWER_CONTROL_WAIT_PERIOD_MS);
        }

        return err;
}

/* The parameters are controller-global (no connection handle), so they can be
 * written as soon as the controller is up. Doing it at init means every
 * connection, including the host link on a peripheral-only half, runs with
 * autonomous power control from its first event; the connected callback is
 * the retry path if the init-time write could not be made. */
static void power_control_try_configure(void)
{
        if (power_control_configured) {
                return;
        }
        if (!configure_power_control()) {
                power_control_configured = true;
        }
}

static void power_control_connected_cb(struct bt_conn *conn, uint8_t err)
{
        if (err) {
                return;
        }
        power_control_try_configure();
}

BT_CONN_CB_DEFINE(power_control_conn_cb) = {
        .connected = power_control_connected_cb,
};

static int power_control_init(void)
{
        if (!bt_is_ready()) {
                LOG_WRN("Bluetooth not ready at init; power control configured on first connection");
                return 0;
        }
        power_control_try_configure();
        return 0;
}

/* After ZMK's BLE init (CONFIG_ZMK_BLE_INIT_PRIORITY), which enables Bluetooth. */
SYS_INIT(power_control_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
