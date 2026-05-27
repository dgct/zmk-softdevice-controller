/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * LE Power Control autonomous mode configuration for SoftDevice Controller.
 * Enables automatic TX power adjustments based on RSSI feedback from the peer.
 */

#include <zephyr/kernel.h>
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

static void power_control_security_changed_cb(struct bt_conn *conn,
                                              bt_security_t level,
                                              enum bt_security_err err)
{
        if (err || level < BT_SECURITY_L2) {
                return;
        }

        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
                return;
        }

        if (!power_control_configured) {
                int ret = configure_power_control();
                if (!ret) {
                        power_control_configured = true;
                }
        }
}

BT_CONN_CB_DEFINE(power_control_conn_cb) = {
        .security_changed = power_control_security_changed_cb,
};
