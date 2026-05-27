/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * BLE Path Loss Monitoring for host/dongle links.
 * Enables LE Path Loss Monitoring on peripheral-role connections and reports
 * zone transitions (low/middle/high/unavailable) for link quality awareness.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ble_path_loss, LOG_LEVEL_INF);

static struct bt_conn *path_loss_conn;

static void path_loss_connected_cb(struct bt_conn *conn, uint8_t err)
{
        if (err) {
                return;
        }

        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) ||
            info.role != BT_CONN_ROLE_PERIPHERAL) {
                return;
        }

        if (!path_loss_conn) {
                static const struct bt_conn_le_path_loss_reporting_param
                        plm_params = {
                        .high_threshold =
                                CONFIG_ZMK_BLE_PATH_LOSS_HIGH_THRESHOLD,
                        .high_hysteresis =
                                CONFIG_ZMK_BLE_PATH_LOSS_HIGH_HYSTERESIS,
                        .low_threshold =
                                CONFIG_ZMK_BLE_PATH_LOSS_LOW_THRESHOLD,
                        .low_hysteresis =
                                CONFIG_ZMK_BLE_PATH_LOSS_LOW_HYSTERESIS,
                        .min_time_spent =
                                CONFIG_ZMK_BLE_PATH_LOSS_MIN_TIME_SPENT,
                };
                int ret = bt_conn_le_set_path_loss_mon_param(conn, &plm_params);
                if (ret) {
                        LOG_WRN("Path loss params failed: %d", ret);
                } else {
                        ret = bt_conn_le_set_path_loss_mon_enable(conn, true);
                        if (ret) {
                                LOG_WRN("Path loss enable failed: %d", ret);
                        } else {
                                path_loss_conn = bt_conn_ref(conn);
                                LOG_INF("Path loss monitoring enabled "
                                        "(high=%u low=%u dB)",
                                        CONFIG_ZMK_BLE_PATH_LOSS_HIGH_THRESHOLD,
                                        CONFIG_ZMK_BLE_PATH_LOSS_LOW_THRESHOLD);
                        }
                }
        }
}

static void path_loss_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
        if (conn == path_loss_conn) {
                LOG_INF("Path loss monitoring: host disconnected");
                bt_conn_unref(path_loss_conn);
                path_loss_conn = NULL;
        }
}

static void path_loss_threshold_cb(struct bt_conn *conn,
        const struct bt_conn_le_path_loss_threshold_report *report)
{
        static const char *zone_names[] = {
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_LOW] = "LOW",
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_MIDDLE] = "MIDDLE",
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_HIGH] = "HIGH",
                [BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE] = "UNAVAILABLE",
        };

        const char *zone = (report->zone <= BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE)
                ? zone_names[report->zone] : "UNKNOWN";

        if (report->zone == BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_HIGH) {
                LOG_WRN("Path loss HIGH zone: %u dB — host link degraded",
                        report->path_loss);
        } else if (report->zone == BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE) {
                LOG_WRN("Path loss UNAVAILABLE — peer may not support "
                        "LE Power Control");
        } else {
                LOG_INF("Path loss zone: %s (%u dB)", zone, report->path_loss);
        }
}

BT_CONN_CB_DEFINE(path_loss_conn_cb) = {
        .connected = path_loss_connected_cb,
        .disconnected = path_loss_disconnected_cb,
        .path_loss_threshold_report = path_loss_threshold_cb,
};
