/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * QoS channel survey without the channel map filter: for a half that cannot
 * change the channel map (the split peripheral) the survey is still the only
 * picture of the interference it sees. The controller measures idle energy
 * on all 40 channels at the fourth scheduling priority; this module logs the
 * latest survey at a slow cadence.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <sdc_hci_vs.h>
#include <zmk/sdc/vs_evt.h>

LOG_MODULE_REGISTER(zmk_ble_survey, LOG_LEVEL_INF);

static int8_t energy[40];
static atomic_t have_data;

static bool on_vs_evt(struct net_buf_simple *buf) {
    uint8_t subevent = net_buf_simple_pull_u8(buf);

    if (subevent != SDC_HCI_SUBEVENT_VS_QOS_CHANNEL_SURVEY_REPORT ||
        buf->len < sizeof(sdc_hci_subevent_vs_qos_channel_survey_report_t)) {
        return false;
    }
    const sdc_hci_subevent_vs_qos_channel_survey_report_t *rep = (const void *)buf->data;

    memcpy(energy, rep->channel_energy, sizeof(energy));
    atomic_set(&have_data, 1);
    return true;
}

static void log_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(log_work, log_handler);

static void log_line(const char *label, int first) {
    char line[100];
    int pos = 0;

    for (int i = first; i < first + 20 && pos < (int)sizeof(line) - 5; i++) {
        pos += snprintk(&line[pos], sizeof(line) - pos, "%d ", energy[i]);
    }
    LOG_INF("survey dBm %s: %s", label, line);
}

static void log_handler(struct k_work *work) {
    if (atomic_get(&have_data)) {
        log_line("ch0-19", 0);
        log_line("ch20-39", 20);
    }
    k_work_schedule(&log_work, K_SECONDS(CONFIG_ZMK_BLE_QOS_SURVEY_LOG_INTERVAL_S));
}

static void enable_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(enable_work, enable_handler);

static void enable_handler(struct k_work *work) {
    if (!bt_is_ready()) {
        k_work_reschedule(&enable_work, K_SECONDS(1));
        return;
    }

    struct net_buf *buf = bt_hci_cmd_alloc(K_MSEC(100));

    if (!buf) {
        return;
    }

    sdc_hci_cmd_vs_qos_channel_survey_enable_t *cp = net_buf_add(buf, sizeof(*cp));

    cp->enable = 1;
    cp->interval_us = sys_cpu_to_le32(CONFIG_ZMK_BLE_QOS_SURVEY_LOG_SURVEY_INTERVAL_US);

    int err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_QOS_CHANNEL_SURVEY_ENABLE, buf, NULL);

    if (err) {
        LOG_WRN("Channel survey: enable failed (%d)", err);
        return;
    }
    LOG_INF("Channel survey every %u us, logged every %d s",
            CONFIG_ZMK_BLE_QOS_SURVEY_LOG_SURVEY_INTERVAL_US,
            CONFIG_ZMK_BLE_QOS_SURVEY_LOG_INTERVAL_S);
    k_work_schedule(&log_work, K_SECONDS(CONFIG_ZMK_BLE_QOS_SURVEY_LOG_INTERVAL_S));
}

static int survey_log_init(void) {
    int err = zmk_sdc_vs_evt_register(on_vs_evt);

    if (err) {
        return err;
    }
    k_work_schedule(&enable_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(survey_log_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
