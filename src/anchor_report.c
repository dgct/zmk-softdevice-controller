/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * Diagnostic: connection anchor point reports from the SoftDevice.
 *
 * The controller reports the anchor point (its own 64-bit microsecond clock)
 * and event counter of every connection event it runs as central, and of
 * every event in which it received a packet as peripheral. From a run of
 * reports this module derives, per link, the effective interval, how many
 * events went unreported (peripheral side: not received; central side: the
 * host could not pull the report in time) and, with two links up, the phase
 * between them modulo the shorter interval. That is the direct measurement
 * of whether the 2 ms split link and the 7.5 ms host link collide.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <sdc_hci_vs.h>
#include <zmk/sdc/vs_evt.h>

LOG_MODULE_REGISTER(zmk_ble_anchor, LOG_LEVEL_INF);

#define MAX_LINKS 4

struct link_stats {
    bool used;
    uint16_t handle;
    uint16_t first_counter;
    uint16_t last_counter;
    uint64_t first_anchor;
    uint64_t last_anchor;
    uint32_t reports;
};

static struct link_stats links[MAX_LINKS];
static struct k_spinlock lock;

static struct link_stats *link_for(uint16_t handle) {
    for (int i = 0; i < MAX_LINKS; i++) {
        if (links[i].used && links[i].handle == handle) {
            return &links[i];
        }
    }
    for (int i = 0; i < MAX_LINKS; i++) {
        if (!links[i].used) {
            links[i].used = true;
            links[i].handle = handle;
            links[i].reports = 0;
            return &links[i];
        }
    }
    return NULL;
}

static bool on_vs_evt(struct net_buf_simple *buf) {
    uint8_t subevent = net_buf_simple_pull_u8(buf);

    if (subevent != SDC_HCI_SUBEVENT_VS_CONN_ANCHOR_POINT_UPDATE_REPORT ||
        buf->len < sizeof(sdc_hci_subevent_vs_conn_anchor_point_update_report_t)) {
        return false;
    }

    const sdc_hci_subevent_vs_conn_anchor_point_update_report_t *rep = (const void *)buf->data;
    uint16_t handle = sys_le16_to_cpu(rep->conn_handle);
    uint16_t counter = sys_le16_to_cpu(rep->event_counter);
    uint64_t anchor = sys_le64_to_cpu(rep->anchor_point_us);

    k_spinlock_key_t key = k_spin_lock(&lock);
    struct link_stats *l = link_for(handle);

    if (l != NULL) {
        if (l->reports == 0) {
            l->first_counter = counter;
            l->first_anchor = anchor;
        }
        l->last_counter = counter;
        l->last_anchor = anchor;
        l->reports++;
    }
    k_spin_unlock(&lock, key);
    return true;
}

static void report_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(report_work, report_handler);

static void report_handler(struct k_work *work) {
    struct link_stats snap[MAX_LINKS];
    k_spinlock_key_t key = k_spin_lock(&lock);

    memcpy(snap, links, sizeof(snap));
    for (int i = 0; i < MAX_LINKS; i++) {
        links[i].used = false;
    }
    k_spin_unlock(&lock, key);

    uint32_t interval[MAX_LINKS] = {0};

    for (int i = 0; i < MAX_LINKS; i++) {
        struct link_stats *l = &snap[i];

        if (!l->used || l->reports < 2) {
            continue;
        }
        uint16_t events = (uint16_t)(l->last_counter - l->first_counter);
        uint64_t span = l->last_anchor - l->first_anchor;

        if (events == 0) {
            continue;
        }
        interval[i] = (uint32_t)(span / events);
        LOG_INF("anchor[0x%04x]: %u reports over %u events, interval %u us, %u unreported",
                l->handle, l->reports, events, interval[i], events + 1 - l->reports);
    }

    for (int a = 0; a < MAX_LINKS; a++) {
        for (int b = a + 1; b < MAX_LINKS; b++) {
            if (interval[a] == 0 || interval[b] == 0) {
                continue;
            }
            uint32_t base = MIN(interval[a], interval[b]);
            int64_t delta = (int64_t)snap[b].last_anchor - (int64_t)snap[a].last_anchor;
            int64_t phase = delta % (int64_t)base;

            if (phase < 0) {
                phase += base;
            }
            LOG_INF("anchor phase 0x%04x -> 0x%04x: %lld us modulo %u us", snap[a].handle,
                    snap[b].handle, (long long)phase, base);
        }
    }
    k_work_schedule(&report_work, K_SECONDS(CONFIG_ZMK_BLE_ANCHOR_REPORT_INTERVAL_S));
}

static void enable_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(enable_work, enable_handler);

static void enable_handler(struct k_work *work) {
    if (!bt_is_ready()) {
        k_work_reschedule(&enable_work, K_SECONDS(1));
        return;
    }

    struct net_buf *buf = bt_hci_cmd_alloc(K_FOREVER);

    if (!buf) {
        return;
    }

    sdc_hci_cmd_vs_conn_anchor_point_update_event_report_enable_t *cp =
        net_buf_add(buf, sizeof(*cp));

    cp->enable = 1;

    int err = bt_hci_cmd_send_sync(
        SDC_HCI_OPCODE_CMD_VS_CONN_ANCHOR_POINT_UPDATE_EVENT_REPORT_ENABLE, buf, NULL);
    if (err) {
        LOG_WRN("Anchor point reports: enable failed (%d)", err);
        return;
    }
    LOG_INF("Anchor point reports enabled, summary every %d s",
            CONFIG_ZMK_BLE_ANCHOR_REPORT_INTERVAL_S);
    k_work_schedule(&report_work, K_SECONDS(CONFIG_ZMK_BLE_ANCHOR_REPORT_INTERVAL_S));
}

static int anchor_report_init(void) {
    int err = zmk_sdc_vs_evt_register(on_vs_evt);

    if (err) {
        LOG_ERR("vendor event handler: %d", err);
        return err;
    }
    k_work_schedule(&enable_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(anchor_report_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
