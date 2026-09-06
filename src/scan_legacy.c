/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * Tell the SoftDevice scanner to ignore extended advertising packets. The
 * split peripheral advertises with legacy PDUs only, and the vendor command's
 * documentation notes that a scanner not receiving extended packets "may be
 * able to receive more legacy advertising packets": every scan window,
 * including the backed-off 30 ms ones, catches the other half sooner.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/logging/log.h>

#include <sdc_hci_vs.h>

LOG_MODULE_REGISTER(zmk_ble_scan_legacy, LOG_LEVEL_INF);

static void apply_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(apply_work, apply_handler);

static void apply_handler(struct k_work *work) {
    if (!bt_is_ready()) {
        k_work_reschedule(&apply_work, K_SECONDS(1));
        return;
    }

    struct net_buf *buf = bt_hci_cmd_alloc(K_FOREVER);

    if (!buf) {
        return;
    }

    sdc_hci_cmd_vs_scan_accept_ext_adv_packets_set_t *cp = net_buf_add(buf, sizeof(*cp));

    cp->accept_ext_adv_packets = 0;

    int err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_SCAN_ACCEPT_EXT_ADV_PACKETS_SET, buf,
                                   NULL);
    if (err) {
        LOG_WRN("Scanner extended-packet filter failed: %d", err);
        return;
    }
    LOG_INF("Scanner accepts legacy advertising packets only");
}

static int scan_legacy_init(void) {
    k_work_schedule(&apply_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(scan_legacy_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
