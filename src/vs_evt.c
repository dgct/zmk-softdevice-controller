/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include <zmk/sdc/vs_evt.h>

LOG_MODULE_REGISTER(zmk_sdc_vs_evt, LOG_LEVEL_INF);

#define MAX_HANDLERS 4

static zmk_sdc_vs_evt_handler_t handlers[MAX_HANDLERS];
static uint8_t handler_count;

static bool dispatch(struct net_buf_simple *buf) {
    struct net_buf_simple_state state;
    bool handled = false;

    for (uint8_t i = 0; i < handler_count; i++) {
        net_buf_simple_save(buf, &state);
        if (handlers[i](buf)) {
            handled = true;
        }
        net_buf_simple_restore(buf, &state);
    }
    return handled;
}

int zmk_sdc_vs_evt_register(zmk_sdc_vs_evt_handler_t handler) {
    if (handler_count >= MAX_HANDLERS) {
        return -ENOMEM;
    }
    handlers[handler_count++] = handler;
    return 0;
}

static int vs_evt_init(void) {
    int err = bt_hci_register_vnd_evt_cb(dispatch);

    if (err) {
        LOG_ERR("vendor event callback: %d", err);
    }
    return err;
}

/* Before the consumers (they register at CONFIG_APPLICATION_INIT_PRIORITY). */
SYS_INIT(vs_evt_init, APPLICATION, 80);
