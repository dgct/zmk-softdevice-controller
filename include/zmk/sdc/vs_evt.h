/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * One host vendor-event callback, many consumers. Zephyr keeps a single
 * bt_hci_register_vnd_evt_cb() slot; the modules here (QoS, anchor point
 * reports, survey log) each register a handler with this dispatcher instead.
 * A handler receives the buffer positioned at the subevent code, exactly as
 * the host delivers it, and returns true when it consumed the event.
 */
#pragma once

#include <stdbool.h>
#include <zephyr/net_buf.h>

typedef bool (*zmk_sdc_vs_evt_handler_t)(struct net_buf_simple *buf);

int zmk_sdc_vs_evt_register(zmk_sdc_vs_evt_handler_t handler);
