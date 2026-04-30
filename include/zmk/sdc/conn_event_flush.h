/*
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_SDC_CONN_EVENT_FLUSH_H_
#define ZMK_SDC_CONN_EVENT_FLUSH_H_

/**
 * @brief Register a callback to be invoked on each BLE connection event.
 *
 * The callback fires from the BT RX thread after the SDC reports a
 * connection anchor point update. On a peripheral, this occurs for
 * every connection event in which a packet from the central is received.
 *
 * Only one callback may be registered. Calling again overwrites the previous.
 *
 * @param cb  Callback function, called from the BT RX thread context.
 * @return 0 on success, negative errno on failure.
 */
int zmk_sdc_conn_event_flush_register(void (*cb)(void));

#endif /* ZMK_SDC_CONN_EVENT_FLUSH_H_ */
