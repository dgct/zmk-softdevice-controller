/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * Zephyr v4.4 compat shim: bt_hci_cmd_create was removed in favour of
 * bt_hci_cmd_alloc.  SDC HCI headers still reference the old API.
 */

#ifndef ZMK_SDC_HCI_COMPAT_H_
#define ZMK_SDC_HCI_COMPAT_H_

#include <zephyr/bluetooth/buf.h>

static inline struct net_buf *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len) {
    (void)opcode; (void)param_len;
    return bt_hci_cmd_alloc(K_FOREVER);
}

#endif /* ZMK_SDC_HCI_COMPAT_H_ */
