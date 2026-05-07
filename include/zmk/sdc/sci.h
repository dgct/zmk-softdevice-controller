/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZMK_SDC_SCI_H_
#define ZMK_SDC_SCI_H_

#include <zephyr/bluetooth/conn.h>

/**
 * @brief Set flush timeout on a connection based on its current CI.
 *
 * Activates Flushable ACL Data so stale HID reports are discarded after
 * ~3 connection events instead of being retransmitted indefinitely.
 * The timeout scales with the connection interval.
 *
 * Safe to call multiple times — recalculates and re-applies.
 * No-op if CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA or flush timeout is disabled.
 *
 * @param conn The BLE connection.
 * @return 0 on success, negative errno on failure.
 */
int sci_set_flush_timeout(struct bt_conn *conn);

#endif /* ZMK_SDC_SCI_H_ */
