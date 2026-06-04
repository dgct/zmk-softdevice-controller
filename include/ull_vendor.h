/*
 * ull_vendor.h — BLE Controller USER_EXT hooks for ESB ticker node
 *
 * This header is included by the Zephyr BLE controller's ull.c when
 * CONFIG_BT_CTLR_USER_EXT is enabled. It provides the three required
 * inline functions that integrate proprietary ticker nodes with the
 * controller's ULL layer.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ULL_VENDOR_H_
#define ULL_VENDOR_H_

#if defined(CONFIG_BT_CTLR_USER_EXT)

#if defined(CONFIG_ESB_TICKER_TIMESLOT)
/* Forward declarations — avoid pulling in ull_esb.h (which includes lll.h,
 * conflicting with ull.c's own lll.h include).
 */
int ull_esb_init(void);
void ull_esb_done(struct node_rx_event_done *done);
#endif

struct ll_conn; /* forward declaration for ull_conn_interval_min_get */

/**
 * @brief Initialize user extension.
 *
 * Called from ull_init() during BLE controller startup.
 * Initializes the ULL ESB ticker node context.
 */
static inline int ull_user_init(void)
{
#if defined(CONFIG_ESB_TICKER_TIMESLOT)
	return ull_esb_init();
#else
	return 0;
#endif
}

/**
 * @brief Handle proprietary RX PDU.
 *
 * Called from rx_demux() when a NODE_RX_TYPE in the user range is received.
 * ESB does not generate proprietary RX PDUs through the controller's RX path,
 * so this is a no-op.
 */
static inline int rx_demux_rx_proprietary(memq_link_t *link,
					  struct node_rx_hdr *rx,
					  memq_link_t *tail,
					  memq_link_t **head)
{
	return 0;
}

/**
 * @brief Handle proprietary event done.
 *
 * Called from rx_demux_event_done() when the event_done_extra type is in
 * the user range (EVENT_DONE_EXTRA_TYPE_USER_START..USER_END).
 * Routes to the ULL ESB done handler.
 */
static inline void ull_proprietary_done(struct node_rx_event_done *evdone)
{
#if defined(CONFIG_ESB_TICKER_TIMESLOT)
	ull_esb_done(evdone);
#endif
}

/**
 * @brief Custom minimum connection interval.
 *
 * Returns the minimum connection interval in units of 1.25ms.
 * 6 = 7.5ms (BLE default minimum).
 */
static inline uint16_t ull_conn_interval_min_get(struct ll_conn *conn)
{
	return 6U;
}

#endif /* CONFIG_BT_CTLR_USER_EXT */

#endif /* ULL_VENDOR_H_ */
