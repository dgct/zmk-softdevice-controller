/*
 * ULL ESB — Upper Link Layer for ESB ticker node
 *
 * Manages the ticker lifecycle: start/stop, ticker callback dispatch,
 * and event-done processing for ESB as a BLE controller ticker node.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ULL_ESB_H_
#define ULL_ESB_H_

/* lll.h has no include guards — include it from .c files before this header.
 * struct ull_hdr and struct lll_hdr must be defined before including this header.
 */
#include "lll_esb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ULL ESB context.
 *
 * Follows the standard ULL/LLL embedding pattern: ull_hdr as first member,
 * lll context with lll_hdr as first member, linked via lll_hdr_init().
 */
struct ull_esb {
	struct ull_hdr ull;
	struct lll_esb lll;
};

/**
 * @brief Initialize the ULL ESB module.
 *
 * Called from ull_user_init() in ull_vendor.h during controller startup.
 * Initializes the ULL/LLL headers.
 *
 * @return 0 on success, negative errno on failure.
 */
int ull_esb_init(void);

/**
 * @brief Start the ESB ticker node.
 *
 * Creates a periodic ticker that fires every ESB_TICKER_INTERVAL_US.
 * Called after esb_start_rx() or esb_start_tx() sets the pending action.
 *
 * @return 0 on success, negative errno on failure.
 */
int ull_esb_start(void);

/** @brief Current ESB ticker slot length in microseconds (valid once started). */
uint32_t ull_esb_get_slot_us(void);

/** @brief Windows skipped between listens (0 = every window). */
uint32_t ull_esb_get_skip(void);

/** @brief Copy the host link's data channel map (see esb_get_host_chan_map). */
int ull_esb_host_chan_map(uint8_t map[5]);

/**
 * @brief Listen in every (skip + 1)th ESB period (idle tiers).
 *
 * Applied through a ticker lazy update from thread context; the requested
 * value is reported by ull_esb_get_skip() immediately. Any context.
 */
int ull_esb_set_skip(uint32_t skip);

/**
 * @brief Request deferred ESB ticker start.
 *
 * Submits a work item to start the ESB ticker on the system workqueue.
 * Safe to call from any context (SYS_INIT, thread). The actual
 * ticker_start() only executes when both preconditions are met:
 *   1. ULL ESB is initialized (ull_esb_init() has run)
 *   2. ESB has a pending action (esb_ticker_is_active() returns true)
 *
 * This handles the init-order ambiguity between esb_start_rx() (SYS_INIT)
 * and ull_esb_init() (bt_enable -> ull_init -> ull_user_init).
 */
void ull_esb_request_start(void);

/**
 * @brief Stop the ESB ticker node.
 *
 * Stops the periodic ticker. Called on esb_stop_rx() / esb_suspend().
 *
 * @return 0 on success, negative errno on failure.
 */
int ull_esb_stop(void);

/**
 * @brief Change the ESB ticker period at runtime.
 *
 * The ESB ticker period cannot be altered in place (ticker_update only
 * adjusts drift/slot), so this defers a ticker stop+restart to thread
 * context. The interval is clamped to [1250, 20000] us. A no-op (and a
 * 0 return) if the requested interval equals the current one.
 *
 * @param interval_us  New ESB ticker period in microseconds.
 * @return 0 on success (including no-op), negative errno on failure.
 */
int ull_esb_reconfigure(uint32_t interval_us);

/**
 * @brief Get the current ESB ticker period in microseconds.
 */
uint32_t ull_esb_get_interval_us(void);

/**
 * @brief Process an ESB event-done signal.
 *
 * Called from ull_proprietary_done() in ull_vendor.h when the LLL ESB
 * event completes. Decrements the ULL reference count.
 *
 * @param done  The event-done node from the done FIFO.
 */
void ull_esb_done(struct node_rx_event_done *done);

/**
 * @brief Get a pointer to the singleton ULL ESB context.
 */
struct ull_esb *ull_esb_get(void);

#ifdef __cplusplus
}
#endif

#endif /* ULL_ESB_H_ */
