/*
 * ESB Ticker Integration API
 *
 * Functions exposed by the ESB core (esb.c) for use by the BLE controller
 * ticker node (ull_esb.c / lll_esb.c). These are called during ESB radio
 * events to configure, operate, and clean up the radio for ESB protocol.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ESB_TICKER_H_
#define ESB_TICKER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start an ESB radio event.
 *
 * Called from the LLL prepare callback. Performs full radio reconfiguration
 * for ESB (MODE, CRC, PCNF, addresses, PPI) and starts the pending action
 * (RX listen or TX transaction). Equivalent to ts_start_action() in MPSL mode.
 *
 * @param slot_duration_us  Duration of this radio slot in microseconds.
 *                          A timer CC is set to close the event after this time.
 *
 * @return 0 on success, negative errno on failure.
 */
int esb_ticker_event_start(uint32_t slot_duration_us);

/**
 * @brief Handle RADIO IRQ during an ESB ticker event.
 *
 * Called from the LLL ISR (dispatched via radio_isr_set). Processes DISABLED
 * and END events through the ESB internal state machine (on_radio_disabled).
 *
 * @return true if the ESB event is complete (slot closed), false if more
 *         radio events are expected.
 */
bool esb_ticker_radio_handler(void);

/**
 * @brief Clean up after an ESB radio event.
 *
 * Called from the LLL done path. Disables radio, frees PPI channels,
 * stops the ESB timer. Must be called after the event ends, whether
 * normally (slot timeout) or by abort.
 */
void esb_ticker_event_end(void);

/**
 * @brief Abort-safe cleanup of ESB peripherals.
 *
 * Called from the LLL abort callback when the BLE controller preempts
 * the ESB event. Frees PPI channels, stops the ESB timer, and resets
 * FEM state, but does NOT disable the radio or mask radio interrupts.
 * The radio must remain interrupt-capable so that radio_disable() →
 * DISABLED event → ISR → lll_isr_cleanup() flow works correctly.
 */
void esb_ticker_event_abort(void);

/**
 * @brief Signal that the ESB slot should close.
 *
 * Called from the ESB timer CC0 interrupt when the slot duration expires.
 * Sets internal state so the next on_radio_disabled call triggers cleanup
 * instead of continuing the RX/TX sequence.
 */
void esb_ticker_slot_close(void);

/**
 * @brief Check if ESB is ready for ticker events.
 *
 * @return true if ESB is initialized and has a pending RX or TX action.
 */
bool esb_ticker_is_active(void);

/**
 * @brief Check if the current event completed (slot closed or action done).
 *
 * @return true if the ESB event has ended and cleanup can proceed.
 */
bool esb_ticker_event_is_done(void);
void esb_ticker_mark_idle(void);

/** Snapshot of the ticker node's diagnostic counters (see ull_esb.c). */
struct esb_ticker_diag {
	uint32_t prepare;        /* LLL prepare callbacks (slots that started) */
	uint32_t done;           /* slots that completed normally */
	uint32_t deferred;       /* prepares queued behind an active BLE event */
	uint32_t abort_active;   /* slots aborted while the radio was running */
	uint32_t abort_pipeline; /* prepares cancelled before starting */
	uint32_t errors;         /* event_start failures */
	uint32_t interval_us;    /* current ticker period */
	bool running;
};
void esb_ticker_get_diag(struct esb_ticker_diag *out);

#ifdef __cplusplus
}
#endif

#endif /* ESB_TICKER_H_ */
