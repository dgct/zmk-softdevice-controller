/*
 * LLL ESB — Lower Link Layer for ESB ticker node
 *
 * Handles the radio event lifecycle: prepare (radio config + ESB RX/TX start),
 * ISR dispatch (routes RADIO_IRQn to ESB's handler), abort policy, and
 * event completion signaling.
 *
 * Follows the same LLL patterns as lll_peripheral.c / lll_conn.c.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <soc.h>

#include "util/util.h"
#include "util/memq.h"

#include "hal/radio.h"

#include "lll.h"
#include "lll_clock.h"
#include "lll_internal.h"

#include "hal/debug.h"

#include "lll_esb.h"
#include "ull_esb.h"
#include "esb_ticker.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lll_esb, CONFIG_ESB_LOG_LEVEL);

static int esb_lll_is_abort_cb(void *next, void *curr,
			       lll_prepare_cb_t *resume_cb);
static void esb_lll_abort_cb(struct lll_prepare_param *prepare_param,
			     void *param);
static int esb_lll_prepare_cb(struct lll_prepare_param *p);

static void esb_lll_isr(void *param);
static void esb_lll_isr_done(void *param);

/* Diagnostic counters — logged from thread context via ull_esb.c */
uint32_t esb_diag_prep_count;
uint32_t esb_diag_done_count;
uint32_t esb_diag_abort_active_count;
uint32_t esb_diag_abort_pipeline_count;
uint32_t esb_diag_err_count;
uint32_t esb_diag_defer_count;

void lll_esb_prepare(void *param)
{
	int err;

	/* Request HFXO — reference-counted, usually already running for BLE.
	 * Balanced by lll_hfclock_off() in esb_lll_isr_done or abort_cb.
	 * For -EINPROGRESS (pipeline enqueue), the abort_cb pipeline path
	 * calls lll_hfclock_off() to balance this.
	 */
	err = lll_hfclock_on();
	LL_ASSERT_ERR(!err || err == -EINPROGRESS);

	err = lll_prepare(esb_lll_is_abort_cb,
			  esb_lll_abort_cb,
			  esb_lll_prepare_cb,
			  0,
			  param);

	if (err == -EINPROGRESS) {
		/* Normal interleaving: ESB enqueued behind active BLE event.
		 * The pipeline will dequeue and run ESB when BLE finishes,
		 * or the preempt timer will preempt BLE in ESB's favor.
		 * This is the designed ticker coexistence path.
		 */
		esb_diag_defer_count++;
	} else if (err) {
		LOG_WRN("LLL ESB: prepare failed (%d)", err);
	}
}

/**
 * @brief Abort policy callback.
 *
 * Determines whether the current ESB event can be preempted by a
 * higher-priority event (e.g., BLE connection). ESB is lower priority
 * than BLE connections, so we always yield.
 *
 * @return -ECANCELED to allow preemption (yield to BLE)
 *         0 to refuse (never used — ESB always yields)
 */
static int esb_lll_is_abort_cb(void *next, void *curr,
			       lll_prepare_cb_t *resume_cb)
{
	/* ESB always yields to BLE events */
	return -ECANCELED;
}

/**
 * @brief Abort handler callback.
 *
 * Called when the ESB event is preempted. Cleans up any partially-started
 * ESB radio state.
 *
 * @param prepare_param  NULL when active event is being aborted (radio running).
 *                       Non-NULL when a pipeline prepare is being cancelled.
 * @param param          The LLL ESB context pointer.
 */
static void esb_lll_abort_cb(struct lll_prepare_param *prepare_param,
			     void *param)
{
	if (!prepare_param) {
		/* Active event abort — radio is running.
		 * Clean up ESB peripherals (PPI, timer, FEM) and set
		 * done_flag=true. radio_disable() triggers DISABLED ISR
		 * → esb_lll_isr_done → lll_done(NULL) → done handler.
		 */
		esb_diag_abort_active_count++;
		esb_ticker_event_abort();
		radio_isr_set(esb_lll_isr_done, param);
		radio_disable();
	} else {
		/* Pipeline prepare cancelled — event never started.
		 * Reverse the HFXO request from lll_esb_prepare().
		 */
		esb_diag_abort_pipeline_count++;
		lll_hfclock_off();
		lll_done(param);
	}
}

/**
 * @brief Radio prepare callback.
 *
 * Called when the ticker scheduler grants us radio time. Routes RADIO IRQs
 * to ESB's handler, then reconfigures the radio for ESB protocol and starts
 * the pending action (RX listen or TX).
 *
 * Follows the BLE LLL contract: start real radio work, return 0, and let the
 * ISR chain signal done asynchronously via esb_lll_isr → esb_lll_isr_done →
 * lll_done(NULL). NEVER call lll_done() from this function on the hot path.
 */
static int esb_lll_prepare_cb(struct lll_prepare_param *p)
{
	struct lll_esb *lll = p->param;
	int ret;

	esb_diag_prep_count++;

	/* Route RADIO interrupts to our ESB handler for this slot */
	radio_isr_set(esb_lll_isr, lll);

	/* Start the ESB radio event — full reconfiguration + RX/TX start.
	 * This is the equivalent of BLE's radio_tmr_start() in
	 * lll_peripheral.c: it starts real radio work and the ISR chain
	 * will call esb_lll_isr_done → lll_done(NULL) when the slot closes.
	 */
	ret = esb_ticker_event_start(CONFIG_ESB_TICKER_SLOT_US);
	if (ret) {
		esb_diag_err_count++;
		LOG_WRN("LLL ESB: event_start failed (%d), releasing slot", ret);
		/* One-shot inline done — safe because esb_ticker_is_active()
		 * returns false on the next tick (ts_next_action is CLOSE/IDLE
		 * after the error), so this does NOT sustain at 200 Hz.
		 */
		lll_hfclock_off();
		lll_done(NULL);
		return ret;
	}

	lll_prepare_done(lll);

	return 0;
}

/**
 * @brief Radio ISR handler for ESB events.
 *
 * Dispatched by the BLE controller's isr_radio() via radio_isr_set().
 * Routes the RADIO interrupt to ESB's internal handler and checks if
 * the ESB event has completed (slot closed).
 */
static void esb_lll_isr(void *param)
{
	struct lll_esb *lll = param;
	bool done;

	/* Dispatch to ESB's radio interrupt handler.
	 * This processes DISABLED/END events through the ESB state machine
	 * (on_radio_disabled function pointer chain).
	 */
	done = esb_ticker_radio_handler();

	if (done) {
		/* ESB event complete — chain to the done ISR */
		esb_lll_isr_done(lll);
	}
	/* else: more radio events expected (e.g., ACK turnaround),
	 * the ISR will fire again on the next DISABLED event.
	 * radio_isr_set() is NOT re-called because the BLE controller's
	 * isr_cb pointer persists until the next lll_prepare_done().
	 */
}

/**
 * @brief Event done ISR handler.
 *
 * Called when the ESB event is complete. Fills the event_done_extra
 * structure for ULL processing and triggers cleanup.
 */
static void esb_lll_isr_done(void *param)
{
	struct event_done_extra *e;

	ARG_UNUSED(param);

	esb_diag_done_count++;

	/* Signal event completion to ULL — routes done to ull_esb_done()
	 * via ull_proprietary_done().
	 */
	e = ull_event_done_extra_get();
	if (e) {
		e->type = EVENT_DONE_EXTRA_TYPE_USER_START;
	}

	/* Clear pending radio events — critically EVENTS_DISABLED.
	 * Without this, a stale EVENTS_DISABLED causes the RADIO IRQ to
	 * re-fire immediately after lll_done() returns, calling this
	 * function again. The second lll_done(NULL) hits
	 * LL_ASSERT_ERR(event.curr.abort_cb) since abort_cb was already
	 * cleared, triggering a HardFault (UDF) in RADIO ISR context
	 * where USB CDC logging cannot output — total system death.
	 *
	 * This is the ESB equivalent of radio_status_reset() in BLE's
	 * lll_isr_cleanup(). We do NOT use lll_isr_cleanup() because it
	 * calls radio_tmr_stop() and radio_stop() which touch TIMER0
	 * and BLE's radio.c, corrupting BLE controller state.
	 */
	radio_status_reset();

	lll_hfclock_off();
	lll_done(NULL);
}
