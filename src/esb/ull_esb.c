/*
 * ULL ESB — Upper Link Layer for ESB ticker node
 *
 * Manages a periodic ticker that gives ESB scheduled radio access
 * alongside BLE connections. Follows the same ULL patterns as
 * ull_peripheral.c / ull_conn.c in the open BLE LL.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/sys/byteorder.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "hal/ticker.h"

#include "ticker/ticker.h"

#include "lll.h"
#include "lll_clock.h"

#include "ull_internal.h"

#include "ull_esb.h"
#include "lll_esb.h"
#include "esb_ticker.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ull_esb, CONFIG_ESB_LOG_LEVEL);

/* Singleton ULL ESB context */
static struct ull_esb esb_ctx;
static bool esb_ticker_running;

/* Runtime ESB ticker period. Seeded from Kconfig, but adjusted at runtime by
 * ull_esb_reconfigure() so the central can harmonize the ESB schedule with the
 * live BLE host connection interval. The ticker period can only be changed by
 * stopping and restarting the ticker (ticker_update cannot alter the periodic
 * interval), so reconfigure defers a stop+start to thread context.
 */
static uint32_t esb_ticker_interval_us = CONFIG_ESB_TICKER_INTERVAL_US;
static atomic_t esb_pending_interval_us;

/* Mayfly for dispatching LLL prepare from ULL context */
static memq_link_t mfy_lll_link;
static struct mayfly mfy_lll_prepare = {0U, 0U, &mfy_lll_link, NULL, NULL};
static struct lll_prepare_param prepare_param;

/* Deferred ticker start — order-independent rendezvous between
 * esb_start_rx() (SYS_INIT) and ull_esb_init() (bt_enable).
 * Whichever runs second submits the work; the handler only proceeds
 * when both preconditions are satisfied.
 */
static atomic_t ull_initialized;

/* Diagnostic counters — defined in lll_esb.c, logged from thread context here */
extern uint32_t esb_diag_prep_count;
extern uint32_t esb_diag_done_count;
extern uint32_t esb_diag_abort_active_count;
extern uint32_t esb_diag_abort_pipeline_count;
extern uint32_t esb_diag_err_count;
extern uint32_t esb_diag_defer_count;

static void esb_diag_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(esb_diag_work, esb_diag_work_handler);

static void esb_diag_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("ESB diag: prep=%u done=%u defer=%u abort_a=%u abort_p=%u err=%u",
		esb_diag_prep_count, esb_diag_done_count,
		esb_diag_defer_count,
		esb_diag_abort_active_count, esb_diag_abort_pipeline_count,
		esb_diag_err_count);
	k_work_schedule(&esb_diag_work, K_SECONDS(2));
}

static void deferred_ticker_start_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(deferred_ticker_start, deferred_ticker_start_handler);

static void ull_esb_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			      uint32_t remainder, uint16_t lazy, uint8_t force,
			      void *param);
static void ticker_op_cb(uint32_t status, void *param);

/**
 * @brief Deferred ticker start work handler.
 *
 * Runs on the system workqueue (thread context) after all SYS_INIT
 * entries have completed. Checks both preconditions:
 *   1. ULL is initialized (ticker infrastructure ready)
 *   2. ESB has a pending RX/TX action
 * Only starts the ticker when both are satisfied.
 */
static void deferred_ticker_start_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!atomic_get(&ull_initialized)) {
		LOG_DBG("ULL ESB: deferred start — ULL not ready yet");
		return;
	}

	if (!esb_ticker_is_active()) {
		LOG_DBG("ULL ESB: deferred start — ESB not pending");
		return;
	}

	if (esb_ticker_running) {
		return;
	}

	LOG_INF("ULL ESB: starting ticker (preconditions met)");
	int err = ull_esb_start();
	if (err && err != -EALREADY) {
		LOG_ERR("ULL ESB: deferred ticker start failed (%d)", err);
	} else {
		/* Start diagnostic reporting from thread context */
		k_work_schedule(&esb_diag_work, K_SECONDS(2));
	}
}

int ull_esb_init(void)
{
	ull_hdr_init(&esb_ctx.ull);
	lll_hdr_init(&esb_ctx.lll, &esb_ctx);

	esb_ticker_running = false;

	/* Mark ULL as ready. If esb_start_rx() already ran and submitted
	 * the work, the handler will see this flag and proceed.
	 */
	atomic_set(&ull_initialized, 1);

	LOG_INF("ULL ESB: initialized");

	/* Submit the deferred start with delay so USB CDC logging is ready.
	 * If ESB is already pending, the work handler will start the ticker.
	 * If not, esb_start_rx() will re-submit when it runs.
	 */
	k_work_schedule(&deferred_ticker_start, K_SECONDS(3));

	return 0;
}

void ull_esb_request_start(void)
{
	k_work_schedule(&deferred_ticker_start, K_SECONDS(3));
}

/**
 * @brief Apply a pending ESB ticker interval change (thread context).
 *
 * Stops the running ticker, swaps in the new period, and restarts it.
 * Both ticker ops are enqueued from TICKER_USER_ID_THREAD and are therefore
 * processed in order by the ticker job, so the stop completes before the
 * restart takes effect.
 */
static void esb_reconfigure_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint32_t want = (uint32_t)atomic_set(&esb_pending_interval_us, 0);

	if (want == 0U || want == esb_ticker_interval_us) {
		return;
	}

	bool was_running = esb_ticker_running;

	if (was_running) {
		(void)ull_esb_stop();
	}

	esb_ticker_interval_us = want;

	if (was_running) {
		int err = ull_esb_start();

		if (err && err != -EALREADY) {
			LOG_ERR("ULL ESB: restart after reconfigure failed (%d)", err);
			return;
		}
	}

	LOG_INF("ULL ESB: reconfigured interval -> %u us", want);
}
static K_WORK_DEFINE(esb_reconfigure_work, esb_reconfigure_handler);

int ull_esb_reconfigure(uint32_t interval_us)
{
	if (interval_us < 1250U) {
		interval_us = 1250U;
	} else if (interval_us > 20000U) {
		interval_us = 20000U;
	}

	if (interval_us == esb_ticker_interval_us) {
		return 0;
	}

	atomic_set(&esb_pending_interval_us, (atomic_val_t)interval_us);
	k_work_submit(&esb_reconfigure_work);

	return 0;
}

uint32_t ull_esb_get_interval_us(void)
{
	return esb_ticker_interval_us;
}

int ull_esb_start(void)
{
	uint32_t ticks_anchor;
	uint32_t ticks_periodic;
	uint32_t remainder_periodic;
	uint32_t ticks_slot;
	uint32_t ret;

	if (esb_ticker_running) {
		return -EALREADY;
	}

	ticks_periodic = HAL_TICKER_US_TO_TICKS(esb_ticker_interval_us);
	remainder_periodic = HAL_TICKER_REMAINDER(esb_ticker_interval_us);
	ticks_slot = HAL_TICKER_US_TO_TICKS(CONFIG_ESB_TICKER_SLOT_US);
	ticks_anchor = ticker_ticks_now_get();

	/* Reserve the slot in the ULL header */
	esb_ctx.ull.ticks_slot = ticks_slot;

	ret = ticker_start(TICKER_INSTANCE_ID_CTLR,
			   TICKER_USER_ID_THREAD,
			   TICKER_ID_USER_BASE,
			   ticks_anchor,
			   0U,                    /* ticks_first: start immediately */
			   ticks_periodic,
			   remainder_periodic,
			   TICKER_NULL_LAZY,
			   ticks_slot,
			   ull_esb_ticker_cb,
			   &esb_ctx,
			   ticker_op_cb,
			   NULL);

	if (ret != TICKER_STATUS_SUCCESS && ret != TICKER_STATUS_BUSY) {
		LOG_ERR("ULL ESB: ticker_start failed (%u)", ret);
		return -EIO;
	}

	esb_ticker_running = true;

	LOG_INF("ULL ESB: ticker started (interval=%u us, slot=%u us)",
		esb_ticker_interval_us, CONFIG_ESB_TICKER_SLOT_US);

	return 0;
}

int ull_esb_stop(void)
{
	uint32_t ret;

	if (!esb_ticker_running) {
		return -EALREADY;
	}

	ret = ticker_stop(TICKER_INSTANCE_ID_CTLR,
			  TICKER_USER_ID_THREAD,
			  TICKER_ID_USER_BASE,
			  ticker_op_cb,
			  NULL);

	if (ret != TICKER_STATUS_SUCCESS && ret != TICKER_STATUS_BUSY) {
		LOG_ERR("ULL ESB: ticker_stop failed (%u)", ret);
		return -EIO;
	}

	esb_ticker_running = false;

	LOG_INF("ULL ESB: ticker stopped");

	return 0;
}

void ull_esb_done(struct node_rx_event_done *done)
{
	struct ull_esb *ctx;
	struct ull_hdr *ull;

	ull = done->param;
	ctx = CONTAINER_OF(ull, struct ull_esb, ull);

	/* Note: rx_demux_event_done() already decremented the ULL
	 * reference count before dispatching here. Do NOT call
	 * ull_ref_dec() again — that would underflow the counter.
	 */

	/* Clean up the ESB event */
	esb_ticker_event_end();
}

struct ull_esb *ull_esb_get(void)
{
	return &esb_ctx;
}

/**
 * @brief Ticker timeout callback — fires every ESB_TICKER_INTERVAL_US.
 *
 * Runs in ULL_HIGH (RTC IRQ) context. Increments the ULL reference count
 * and dispatches the LLL prepare via a mayfly.
 *
 * Follows the same pattern as ull_periph_ticker_cb().
 */
static void ull_esb_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			      uint32_t remainder, uint16_t lazy, uint8_t force,
			      void *param)
{
	struct ull_esb *ctx = param;

	/* Check if ESB is still in a state that wants events */
	if (!esb_ticker_is_active()) {
		return;
	}

	/* Increment prepare reference count */
	ull_ref_inc(&ctx->ull);

	/* Build the prepare parameters for LLL */
	prepare_param.ticks_at_expire = ticks_at_expire;
	prepare_param.remainder = remainder;
	prepare_param.lazy = lazy;
	prepare_param.force = force;
	prepare_param.param = &ctx->lll;

	/* Dispatch LLL prepare via mayfly to LLL context */
	mfy_lll_prepare.fp = lll_esb_prepare;
	mfy_lll_prepare.param = &prepare_param;
	mayfly_enqueue(TICKER_USER_ID_ULL_HIGH, TICKER_USER_ID_LLL, 0,
		       &mfy_lll_prepare);
}

static void ticker_op_cb(uint32_t status, void *param)
{
	ARG_UNUSED(param);

	if (status != TICKER_STATUS_SUCCESS) {
		LOG_ERR("ULL ESB: ticker op failed (%u)", status);
	}
}
