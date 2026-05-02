/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>

#include <mpsl.h>
#include <mpsl_timeslot.h>
#include <hal/nrf_timer.h>

#include "multithreading_lock.h"
#include "soc_flash_nrf.h"

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_sync_mpsl);

/* The request length specified by the upper layers is only time required to do
 * the flash operations itself. Therefore we need to add some additional slack
 * to each timeslot request.
 */
#if defined(CONFIG_SOC_FLASH_NRF_PARTIAL_ERASE)
#define TIMESLOT_LENGTH_SLACK_US 1000
#else
#define TIMESLOT_LENGTH_SLACK_US 100
#endif

/* After this many us's, start using higher priority when requesting. */
#define TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US \
	CONFIG_SOC_FLASH_NRF_RADIO_SYNC_MPSL_NORMAL_PRIORITY_TIMEOUT_US

struct mpsl_flash_sync_context {
	struct k_sem timeout_sem;
	mpsl_timeslot_session_id_t session_id;
	uint32_t request_length_us;
	struct flash_op_desc *op_desc;
	mpsl_timeslot_request_t timeslot_request;
	mpsl_timeslot_signal_return_param_t return_param;
	int status;
	atomic_t timeout_occured;
};

static struct mpsl_flash_sync_context _context;

/**
 * Get time in microseconds since the beginning of the timeslot.
 * TIMER0 is started by MPSL at timeslot start on nRF52.
 */
static uint32_t get_timeslot_time_us(void)
{
	nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CAPTURE0);
	return nrf_timer_cc_get(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0);
}

static void reschedule_next_timeslot(void)
{
	_context.timeslot_request.params.earliest.priority =
		MPSL_TIMESLOT_PRIORITY_HIGH;
	_context.timeslot_request.params.earliest.timeout_us =
		MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US;

	int32_t ret = mpsl_timeslot_request(_context.session_id,
					    &_context.timeslot_request);
	__ASSERT_EVAL((void)ret, (void)ret, ret == 0,
		      "mpsl_timeslot_request failed: %d", ret);
}

static mpsl_timeslot_signal_return_param_t *
timeslot_callback(mpsl_timeslot_session_id_t session_id, uint32_t signal)
{
	int rc;

	__ASSERT_NO_MSG(session_id == _context.session_id);

	if (atomic_get(&_context.timeout_occured)) {
		return NULL;
	}

	switch (signal) {
	case MPSL_TIMESLOT_SIGNAL_START:
		rc = _context.op_desc->handler(_context.op_desc->context);

		if (rc != FLASH_OP_ONGOING) {
			_context.status = (rc == FLASH_OP_DONE) ? 0 : rc;
			_context.return_param.callback_action =
				MPSL_TIMESLOT_SIGNAL_ACTION_END;
		} else {
			/* Reset the priority back to normal after a successful
			 * timeslot.
			 */
			_context.timeslot_request.params.earliest.priority =
				MPSL_TIMESLOT_PRIORITY_NORMAL;
			_context.timeslot_request.params.earliest.timeout_us =
				TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US;

			_context.return_param.callback_action =
				MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			_context.return_param.params.request.p_next =
				&_context.timeslot_request;
		}
		break;

	case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
		/* All requests are done — operation complete. */
		k_sem_give(&_context.timeout_sem);
		return NULL;

	case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
		return NULL;

	case MPSL_TIMESLOT_SIGNAL_CANCELLED:
	case MPSL_TIMESLOT_SIGNAL_BLOCKED:
		/* Retry the failed request. */
		reschedule_next_timeslot();
		return NULL;

	default:
		__ASSERT(false, "unexpected signal: %u", signal);
		return NULL;
	}

	return &_context.return_param;
}

int nrf_flash_sync_init(void)
{
	LOG_DBG("");
	return k_sem_init(&_context.timeout_sem, 0, 1);
}

void nrf_flash_sync_set_context(uint32_t duration)
{
	_context.request_length_us = duration;
}

static bool is_in_fault_isr(void)
{
	int32_t irqn = __get_IPSR() - 16;

	return (irqn >= HardFault_IRQn && irqn <= UsageFault_IRQn);
}

bool nrf_flash_sync_is_required(void)
{
	return mpsl_is_initialized() && !is_in_fault_isr();
}

int nrf_flash_sync_exe(struct flash_op_desc *op_desc)
{
	LOG_DBG("duration: %u", _context.request_length_us);

	int errcode = MULTITHREADING_LOCK_ACQUIRE();

	__ASSERT_NO_MSG(errcode == 0);

	int32_t ret = mpsl_timeslot_session_open(timeslot_callback,
						 &_context.session_id);
	MULTITHREADING_LOCK_RELEASE();

	if (ret < 0) {
		LOG_ERR("mpsl_timeslot_session_open failed: %d", ret);
		return -ENOMEM;
	}

	mpsl_timeslot_request_t *req = &_context.timeslot_request;

	req->request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST;
	req->params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE;
	req->params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL;
	req->params.earliest.length_us =
		_context.request_length_us + TIMESLOT_LENGTH_SLACK_US;
	req->params.earliest.timeout_us = TIMESLOT_TIMEOUT_PRIORITY_NORMAL_US;

	_context.op_desc = op_desc;
	_context.status = -ETIMEDOUT;
	atomic_clear(&_context.timeout_occured);

	__ASSERT_NO_MSG(k_sem_count_get(&_context.timeout_sem) == 0);

	errcode = MULTITHREADING_LOCK_ACQUIRE();
	__ASSERT_NO_MSG(errcode == 0);
	ret = mpsl_timeslot_request(_context.session_id, req);
	__ASSERT_EVAL((void)ret, (void)ret, ret == 0,
		      "mpsl_timeslot_request failed: %d", ret);
	MULTITHREADING_LOCK_RELEASE();

	if (k_sem_take(&_context.timeout_sem, K_MSEC(FLASH_TIMEOUT_MS)) < 0) {
		LOG_ERR("timeout");
		atomic_set(&_context.timeout_occured, 1);
	}

	/* Close the session (cancels any pending timeslot). */
	errcode = MULTITHREADING_LOCK_ACQUIRE();
	__ASSERT_NO_MSG(errcode == 0);
	mpsl_timeslot_session_close(_context.session_id);
	MULTITHREADING_LOCK_RELEASE();

	/* Reset the semaphore after timeout, in case the operation did
	 * complete before closing the session.
	 */
	if (atomic_get(&_context.timeout_occured)) {
		k_sem_reset(&_context.timeout_sem);
	}

	return _context.status;
}

void nrf_flash_sync_get_timestamp_begin(void)
{
	/* Not needed for the MPSL backend — time limit is checked
	 * relative to timeslot start via TIMER0.
	 */
}

bool nrf_flash_sync_check_time_limit(uint32_t iteration)
{
	uint32_t now_us = get_timeslot_time_us();
	uint32_t time_per_iteration_us = now_us / iteration;

	return now_us + time_per_iteration_us >= _context.request_length_us;
}
