/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* MPSL-backed replacement for nrfx's split per-clock drivers (nrfx >= 4.x as
 * consumed by Zephyr >= 4.5's clock_control_nrf_hfclk.c / _lfclk.c). Selected
 * together with CLOCK_CONTROL_NRF_FORCE_ALT so Zephyr's new drivers keep their
 * on/off-manager front end while the crystal is actually owned by MPSL, which
 * arbitrates it with the SoftDevice Controller's radio timeslots.
 *
 * Semantics match the monolithic-API shim (nrfx_clock_mpsl.c) used with the
 * legacy CONFIG_CLOCK_CONTROL_NRF driver on Zephyr <= 4.4:
 *  - HFCLK start/stop -> mpsl_clock_hfclk_src_request/release(XO)
 *  - LFCLK is always running under MPSL; start just reports "started".
 */
#include <nrfx_clock_hfclk.h>
#include <nrfx_clock_lfclk.h>

#include <mpsl.h>
#include <mpsl_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nrfx_clock_mpsl, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

static nrfx_clock_hfclk_event_handler_t hf_event_handler;
static nrfx_clock_lfclk_event_handler_t lf_event_handler;
static bool hf_initialized;
static bool lf_initialized;

static void mpsl_hfclk_src_callback(mpsl_clock_evt_type_t evt_type)
{
	switch (evt_type) {
	case MPSL_CLOCK_EVT_HFCLK_STARTED:
		if (hf_event_handler) {
			hf_event_handler();
		}
		break;
	default:
		/* XO tune / 24M events are not consumed by the Zephyr driver. */
		LOG_DBG("Unhandled MPSL HFCLK event: %d", evt_type);
	}
}

/* ---- HFCLK ---- */

int nrfx_clock_hfclk_init(nrfx_clock_hfclk_event_handler_t event_handler)
{
	hf_event_handler = event_handler;
	hf_initialized = true;
	return 0;
}

bool nrfx_clock_hfclk_init_check(void)
{
	return hf_initialized;
}

void nrfx_clock_hfclk_uninit(void)
{
	hf_initialized = false;
}

void nrfx_clock_hfclk_start(void)
{
	mpsl_clock_hfclk_src_request(MPSL_CLOCK_HF_SRC_XO, mpsl_hfclk_src_callback);
}

void nrfx_clock_hfclk_stop(void)
{
	mpsl_clock_hfclk_src_release(MPSL_CLOCK_HF_SRC_XO);
}

void nrfx_clock_hfclk_irq_handler(void)
{
	MPSL_IRQ_CLOCK_Handler();
}

/* ---- LFCLK ---- */

int nrfx_clock_lfclk_init(nrfx_clock_lfclk_event_handler_t event_handler)
{
	lf_event_handler = event_handler;
	lf_initialized = true;
	return 0;
}

bool nrfx_clock_lfclk_init_check(void)
{
	return lf_initialized;
}

void nrfx_clock_lfclk_uninit(void)
{
	lf_initialized = false;
}

void nrfx_clock_lfclk_start(void)
{
	/* LFCLK is always running under MPSL — just report started. */
	if (lf_event_handler) {
		lf_event_handler(NRFX_CLOCK_LFCLK_EVT_LFCLK_STARTED);
	}
}

void nrfx_clock_lfclk_stop(void)
{
	/* LFCLK cannot be stopped under MPSL. */
}

void nrfx_clock_lfclk_irq_handler(void)
{
	/* The POWER_CLOCK IRQ is dispatched to MPSL from the HFCLK handler. */
}
