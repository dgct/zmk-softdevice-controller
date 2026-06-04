/*
 * LLL ESB — Lower Link Layer for ESB ticker node
 *
 * Handles the radio event lifecycle: prepare callback (radio config + ESB
 * start), ISR dispatch, abort policy, and event completion signaling.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef LLL_ESB_H_
#define LLL_ESB_H_

/* lll.h has no include guards — include it from .c files before this header.
 * struct lll_hdr must be defined before including this header.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LLL ESB context.
 *
 * Embedded in struct ull_esb. Must have struct lll_hdr as first member.
 */
struct lll_esb {
	struct lll_hdr hdr;
};

/**
 * @brief LLL ESB prepare function (mayfly entry point).
 *
 * Called from the ULL ticker callback via mayfly in LLL context.
 * Requests HFXO and enters the LLL prepare pipeline.
 *
 * @param param  Pointer to struct lll_prepare_param.
 */
void lll_esb_prepare(void *param);

#ifdef __cplusplus
}
#endif

#endif /* LLL_ESB_H_ */
