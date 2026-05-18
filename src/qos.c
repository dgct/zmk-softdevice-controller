/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * BLE QoS channel map filter for ZMK + SoftDevice Controller.
 *
 * Uses SDC's per-connection-event CRC statistics, channel survey energy
 * measurements, and Nordic's chmap_filter library as inputs to a priority-
 * score system that ranks all 37 BLE data channels by quality.
 *
 * The channel map is constructed by including the best-scoring channels,
 * with continuous EWMA-smoothed scores replacing the old binary block/
 * unblock approach.  Host channel maps (from macOS/Windows/dongle) are
 * incorporated as a soft penalty rather than a hard AND, eliminating the
 * "host map merge would leave N channels" failure mode.
 *
 * Only runs on the central (the side that can call bt_le_set_chan_map).
 *
 * Adaptive interval: when CRC errors are detected the processing interval
 * drops to QOS_INTERVAL_FAST and then doubles each cycle (exponential
 * back-off) until it returns to QOS_INTERVAL_BASE.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include <sdc_hci_vs.h>

#include "chmap_filter.h"

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>

/* Zephyr v4.4 compat: bt_hci_cmd_create -> bt_hci_cmd_alloc */
static inline struct net_buf *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len) {
	(void)opcode; (void)param_len;
	return bt_hci_cmd_alloc(K_FOREVER);
}

LOG_MODULE_REGISTER(zmk_ble_qos, CONFIG_ZMK_BLE_QOS_LOG_LEVEL);

#define QOS_THREAD_PRIORITY K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO)
#define QOS_INTERVAL_BASE   CONFIG_ZMK_BLE_QOS_INTERVAL
#define QOS_INTERVAL_FAST   CONFIG_ZMK_BLE_QOS_INTERVAL_FAST

/* Score weights (Q8 fixed point: 256 = 1.0) */
#define W_ENERGY_DEFAULT CONFIG_ZMK_BLE_QOS_SCORE_W_ENERGY
#define W_CRC            CONFIG_ZMK_BLE_QOS_SCORE_W_CRC
#define W_HOST_PENALTY   CONFIG_ZMK_BLE_QOS_SCORE_W_HOST
#define W_FILTER_PENALTY CONFIG_ZMK_BLE_QOS_SCORE_W_FILTER

/* Noise floor below which energy contributes zero score (Q8) */
#define NOISE_FLOOR_Q8   ((int16_t)CONFIG_ZMK_BLE_QOS_SCORE_NOISE_FLOOR << 8)

/* Score threshold: channels scoring above this are excluded if we have
 * enough channels.  Expressed in raw score units. */
#define BLOCK_THRESHOLD  CONFIG_ZMK_BLE_QOS_SCORE_BLOCK_THRESHOLD

/* EWMA shift factors: alpha = 1/(1<<shift).  Larger shift = slower. */
#define EWMA_ENERGY_SHIFT 2  /* alpha = 1/4, ~4-sample half-life */
#define EWMA_CRC_SHIFT    3  /* alpha = 1/8, ~8-sample half-life */

#define MIN_CHANNELS     CONFIG_ZMK_BLE_QOS_MIN_CHANNEL_COUNT

/* Map-diff hysteresis: only send LL_CHANNEL_MAP_IND when the new map
 * differs from the current by at least this many channels.  Prevents
 * churn under broadband interference where the "best N" set fluctuates. */
#define MAP_DIFF_MIN  3

/* Adaptive energy weight (LTE-U pattern): CRC confirms/denies energy
 * predictions, tuning trust in the energy sensor at runtime.
 * Wiener-optimal step size tied to adaptive interval. */
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
#define W_ENERGY_MIN             CONFIG_ZMK_BLE_QOS_SCORE_W_ENERGY_MIN
#define W_ENERGY_MAX             CONFIG_ZMK_BLE_QOS_SCORE_W_ENERGY_MAX
#define ENERGY_CORR_THRESHOLD_Q8 2560  /* 10 dB above noise floor */
#define CRC_CORR_THRESHOLD_BASE  8     /* ~3% error rate in Q8 */
#define ADAPT_HOLDOFF_CYCLES     16    /* 2x CRC EWMA half-life */
#define STEP_UP_BASE             2     /* Wiener-optimal for persistent */
#define STEP_DOWN_BASE           1     /* Neyman-Pearson 2:1 asymmetry */
#define MIN_ADAPT_CHANNELS       5
#endif

static K_THREAD_STACK_DEFINE(qos_stack, CONFIG_ZMK_BLE_QOS_STACK_SIZE);
static struct k_thread qos_thread;

static uint8_t chmap_inst_buf[CHMAP_FILTER_INST_SIZE]
        __aligned(CHMAP_FILTER_INST_ALIGN);
static struct chmap_instance *chmap_inst;

static atomic_t processing;
static atomic_t crc_errors_seen;
static atomic_t burst_requested;
static atomic_t reinit_requested;
static bool reporting_enabled;
static uint32_t current_interval_ms = QOS_INTERVAL_FAST;
static enum zmk_activity_state last_activity_state = ZMK_ACTIVITY_SLEEP;

/*
 * Per-channel priority score state.
 *
 * energy_ewma: EWMA of survey energy in Q8 fixed point (dBm * 256).
 *   Initialized to NOISE_FLOOR_Q8 (neutral).  Updated each survey report.
 *
 * crc_ratio_ewma: EWMA of CRC error ratio in Q8 (0 = perfect, 256 = 100%
 *   errors).  Updated each QoS processing cycle from accumulated counts.
 *
 * crc_ok_acc / crc_err_acc: per-channel CRC accumulators filled by the
 *   BT RX thread (on_vs_evt) and consumed by the QoS thread.  Saturating
 *   uint8_t — overflow caps at 255 which preserves the ratio.
 */
struct channel_score {
        int16_t energy_ewma;
        int16_t crc_ratio_ewma;
        uint8_t crc_ok_acc;
        uint8_t crc_err_acc;
};

static struct channel_score ch_scores[37];

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
static int8_t survey_energy[40];
static atomic_t survey_data_ready;
static bool survey_enabled;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_POWER_CONTROL_AUTO)
static bool power_control_configured;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
static struct bt_conn *path_loss_conn;
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
static struct bt_conn *host_conn;
#endif

static uint8_t last_applied_map[5];

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
static int16_t w_energy = W_ENERGY_DEFAULT;
static uint8_t adapt_holdoff;
#else
#define w_energy W_ENERGY_DEFAULT
#endif

/* Central-role connection handles for CRC event filtering.
 * Only split-link (central-role) CRC data should feed into chmap_filter
 * and the scoring system.  Host/dongle-link (peripheral-role) CRC errors
 * would pollute channel ratings because that link's channel map is
 * controlled by the remote central, not by us.
 *
 * Design: sentinel-based (no count variable). Each slot is either a valid
 * handle or HANDLE_UNUSED. Writers (connected/disconnected on sysworkq)
 * and readers (on_vs_evt on BT RX thread) operate on individual atomic slots.
 * Worst-case race: reader misses a just-added handle (drops one CRC sample)
 * or matches a just-removed handle (feeds one extra sample). Both harmless. */
#define HANDLE_UNUSED (-1)
static atomic_t central_handles[CONFIG_BT_MAX_CONN] = {
        [0 ... CONFIG_BT_MAX_CONN - 1] = ATOMIC_INIT(HANDLE_UNUSED)
};

static int apply_filter_params(void);

/*
 * VS HCI event callback — called from the BT RX thread whenever the
 * SoftDevice Controller generates a vendor-specific event.
 *
 * Handles:
 * - QOS_CONN_EVENT_REPORT: per-connection-event CRC ok/error counts.
 *   Fed to both chmap_filter (for WiFi detection) and our per-channel
 *   accumulators (for priority scoring).
 * - QOS_CHANNEL_SURVEY_REPORT: RF energy per channel.
 *
 * Uses an atomic flag to cheaply skip updates while the QoS thread
 * is processing — avoids lock contention on the hot path.
 */
static bool on_vs_evt(struct net_buf_simple *buf)
{
        uint8_t *subevent_code;
        sdc_hci_subevent_vs_qos_conn_event_report_t *evt;

        subevent_code = net_buf_simple_pull_mem(buf, sizeof(*subevent_code));

        switch (*subevent_code) {
        case SDC_HCI_SUBEVENT_VS_QOS_CONN_EVENT_REPORT:
                if (atomic_get(&processing)) {
                        return true;
                }
                evt = (void *)buf->data;
                /* Only feed central-role (split-link) CRC data.
                 * Skip peripheral-role (host/dongle-link) events. */
                {
                        bool is_central = false;
                        for (uint8_t i = 0; i < ARRAY_SIZE(central_handles); i++) {
                                if (atomic_get(&central_handles[i]) ==
                                    (atomic_val_t)evt->conn_handle) {
                                        is_central = true;
                                        break;
                                }
                        }
                        if (!is_central) {
                                return true;
                        }
                }
                /* Feed Nordic's chmap_filter (preserves WiFi detection) */
                chmap_filter_crc_update(chmap_inst,
                        evt->channel_index,
                        evt->crc_ok_count,
                        evt->crc_error_count);
                /* Accumulate for priority score system.
                 * Saturating add — overflow caps at 255. */
                if (evt->channel_index < 37) {
                        struct channel_score *s = &ch_scores[evt->channel_index];
                        uint16_t new_ok = (uint16_t)s->crc_ok_acc +
                                          evt->crc_ok_count;
                        uint16_t new_err = (uint16_t)s->crc_err_acc +
                                           evt->crc_error_count;
                        s->crc_ok_acc = (new_ok > 255) ? 255 : (uint8_t)new_ok;
                        s->crc_err_acc = (new_err > 255) ? 255 :
                                         (uint8_t)new_err;
                }
                if (evt->crc_error_count > 0) {
                        atomic_set(&crc_errors_seen, true);
                }
                return true;
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
        case SDC_HCI_SUBEVENT_VS_QOS_CHANNEL_SURVEY_REPORT: {
                sdc_hci_subevent_vs_qos_channel_survey_report_t *srv =
                        (void *)buf->data;
                memcpy(survey_energy, srv->channel_energy, sizeof(survey_energy));
                atomic_set(&survey_data_ready, true);
                return true;
        }
#endif
        default:
                return false;
        }
}

/*
 * Enable QoS connection event reporting via VS HCI command.
 */
static int enable_conn_event_reporting(void)
{
        struct net_buf *buf;
        int err;

        buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_QOS_CONN_EVENT_REPORT_ENABLE,
                                sizeof(sdc_hci_cmd_vs_qos_conn_event_report_enable_t));
        if (!buf) {
                LOG_ERR("Failed to allocate HCI command buffer");
                return -ENOMEM;
        }

        sdc_hci_cmd_vs_qos_conn_event_report_enable_t *cmd = net_buf_add(buf,
                sizeof(sdc_hci_cmd_vs_qos_conn_event_report_enable_t));
        cmd->enable = 1;

        err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_QOS_CONN_EVENT_REPORT_ENABLE,
                                   buf, NULL);
        if (err) {
                LOG_ERR("Failed to enable QoS conn event reporting: %d", err);
        } else {
                LOG_INF("QoS conn event reporting enabled");
        }

        return err;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_POWER_CONTROL_AUTO)
static int configure_power_control(void)
{
        struct net_buf *buf;
        int err;

        buf = bt_hci_cmd_create(
                SDC_HCI_OPCODE_CMD_VS_SET_POWER_CONTROL_REQUEST_PARAMS,
                sizeof(sdc_hci_cmd_vs_set_power_control_request_params_t));
        if (!buf) {
                LOG_ERR("Failed to allocate HCI buf for power control params");
                return -ENOMEM;
        }

        sdc_hci_cmd_vs_set_power_control_request_params_t *cmd = net_buf_add(
                buf, sizeof(sdc_hci_cmd_vs_set_power_control_request_params_t));

        cmd->auto_enable = 1;
        cmd->apr_enable = 1;
        cmd->beta = CONFIG_ZMK_BLE_POWER_CONTROL_BETA;
        cmd->lower_limit = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_LIMIT;
        cmd->upper_limit = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_LIMIT;
        cmd->lower_target_rssi = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_TARGET;
        cmd->upper_target_rssi = CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_TARGET;
        cmd->wait_period_ms = CONFIG_ZMK_BLE_POWER_CONTROL_WAIT_PERIOD_MS;
        cmd->apr_margin = CONFIG_ZMK_BLE_POWER_CONTROL_APR_MARGIN;

        err = bt_hci_cmd_send_sync(
                SDC_HCI_OPCODE_CMD_VS_SET_POWER_CONTROL_REQUEST_PARAMS,
                buf, NULL);
        if (err) {
                LOG_ERR("Failed to set power control params: %d", err);
        } else {
                LOG_INF("LE Power Control autonomous mode enabled "
                        "(RSSI range [%d, %d] dBm, wait %u ms)",
                        CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_LOWER_LIMIT,
                        CONFIG_ZMK_BLE_POWER_CONTROL_RSSI_UPPER_LIMIT,
                        CONFIG_ZMK_BLE_POWER_CONTROL_WAIT_PERIOD_MS);
        }

        return err;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
static int enable_channel_survey(void)
{
        struct net_buf *buf;
        int err;

        buf = bt_hci_cmd_create(SDC_HCI_OPCODE_CMD_VS_QOS_CHANNEL_SURVEY_ENABLE,
                                sizeof(sdc_hci_cmd_vs_qos_channel_survey_enable_t));
        if (!buf) {
                LOG_ERR("Failed to allocate HCI command buffer for survey");
                return -ENOMEM;
        }

        sdc_hci_cmd_vs_qos_channel_survey_enable_t *cmd = net_buf_add(buf,
                sizeof(sdc_hci_cmd_vs_qos_channel_survey_enable_t));
        cmd->enable = 1;
        cmd->interval_us = CONFIG_ZMK_BLE_QOS_SURVEY_INTERVAL_US;

        err = bt_hci_cmd_send_sync(SDC_HCI_OPCODE_CMD_VS_QOS_CHANNEL_SURVEY_ENABLE,
                                   buf, NULL);
        if (err) {
                LOG_ERR("Failed to enable channel survey: %d", err);
        } else {
                LOG_INF("Channel survey enabled (interval %u us)",
                        CONFIG_ZMK_BLE_QOS_SURVEY_INTERVAL_US);
        }

        return err;
}
#endif

/*
 * Count set bits in a 5-byte channel map (37 data channels).
 */
static int chmap_popcount(const uint8_t map[5])
{
        int count = 0;

        for (int i = 0; i < 5; i++) {
                uint8_t b = map[i];
                while (b) {
                        count++;
                        b &= b - 1;
                }
        }
        return count;
}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
/*
 * Read the channel map of the host/dongle (peripheral-role) connection.
 * Returns the map the remote central assigned via its AFH algorithm.
 *
 * For OS mode (macOS/Windows/Linux): reflects WiFi coexistence and
 * independent RF assessment from the host's perspective.
 * For dongle mode: reflects the dongle's local RF survey (USB-powered,
 * no WiFi radio to confound, different physical location at USB port).
 *
 * Both provide complementary intelligence about the RF environment.
 */
static int read_host_chan_map(uint8_t out[5])
{
        struct bt_conn *conn = host_conn;
        struct net_buf *buf;
        struct net_buf *rsp = NULL;
        int err;

        if (!conn) {
                return -ENOTCONN;
        }

        conn = bt_conn_ref(conn);
        if (!conn) {
                return -ENOTCONN;
        }

        uint16_t handle;

        err = bt_hci_get_conn_handle(conn, &handle);
        if (err) {
                goto out;
        }

        buf = bt_hci_cmd_create(BT_HCI_OP_LE_READ_CHAN_MAP,
                                sizeof(struct bt_hci_cp_le_read_chan_map));
        if (!buf) {
                err = -ENOMEM;
                goto out;
        }

        struct bt_hci_cp_le_read_chan_map *cp = net_buf_add(buf,
                sizeof(struct bt_hci_cp_le_read_chan_map));
        cp->handle = sys_cpu_to_le16(handle);

        err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_CHAN_MAP, buf, &rsp);
        if (err) {
                goto out;
        }

        struct bt_hci_rp_le_read_chan_map *rp = (void *)rsp->data;

        if (rp->status) {
                err = -EIO;
        } else {
                memcpy(out, rp->ch_map, 5);
        }

        net_buf_unref(rsp);
out:
        bt_conn_unref(conn);
        return err;
}
#endif /* CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE */

/*
 * Reset all channel scores to neutral state.
 * Called on init and on wake-from-sleep reinit.
 */
static void scores_reset(void)
{
        for (int ch = 0; ch < 37; ch++) {
                ch_scores[ch].energy_ewma = NOISE_FLOOR_Q8;
                ch_scores[ch].crc_ratio_ewma = 0;
                ch_scores[ch].crc_ok_acc = 0;
                ch_scores[ch].crc_err_acc = 0;
        }
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
        w_energy = W_ENERGY_DEFAULT;
        adapt_holdoff = 0;
#endif
}

/*
 * Compute the priority score for a single channel.
 * Higher score = worse channel = lower priority to include in map.
 *
 * Components:
 * 1. Energy above noise floor (proactive: survey-based)
 * 2. CRC error ratio (reactive: confirmed packet loss)
 * 3. Host/dongle map penalty (external intelligence from remote AFH)
 * 4. chmap_filter penalty (Nordic's WiFi pattern detection)
 *
 * The host penalty leverages the asymmetry between modes:
 * - Dongle: physically close, battery-unlimited, sees USB3/desktop noise
 * - OS: may be distant, has WiFi coexistence data, sees its local RF
 * Both contribute useful but different perspective on channel quality.
 */
static int16_t compute_channel_score(int ch,
                                     const uint8_t *filter_map,
                                     const uint8_t *host_map,
                                     bool host_map_valid)
{
        int32_t score = 0;
        struct channel_score *s = &ch_scores[ch];

        /* Energy component: energy above noise floor, weighted.
         * Survey captures WiFi/microwave/USB3 interference proactively.
         * w_energy is adaptive when CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY. */
        int16_t energy_above = s->energy_ewma - NOISE_FLOOR_Q8;
        if (energy_above > 0) {
                score += ((int32_t)energy_above * w_energy) >> 8;
        }

        /* CRC component: error ratio, weighted heavily.
         * CRC errors are confirmed packet loss — no false positives. */
        score += ((int32_t)s->crc_ratio_ewma * W_CRC) >> 8;

        /* Host/dongle map penalty: soft signal from remote central's AFH.
         * Not a hard veto — just makes these channels sort lower in the
         * priority queue.  If we're channel-starved, these still get
         * included (their absolute score might still be low enough). */
        if (host_map_valid && !(host_map[ch / 8] & (1U << (ch % 8)))) {
                score += W_HOST_PENALTY;
        }

        /* chmap_filter penalty: Nordic's library detected this channel as
         * bad.  Preserves WiFi-width pattern detection (contiguous blocks
         * spanning ~10 BLE channels matching 802.11 channel widths). */
        if (!(filter_map[ch / 8] & (1U << (ch % 8)))) {
                score += W_FILTER_PENALTY;
        }

        /* Clamp to int16_t range */
        if (score > INT16_MAX) {
                score = INT16_MAX;
        }
        return (int16_t)score;
}

/*
 * Build channel map from priority-sorted scores.
 * Includes channels in ascending score order (best first) until we hit
 * the block threshold and have at least MIN_CHANNELS.
 *
 * This is the priority queue: channels compete on composite quality.
 * N=37 so insertion sort is optimal (cache-friendly, ~666 comparisons
 * worst case, no heap overhead).
 */
static void build_scored_map(const uint8_t *filter_map,
                             const uint8_t *host_map,
                             bool host_map_valid,
                             uint8_t out_map[5])
{
        struct { uint8_t ch; int16_t score; } ranked[37];

        /* Score all channels */
        for (int ch = 0; ch < 37; ch++) {
                ranked[ch].ch = ch;
                ranked[ch].score = compute_channel_score(ch, filter_map,
                                                         host_map,
                                                         host_map_valid);
        }

        /* Insertion sort ascending (best = lowest score first) */
        for (int i = 1; i < 37; i++) {
                __typeof__(ranked[0]) tmp = ranked[i];
                int j = i - 1;
                while (j >= 0 && ranked[j].score > tmp.score) {
                        ranked[j + 1] = ranked[j];
                        j--;
                }
                ranked[j + 1] = tmp;
        }

        /* Build map: include best channels up to threshold */
        memset(out_map, 0, 5);
        int included = 0;

        for (int i = 0; i < 37; i++) {
                if (included >= MIN_CHANNELS &&
                    ranked[i].score >= BLOCK_THRESHOLD) {
                        break;
                }
                uint8_t ch = ranked[i].ch;
                out_map[ch / 8] |= (1U << (ch % 8));
                included++;
        }

        LOG_DBG("Score map: %d ch (worst_in=%d, first_out=%d)",
                included,
                included > 0 ? ranked[included - 1].score : 0,
                included < 37 ? ranked[included].score : 0);
}

/*
 * Process accumulated CRC data into EWMA scores.
 * Called once per QoS cycle while processing==true (safe from concurrent
 * accumulation by on_vs_evt).
 */
static void scores_process_crc(void)
{
        for (int ch = 0; ch < 37; ch++) {
                struct channel_score *s = &ch_scores[ch];
                uint8_t ok = s->crc_ok_acc;
                uint8_t err_cnt = s->crc_err_acc;

                /* Reset accumulators atomically w.r.t. processing flag */
                s->crc_ok_acc = 0;
                s->crc_err_acc = 0;

                uint16_t total = (uint16_t)ok + err_cnt;
                if (total == 0) {
                        /* No data this cycle for this channel.
                         * At 2ms CI, each channel visited ~once per 74ms.
                         * In 100ms fast cycle: 0-2 samples per channel.
                         * Leave EWMA unchanged — no news is not good news,
                         * it's just no news. */
                        continue;
                }

                /* Compute ratio in Q8: 0 = perfect, 256 = all errors */
                int16_t ratio_q8 = ((int32_t)err_cnt << 8) / total;

                /* EWMA update: asymmetric — fast attack (alpha=1/2),
                 * slow decay (alpha=1/8).  Blocks bad channels in ~100ms,
                 * holds block for ~600ms after interference clears. */
                int16_t diff = ratio_q8 - s->crc_ratio_ewma;
                int shift = (diff > 0) ? 1 : EWMA_CRC_SHIFT;
                s->crc_ratio_ewma += diff >> shift;
        }
}

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
/*
 * Process survey energy data into EWMA scores.
 * Called when survey_data_ready flag was set by on_vs_evt.
 */
static void scores_process_survey(void)
{
        int8_t local_energy[40];

        memcpy(local_energy, survey_energy, sizeof(local_energy));

        for (int ch = 0; ch < 37; ch++) {
                if (local_energy[ch] == 127) {
                        /* No measurement (SDC sentinel value) */
                        continue;
                }

                /* Convert to Q8 and EWMA update: alpha = 1/4 */
                int16_t sample_q8 = (int16_t)local_energy[ch] << 8;
                struct channel_score *s = &ch_scores[ch];

                s->energy_ewma += (sample_q8 - s->energy_ewma)
                                  >> EWMA_ENERGY_SHIFT;
        }

        LOG_DBG("Survey energy processed into scores");
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
/*
 * Adapt the energy weight based on correlation between energy detection
 * and CRC error rates on active channels.
 *
 * LTE-U pattern: reactive metrics (CRC) tune trust in proactive sensor
 * (energy detection).  Wiener-optimal step size scales with the adaptive
 * interval — transient interference triggers burst mode (100ms), which
 * naturally yields 10x faster convergence via more cycles per second.
 *
 * Channel-count-normalized CRC threshold prevents the starvation feedback
 * loop: at fewer active channels, collision probability rises even with
 * constant interference, so the threshold for "CRC confirms energy" is
 * raised proportionally.
 */
static void adapt_energy_weight(void)
{
        if (adapt_holdoff > 0) {
                adapt_holdoff--;
                return;
        }

        int n_active = 0;
        int true_pos = 0;
        int false_pos = 0;

        /* Count active channels for CRC threshold normalization */
        for (int ch = 0; ch < 37; ch++) {
                if (last_applied_map[ch / 8] & (1U << (ch % 8))) {
                        n_active++;
                }
        }

        if (n_active < MIN_ADAPT_CHANNELS) {
                return;
        }

        /* CRC threshold scaled by channel count: more channels = lower
         * baseline collision rate = lower threshold for significance.
         * CRC_CORR_THRESHOLD_BASE is calibrated for 37 channels. */
        int16_t crc_thresh = (int16_t)(CRC_CORR_THRESHOLD_BASE * 37
                                       / n_active);

        int n_classified = 0;

        for (int ch = 0; ch < 37; ch++) {
                /* Only examine channels in the active map —
                 * blocked channels have no CRC data, which would
                 * bias toward false-negative detection. */
                if (!(last_applied_map[ch / 8] & (1U << (ch % 8)))) {
                        continue;
                }

                struct channel_score *s = &ch_scores[ch];
                int16_t energy_above = s->energy_ewma - NOISE_FLOOR_Q8;
                bool energy_high = (energy_above > ENERGY_CORR_THRESHOLD_Q8);
                bool crc_high = (s->crc_ratio_ewma > crc_thresh);

                if (energy_high && crc_high) {
                        true_pos++;
                } else if (energy_high && !crc_high) {
                        false_pos++;
                }
                n_classified++;
        }

        if (n_classified < MIN_ADAPT_CHANNELS) {
                return;
        }

        /* Step scales with interval: at burst (100ms), step is 10x
         * baseline — Wiener-optimal for transient interference. */
        int scale = QOS_INTERVAL_BASE / current_interval_ms;
        if (scale < 1) {
                scale = 1;
        }

        int16_t prev = w_energy;

        if (true_pos > false_pos) {
                w_energy += STEP_UP_BASE * scale;
                if (w_energy > W_ENERGY_MAX) {
                        w_energy = W_ENERGY_MAX;
                }
        } else if (false_pos > true_pos) {
                w_energy -= STEP_DOWN_BASE * scale;
                if (w_energy < W_ENERGY_MIN) {
                        w_energy = W_ENERGY_MIN;
                }
        }

        if (w_energy != prev) {
                LOG_INF("LTE-U: w_e %d->%d (tp=%d fp=%d n=%d scale=%d)",
                        prev, w_energy, true_pos, false_pos,
                        n_classified, scale);
        }
}
#endif /* CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY */

/*
 * QoS thread — runs at the lowest application priority.
 *
 * Adaptive interval logic:
 * - Baseline: sleeps for QOS_INTERVAL_BASE (default 1000ms)
 * - On CRC errors: drops to QOS_INTERVAL_FAST (default 100ms)
 * - Each clean cycle: interval doubles toward BASE
 *
 * Each cycle:
 * 1. Adapt interval based on CRC errors / burst request
 * 2. Run chmap_filter_process() for WiFi pattern detection
 * 3. Process CRC accumulators into score EWMAs
 * 4. Process survey energy into score EWMAs (if available)
 * 5. Read host/dongle channel map (if connected)
 * 6. Build priority-scored channel map
 * 7. Apply if changed
 */
static void qos_thread_fn(void *p1, void *p2, void *p3)
{
        ARG_UNUSED(p1);
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);

        while (true) {
                int err;

                k_sleep(K_MSEC(current_interval_ms));

                if (!reporting_enabled) {
                        continue;
                }

                /* Adapt interval: burst on CRC errors or wake-from-sleep */
                bool burst = atomic_cas(&burst_requested, true, false);
                bool errors = atomic_cas(&crc_errors_seen, true, false);
                if (burst || errors) {
                        if (current_interval_ms > QOS_INTERVAL_FAST) {
                                LOG_INF("QoS: %s, interval %u -> %u ms",
                                        errors ? "CRC errors" : "wake burst",
                                        current_interval_ms, QOS_INTERVAL_FAST);
                        }
                        current_interval_ms = QOS_INTERVAL_FAST;
                } else if (current_interval_ms < QOS_INTERVAL_BASE) {
                        uint32_t next = current_interval_ms * 2;
                        if (next > QOS_INTERVAL_BASE) {
                                next = QOS_INTERVAL_BASE;
                        }
                        current_interval_ms = next;
                }

                atomic_set(&processing, true);

                /* Handle reinit request from wake listener.
                 * Must happen inside processing gate to prevent concurrent
                 * CRC updates from on_vs_evt hitting a half-init'd state. */
                if (atomic_cas(&reinit_requested, true, false)) {
                        int rinit_err = chmap_filter_instance_init(chmap_inst,
                                                                   sizeof(chmap_inst_buf));
                        if (rinit_err) {
                                LOG_ERR("chmap_filter re-init failed: %d",
                                        rinit_err);
                        } else {
                                apply_filter_params();
                                memset(last_applied_map, 0,
                                       sizeof(last_applied_map));
                        }
                        scores_reset();
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
                        atomic_set(&survey_data_ready, false);
#endif
                }

                /* Step 2: Run chmap_filter for WiFi pattern detection.
                 * Its suggested map feeds into our scoring as a penalty. */
                chmap_filter_process(chmap_inst);

                /* Step 3: Process CRC accumulators into score EWMAs.
                 * Must complete while processing==true so on_vs_evt
                 * doesn't race with accumulator reads/resets. */
                scores_process_crc();

                atomic_set(&processing, false);

                /* Step 4: Process survey energy if new data arrived */
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
                if (atomic_cas(&survey_data_ready, true, false)) {
                        scores_process_survey();
                }
#endif

                /* Step 4b: Adapt energy weight from CRC/energy correlation */
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
                adapt_energy_weight();
#endif

                /* Get chmap_filter's suggested map as penalty signal.
                 * Channels it blocked get W_FILTER_PENALTY added. */
                uint8_t *filter_map =
                        chmap_filter_suggested_map_get(chmap_inst);

                /* Step 5: Read host/dongle channel map (soft penalty) */
                uint8_t host_map[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
                bool host_map_valid = false;

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
                if (host_conn && !read_host_chan_map(host_map)) {
                        host_map_valid = true;
                }
#endif

                /* Step 6: Build priority-scored channel map */
                uint8_t final_map[5];
                build_scored_map(filter_map, host_map, host_map_valid,
                                 final_map);

                /* Step 7: Map-diff hysteresis — only update if the new
                 * map differs by at least MAP_DIFF_MIN channels.
                 * Prevents LL_CHANNEL_MAP_IND churn under broadband. */
                int diff_count = 0;
                for (int i = 0; i < 5; i++) {
                        uint8_t xor = final_map[i] ^ last_applied_map[i];
                        while (xor) {
                                diff_count++;
                                xor &= xor - 1;
                        }
                }
                if (diff_count < MAP_DIFF_MIN) {
                        continue;
                }

                int count = chmap_popcount(final_map);
                err = bt_le_set_chan_map(final_map);
                if (err) {
                        LOG_WRN("bt_le_set_chan_map failed: %d", err);
                } else {
                        memcpy(last_applied_map, final_map, 5);
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_ADAPTIVE_ENERGY)
                        adapt_holdoff = ADAPT_HOLDOFF_CYCLES;
#endif
                        LOG_INF("Channel map updated: %d ch "
                                "(interval=%u ms, w_e=%d)", count,
                                current_interval_ms, w_energy);
                }

                chmap_filter_suggested_map_confirm(chmap_inst);
        }
}

/*
 * Connection callback — enable QoS reporting on first connection.
 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
        if (err) {
                return;
        }

        /* Track central-role connection handles for CRC event filtering */
        {
                struct bt_conn_info info;
                if (!bt_conn_get_info(conn, &info) &&
                    info.role == BT_CONN_ROLE_CENTRAL) {
                        uint16_t handle;
                        if (!bt_hci_get_conn_handle(conn, &handle)) {
                                for (uint8_t i = 0; i < ARRAY_SIZE(central_handles); i++) {
                                        if (atomic_cas(&central_handles[i],
                                                       HANDLE_UNUSED,
                                                       (atomic_val_t)handle)) {
                                                LOG_INF("QoS: tracking central handle "
                                                        "0x%04x (slot %u)", handle, i);
                                                break;
                                        }
                                }
                        }
                }
        }

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE) || \
    IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
        struct bt_conn_info info;

        if (!bt_conn_get_info(conn, &info) &&
            info.role == BT_CONN_ROLE_PERIPHERAL) {
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
                if (!host_conn) {
                        host_conn = bt_conn_ref(conn);
                        LOG_INF("QoS: host/dongle connection tracked "
                                "for score penalty");
                }
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
                if (!path_loss_conn) {
                        static const struct bt_conn_le_path_loss_reporting_param
                                plm_params = {
                                .high_threshold =
                                        CONFIG_ZMK_BLE_PATH_LOSS_HIGH_THRESHOLD,
                                .high_hysteresis =
                                        CONFIG_ZMK_BLE_PATH_LOSS_HIGH_HYSTERESIS,
                                .low_threshold =
                                        CONFIG_ZMK_BLE_PATH_LOSS_LOW_THRESHOLD,
                                .low_hysteresis =
                                        CONFIG_ZMK_BLE_PATH_LOSS_LOW_HYSTERESIS,
                                .min_time_spent =
                                        CONFIG_ZMK_BLE_PATH_LOSS_MIN_TIME_SPENT,
                        };
                        int ret = bt_conn_le_set_path_loss_mon_param(conn,
                                                                     &plm_params);
                        if (ret) {
                                LOG_WRN("Path loss params failed: %d", ret);
                        } else {
                                ret = bt_conn_le_set_path_loss_mon_enable(conn,
                                                                          true);
                                if (ret) {
                                        LOG_WRN("Path loss enable failed: %d",
                                                 ret);
                                } else {
                                        path_loss_conn = bt_conn_ref(conn);
                                        LOG_INF("Path loss monitoring enabled "
                                                "(high=%u low=%u dB)",
                                                CONFIG_ZMK_BLE_PATH_LOSS_HIGH_THRESHOLD,
                                                CONFIG_ZMK_BLE_PATH_LOSS_LOW_THRESHOLD);
                                }
                        }
                }
#endif /* ZMK_BLE_PATH_LOSS_MONITORING */
        }
#endif /* HOST_MAP_MERGE || PATH_LOSS_MONITORING */

        if (!reporting_enabled) {
                int ret = enable_conn_event_reporting();
                if (!ret) {
                        reporting_enabled = true;
                }
        }

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_CHANNEL_SURVEY)
        if (!survey_enabled) {
                int ret = enable_channel_survey();
                if (!ret) {
                        survey_enabled = true;
                }
        }
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_POWER_CONTROL_AUTO)
        if (!power_control_configured) {
                int ret = configure_power_control();
                if (!ret) {
                        power_control_configured = true;
                }
        }
#endif
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
        /* Remove central-role handle from tracking array */
        {
                uint16_t handle;
                if (!bt_hci_get_conn_handle(conn, &handle)) {
                        for (uint8_t i = 0; i < ARRAY_SIZE(central_handles); i++) {
                                if (atomic_cas(&central_handles[i],
                                               (atomic_val_t)handle,
                                               HANDLE_UNUSED)) {
                                        LOG_INF("QoS: removed central handle "
                                                "0x%04x (slot %u)", handle, i);
                                        break;
                                }
                        }
                }
        }

#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_HOST_MAP_MERGE)
        if (conn == host_conn) {
                LOG_INF("QoS: host/dongle connection lost");
                bt_conn_unref(host_conn);
                host_conn = NULL;
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
        if (conn == path_loss_conn) {
                LOG_INF("Path loss monitoring: host disconnected");
                bt_conn_unref(path_loss_conn);
                path_loss_conn = NULL;
        }
#endif
}

#if IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
static void path_loss_threshold_cb(struct bt_conn *conn,
        const struct bt_conn_le_path_loss_threshold_report *report)
{
#if CONFIG_ZMK_BLE_QOS_LOG_LEVEL > 0
        static const char *zone_names[] = {
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_LOW] = "LOW",
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_MIDDLE] = "MIDDLE",
                [BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_HIGH] = "HIGH",
                [BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE] = "UNAVAILABLE",
        };

        const char *zone = (report->zone <= BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE)
                ? zone_names[report->zone] : "UNKNOWN";

        if (report->zone == BT_CONN_LE_PATH_LOSS_ZONE_ENTERED_HIGH) {
                LOG_WRN("Path loss HIGH zone: %u dB — host link degraded",
                        report->path_loss);
        } else if (report->zone == BT_CONN_LE_PATH_LOSS_ZONE_UNAVAILABLE) {
                LOG_WRN("Path loss UNAVAILABLE — peer may not support "
                        "LE Power Control");
        } else {
                LOG_INF("Path loss zone: %s (%u dB)", zone, report->path_loss);
        }
#else
        ARG_UNUSED(conn);
        ARG_UNUSED(report);
#endif
}
#endif

BT_CONN_CB_DEFINE(qos_conn_cb) = {
        .connected = connected_cb,
        .disconnected = disconnected_cb,
#if IS_ENABLED(CONFIG_ZMK_BLE_PATH_LOSS_MONITORING)
        .path_loss_threshold_report = path_loss_threshold_cb,
#endif
};

static int apply_filter_params(void)
{
        struct chmap_filter_params params;

        chmap_filter_params_get(chmap_inst, &params);
        params.min_channel_count = CONFIG_ZMK_BLE_QOS_MIN_CHANNEL_COUNT;
        params.eval_keepout_duration = CONFIG_ZMK_BLE_QOS_EVAL_KEEPOUT_DURATION;

        int err = chmap_filter_params_set(chmap_inst, &params);
        if (err) {
                LOG_ERR("chmap_filter_params_set failed: %d", err);
        } else {
                LOG_INF("QoS filter params: min_ch=%u keepout=%u",
                        params.min_channel_count, params.eval_keepout_duration);
        }
        return err;
}

static int qos_init(void)
{
        int err;

        chmap_filter_init();
        chmap_inst = (struct chmap_instance *)chmap_inst_buf;

        err = chmap_filter_instance_init(chmap_inst, sizeof(chmap_inst_buf));
        if (err) {
                LOG_ERR("chmap_filter_instance_init failed: %d", err);
                return err;
        }

        apply_filter_params();
        scores_reset();

        err = bt_hci_register_vnd_evt_cb(on_vs_evt);
        if (err) {
                LOG_ERR("Failed to register VS event callback: %d", err);
                return err;
        }

        k_thread_create(&qos_thread, qos_stack,
                        K_THREAD_STACK_SIZEOF(qos_stack),
                        qos_thread_fn, NULL, NULL, NULL,
                        QOS_THREAD_PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&qos_thread, "ble_qos");

        LOG_INF("BLE QoS priority-score system initialized");

        return 0;
}

SYS_INIT(qos_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Activity state listener — triggers a QoS burst on wake from sleep.
 * Idle->active transitions are ignored (frequent with 1s idle timeout).
 * Sleep->active: RF environment may have changed (different room, etc).
 *
 * With REINIT_ON_WAKE: re-initializes chmap_filter AND resets all scores.
 * All channels start at neutral, scores reconverge within ~10 seconds
 * (EWMA half-lives: energy ~4s, CRC ~8s).
 */
static int qos_activity_listener(const zmk_event_t *eh)
{
        struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
        if (ev == NULL) {
                return -ENOTSUP;
        }

        enum zmk_activity_state prev = last_activity_state;
        last_activity_state = ev->state;

        if (ev->state == ZMK_ACTIVITY_ACTIVE && prev == ZMK_ACTIVITY_SLEEP) {
#if IS_ENABLED(CONFIG_ZMK_BLE_QOS_REINIT_ON_WAKE)
                LOG_INF("QoS: wake from sleep, requesting reinit + burst");
                atomic_set(&reinit_requested, true);
#else
                LOG_INF("QoS: wake from sleep, requesting burst");
#endif
                atomic_set(&burst_requested, true);
                k_wakeup(&qos_thread);
        }

        return 0;
}

ZMK_LISTENER(ble_qos_activity, qos_activity_listener);
ZMK_SUBSCRIPTION(ble_qos_activity, zmk_activity_state_changed);
