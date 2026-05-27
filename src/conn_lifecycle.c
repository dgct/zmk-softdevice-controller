/*
 * Copyright (c) 2026 dgct
 * SPDX-License-Identifier: MIT
 *
 * Unified connection-lifecycle state machine for ZMK split central.
 *
 * Consolidates SCI (Shorter Connection Intervals) and subrating into a
 * single callback-driven state machine.  Every SCI transition is confirmed
 * by le_param_updated_cb (since ZMK's Zephyr lacks conn_rate_changed).
 *
 * Manages two independent instances:
 *   - Split link (central role): full SCI + subrating
 *   - Host link (peripheral role): subrating only (if supported)
 *
 * Replaces the former sci.c + subrating.c pair.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/sys/byteorder.h>

/* Zephyr v4.4 compat: bt_hci_cmd_create -> bt_hci_cmd_alloc */
static inline struct net_buf *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len) {
    (void)opcode; (void)param_len;
    return bt_hci_cmd_alloc(K_FOREVER);
}

#include <sdc_hci_cmd_le.h>
#include <sdc_hci_cmd_controller_baseband.h>

#include <zmk/sdc/sci.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/events/activity_state_changed.h>
#if IS_ENABLED(CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY)
#include <zmk/events/ble_host_param_request.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(conn_lifecycle, CONFIG_ZMK_BLE_SCI_LOG_LEVEL);

/* ──── SCI parameters ──── */
#define SCI_TARGET_US           CONFIG_ZMK_BLE_SCI_INTERVAL_US
#define SCI_SUPERVISION_TO      CONFIG_ZMK_BLE_SCI_SUPERVISION_TIMEOUT
#define SCI_SWITCH_DELAY_MS     CONFIG_ZMK_BLE_SCI_SWITCH_DELAY_MS
#define SCI_PHY_SWITCH_DELAY_MS CONFIG_ZMK_BLE_SCI_PHY_SWITCH_DELAY_MS
#define SCI_RETRIGGER_DELAY_MS  CONFIG_ZMK_BLE_SCI_RETRIGGER_DELAY_MS
#define SCI_FSU_TO_CRR_DELAY_MS 200
#define MIN_FLUSH_SLOTS         CONFIG_ZMK_BLE_SCI_FLUSH_TIMEOUT_MIN_SLOTS
#define SCI_MAX_RETRIES         5

/* Frame space update constants */
#define FSU_PHY_2M              0x02
#define FSU_SPACING_ACL_IFS     0x0001

/* ──── Subrating parameters ──── */
#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#define SUBRATE_TIMEOUT            CONFIG_ZMK_BLE_SUBRATE_TIMEOUT
#define SUBRATE_DORMANT_DELAY_MS   CONFIG_ZMK_BLE_SUBRATE_DORMANT_DELAY

#define SUBRATE_ACTIVE_MIN         CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MIN
#define SUBRATE_ACTIVE_MAX         CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX
#define SUBRATE_ACTIVE_CN          CONFIG_ZMK_BLE_SUBRATE_ACTIVE_CN
#define SUBRATE_ACTIVE_MAX_LATENCY CONFIG_ZMK_BLE_SUBRATE_ACTIVE_MAX_LATENCY
#define SUBRATE_IDLE_MIN           CONFIG_ZMK_BLE_SUBRATE_IDLE_MIN
#define SUBRATE_IDLE_MAX           CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX
#define SUBRATE_IDLE_CN            CONFIG_ZMK_BLE_SUBRATE_IDLE_CN
#define SUBRATE_IDLE_MAX_LATENCY   CONFIG_ZMK_BLE_SUBRATE_IDLE_MAX_LATENCY
#define SUBRATE_DORMANT_MIN        CONFIG_ZMK_BLE_SUBRATE_DORMANT_MIN
#define SUBRATE_DORMANT_MAX        CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX
#define SUBRATE_DORMANT_CN         CONFIG_ZMK_BLE_SUBRATE_DORMANT_CN
#define SUBRATE_DORMANT_MAX_LATENCY CONFIG_ZMK_BLE_SUBRATE_DORMANT_MAX_LATENCY

/* Compile-time safety checks */
BUILD_ASSERT(SUBRATE_ACTIVE_MAX >= SUBRATE_ACTIVE_MIN, "ACTIVE_MAX >= ACTIVE_MIN");
BUILD_ASSERT(SUBRATE_IDLE_MAX >= SUBRATE_IDLE_MIN, "IDLE_MAX >= IDLE_MIN");
BUILD_ASSERT(SUBRATE_DORMANT_MAX >= SUBRATE_DORMANT_MIN, "DORMANT_MAX >= DORMANT_MIN");
BUILD_ASSERT(SUBRATE_ACTIVE_CN < SUBRATE_ACTIVE_MAX, "ACTIVE_CN < ACTIVE_MAX");
BUILD_ASSERT(SUBRATE_IDLE_CN < SUBRATE_IDLE_MAX, "IDLE_CN < IDLE_MAX");
BUILD_ASSERT(SUBRATE_DORMANT_CN < SUBRATE_DORMANT_MAX, "DORMANT_CN < DORMANT_MAX");

static const struct bt_conn_le_subrate_param active_params = {
    .subrate_min = SUBRATE_ACTIVE_MIN, .subrate_max = SUBRATE_ACTIVE_MAX,
    .max_latency = SUBRATE_ACTIVE_MAX_LATENCY, .continuation_number = SUBRATE_ACTIVE_CN,
    .supervision_timeout = SUBRATE_TIMEOUT,
};
static const struct bt_conn_le_subrate_param idle_params = {
    .subrate_min = SUBRATE_IDLE_MIN, .subrate_max = SUBRATE_IDLE_MAX,
    .max_latency = SUBRATE_IDLE_MAX_LATENCY, .continuation_number = SUBRATE_IDLE_CN,
    .supervision_timeout = SUBRATE_TIMEOUT,
};
static const struct bt_conn_le_subrate_param dormant_params = {
    .subrate_min = SUBRATE_DORMANT_MIN, .subrate_max = SUBRATE_DORMANT_MAX,
    .max_latency = SUBRATE_DORMANT_MAX_LATENCY, .continuation_number = SUBRATE_DORMANT_CN,
    .supervision_timeout = SUBRATE_TIMEOUT,
};

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_CONN_PARAM_DORMANT)
#define HOST_DORMANT_INT_MIN    CONFIG_ZMK_BLE_HOST_CONN_DORMANT_INT_MIN
#define HOST_DORMANT_INT_MAX    CONFIG_ZMK_BLE_HOST_CONN_DORMANT_INT_MAX
#define HOST_DORMANT_LATENCY    CONFIG_ZMK_BLE_HOST_CONN_DORMANT_LATENCY
#define HOST_DORMANT_TIMEOUT    CONFIG_ZMK_BLE_HOST_CONN_DORMANT_TIMEOUT

#if !IS_ENABLED(CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY)
static const struct bt_le_conn_param host_dormant_params = {
    .interval_min = HOST_DORMANT_INT_MIN, .interval_max = HOST_DORMANT_INT_MAX,
    .latency = HOST_DORMANT_LATENCY, .timeout = HOST_DORMANT_TIMEOUT,
};
static const struct bt_le_conn_param host_active_params = {
    .interval_min = CONFIG_BT_PERIPHERAL_PREF_MIN_INT,
    .interval_max = CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
    .latency = CONFIG_BT_PERIPHERAL_PREF_LATENCY,
    .timeout = CONFIG_BT_PERIPHERAL_PREF_TIMEOUT,
};
#endif
#endif /* ZMK_BLE_HOST_CONN_PARAM_DORMANT */

#endif /* BT_SUBRATING && ZMK_SPLIT_ROLE_CENTRAL */

/* ──── State machine ──── */

enum conn_state {
    CS_IDLE,
    CS_PHY_PENDING,     /* PHY 2M requested (central only) */
    CS_TP_WAIT,         /* Waiting for TP init (split central only) */
    CS_FSU_PENDING,     /* Frame Space Update sent */
    CS_CRR_PENDING,     /* Connection Rate Request sent */
    CS_CRR_VERIFYING,   /* CRR sent, waiting for le_param_updated confirmation */
    CS_SCI_ACTIVE,      /* SCI confirmed via param update */
    CS_OPERATIONAL,     /* Steady state — subrating active */
};

static const char *state_name(enum conn_state s) {
    static const char *n[] = {
        "IDLE","PHY_PENDING","TP_WAIT","FSU_PENDING",
        "CRR_PENDING","CRR_VERIFYING","SCI_ACTIVE","OPERATIONAL"
    };
    return (s < ARRAY_SIZE(n)) ? n[s] : "?";
}

/* Per-connection lifecycle instance */
struct lifecycle {
    struct bt_conn *conn;
    enum conn_state state;
    uint32_t achieved_ci_us;
    bool fsu_done;
    bool tp_ready;
    bool encrypted;  /* Set when security_changed confirms L2+ */
    int retries;
    struct k_work_delayable step_work;
    struct k_spinlock lock;
};

/* Split link instance (central role) */
static struct lifecycle split_lc;

/* Subrating state (split link only, central role) */
#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
enum subrate_tier { TIER_ACTIVE, TIER_IDLE, TIER_DORMANT };
static enum subrate_tier current_tier = TIER_IDLE;
static bool tier_confirmed = true;
static uint16_t last_confirmed_factor = 1;

/* Deferred tier — stores the desired tier when set_tier() is gated
 * during SCI setup. Replayed by on_operational(). */
static enum subrate_tier pending_tier;
static bool has_pending_tier;

/* Bounded retry counter for tier_retry_handler */
#define TIER_MAX_RETRIES 5
static uint8_t tier_retry_count;

static void dormant_timer_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dormant_work, dormant_timer_handler);
static void tier_retry_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tier_retry_work, tier_retry_handler);
#endif

/* Forward declarations */
static void split_step_handler(struct k_work *work);
static void do_fsu(struct lifecycle *lc);
static void do_crr(struct lifecycle *lc);

/* ──── Guarded state transitions ──── */

static bool lc_transition(struct lifecycle *lc, enum conn_state from,
                          enum conn_state to, struct bt_conn *conn) {
    k_spinlock_key_t key = k_spin_lock(&lc->lock);
    bool ok = (lc->state == from && lc->conn == conn);
    if (ok) {
        lc->state = to;
        LOG_INF("[split] -> %s", state_name(to));
    }
    k_spin_unlock(&lc->lock, key);
    return ok;
}

/* ──── Raw HCI helpers (ZMK lacks host-layer SCI APIs) ──── */

static int send_frame_space_update(struct bt_conn *conn) {
    uint16_t conn_handle;
    int err = bt_hci_get_conn_handle(conn, &conn_handle);
    if (err) return err;

    struct net_buf *buf = bt_hci_cmd_create(
        SDC_HCI_OPCODE_CMD_LE_FRAME_SPACE_UPDATE,
        sizeof(sdc_hci_cmd_le_frame_space_update_t));
    if (!buf) return -ENOMEM;

    sdc_hci_cmd_le_frame_space_update_t *cmd =
        net_buf_add(buf, sizeof(*cmd));
    cmd->conn_handle = conn_handle;
    cmd->frame_space_min = 0;
    cmd->frame_space_max = 150;
    cmd->phys = FSU_PHY_2M;
    cmd->spacing_types = FSU_SPACING_ACL_IFS;

    err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_FRAME_SPACE_UPDATE, buf);
    if (err) LOG_ERR("FSU failed: %d", err);
    else LOG_INF("FSU requested (min=0, max=150, 2M PHY)");
    return err;
}

static int send_conn_rate_request(struct bt_conn *conn, uint32_t interval_us,
                                  uint16_t latency, uint16_t timeout) {
    uint16_t conn_handle;
    int err = bt_hci_get_conn_handle(conn, &conn_handle);
    if (err) return err;

    uint16_t ci_val = (uint16_t)(interval_us / 125);
    struct net_buf *buf = bt_hci_cmd_create(
        SDC_HCI_OPCODE_CMD_LE_CONN_RATE_REQUEST,
        sizeof(sdc_hci_cmd_le_conn_rate_request_t));
    if (!buf) return -ENOMEM;

    sdc_hci_cmd_le_conn_rate_request_t *cmd =
        net_buf_add(buf, sizeof(*cmd));
    cmd->conn_handle = conn_handle;
    cmd->conn_interval_min = ci_val;
    cmd->conn_interval_max = ci_val;
#if IS_ENABLED(CONFIG_BT_SUBRATING)
    cmd->subrate_min = SUBRATE_ACTIVE_MIN;
    cmd->subrate_max = SUBRATE_ACTIVE_MAX;
#else
    cmd->subrate_min = 1;
    cmd->subrate_max = 1;
#endif
    cmd->max_latency = latency;
    cmd->continuation_number = 0;
    cmd->supervision_timeout = timeout;
    cmd->min_ce_length = 0;
    cmd->max_ce_length = 0;

    err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_CONN_RATE_REQUEST, buf);
    if (err) LOG_ERR("CRR failed: %d", err);
    else LOG_INF("CRR sent: CI=%u (0.125ms units, %u us)", ci_val, interval_us);
    return err;
}

static int send_flush_timeout(struct bt_conn *conn, uint16_t flush_slots) {
    uint16_t conn_handle;
    int err = bt_hci_get_conn_handle(conn, &conn_handle);
    if (err) return err;

    struct net_buf *buf = bt_hci_cmd_create(
        SDC_HCI_OPCODE_CMD_CB_WRITE_AUTOMATIC_FLUSH_TIMEOUT,
        sizeof(sdc_hci_cmd_cb_write_automatic_flush_timeout_t));
    if (!buf) return -ENOMEM;

    sdc_hci_cmd_cb_write_automatic_flush_timeout_t *cmd =
        net_buf_add(buf, sizeof(*cmd));
    cmd->conn_handle = conn_handle;
    cmd->flush_timeout = flush_slots;

    err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_CB_WRITE_AUTOMATIC_FLUSH_TIMEOUT, buf);
    if (err) LOG_ERR("Flush timeout write failed: %d", err);
    else LOG_INF("Flush timeout: %u slots", flush_slots);
    return err;
}

/* ──── SCI procedure helpers ──── */

static void do_fsu(struct lifecycle *lc) {
    int err = send_frame_space_update(lc->conn);
    if (err) {
        LOG_WRN("FSU failed: %d — skipping to CRR", err);
        lc_transition(lc, CS_FSU_PENDING, CS_CRR_PENDING, lc->conn);
        lc->fsu_done = true;
        do_crr(lc);
        return;
    }
    /* ZMK lacks FSU callback — wait fixed delay, then advance */
    k_work_schedule(&lc->step_work, K_MSEC(SCI_FSU_TO_CRR_DELAY_MS));
}

/* Forward-declare: applies correct subrating tier on entering OPERATIONAL */
#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static void on_operational(void);
#else
static inline void on_operational(void) {}
#endif

static void do_crr(struct lifecycle *lc) {
    int err = send_conn_rate_request(lc->conn, SCI_TARGET_US, 0,
                                     SCI_SUPERVISION_TO);
    if (err) {
        k_spinlock_key_t key = k_spin_lock(&lc->lock);
        int retries = ++lc->retries;
        k_spin_unlock(&lc->lock, key);

        if (retries >= SCI_MAX_RETRIES) {
            LOG_ERR("CRR failed after %d attempts", retries);
            lc_transition(lc, CS_CRR_PENDING, CS_OPERATIONAL, lc->conn);
            lc->achieved_ci_us = 7500;
            on_operational();
        } else {
            LOG_WRN("CRR failed: %d — retry %d/%d", err, retries, SCI_MAX_RETRIES);
            k_work_schedule(&lc->step_work, K_MSEC(SCI_SWITCH_DELAY_MS));
        }
        return;
    }
    /* Advance to VERIFYING — le_param_updated_cb will confirm */
    lc_transition(lc, CS_CRR_PENDING, CS_CRR_VERIFYING, lc->conn);
    /* Fallback: if param update never arrives, retry */
    k_work_schedule(&lc->step_work, K_MSEC(2000));
}

/* Activate SCI after confirmed CI */
static void sci_activate(struct lifecycle *lc) {
    lc_transition(lc, CS_SCI_ACTIVE, CS_OPERATIONAL, lc->conn);

    if (IS_ENABLED(CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA) && MIN_FLUSH_SLOTS > 0) {
        uint16_t ci_slots = (uint16_t)(lc->achieved_ci_us / 625);
        uint16_t slots = MAX(MIN_FLUSH_SLOTS, ci_slots * 6);
        send_flush_timeout(lc->conn, slots);
    }

    on_operational();
}

/* ──── Split link step work handler ──── */

static void split_step_handler(struct k_work *work) {
    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    enum conn_state state = split_lc.state;
    struct bt_conn *conn = split_lc.conn;
    k_spin_unlock(&split_lc.lock, key);

    if (!conn) return;

    switch (state) {
    case CS_PHY_PENDING:
        /* PHY 2M not achieved — operate without SCI */
        LOG_WRN("PHY 2M timeout — operating without SCI");
        lc_transition(&split_lc, CS_PHY_PENDING, CS_OPERATIONAL, conn);
        split_lc.achieved_ci_us = 7500;
        on_operational();
        break;

    case CS_TP_WAIT:
        /* TP init timeout — proceed to SCI anyway */
        LOG_WRN("TP_WAIT timeout — proceeding to SCI");
        if (split_lc.fsu_done) {
            lc_transition(&split_lc, CS_TP_WAIT, CS_CRR_PENDING, conn);
            do_crr(&split_lc);
        } else {
            lc_transition(&split_lc, CS_TP_WAIT, CS_FSU_PENDING, conn);
            do_fsu(&split_lc);
        }
        break;

    case CS_FSU_PENDING:
        /* FSU delay elapsed — advance to CRR */
        split_lc.fsu_done = true;
        lc_transition(&split_lc, CS_FSU_PENDING, CS_CRR_PENDING, conn);
        do_crr(&split_lc);
        break;

    case CS_CRR_PENDING:
        /* Retry delay — send CRR */
        do_crr(&split_lc);
        break;

    case CS_CRR_VERIFYING: {
        /* le_param_updated never confirmed CRR — retry */
        key = k_spin_lock(&split_lc.lock);
        int retries = ++split_lc.retries;
        k_spin_unlock(&split_lc.lock, key);

        if (retries >= SCI_MAX_RETRIES) {
            LOG_ERR("SCI confirmation timeout after %d retries", SCI_MAX_RETRIES);
            lc_transition(&split_lc, CS_CRR_VERIFYING, CS_OPERATIONAL, conn);
            split_lc.achieved_ci_us = 7500;
            on_operational();
        } else {
            LOG_WRN("CRR verify timeout — retry %d/%d", retries, SCI_MAX_RETRIES);
            lc_transition(&split_lc, CS_CRR_VERIFYING, CS_CRR_PENDING, conn);
            do_crr(&split_lc);
        }
        break;
    }

    default:
        break;
    }
}

/* ──── TP ready signal (called from PS/2 driver) ──── */

void conn_lifecycle_tp_ready(void) {
    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    split_lc.tp_ready = true;
    bool waiting = (split_lc.state == CS_TP_WAIT && split_lc.conn);
    k_spin_unlock(&split_lc.lock, key);

    if (waiting) {
        LOG_INF("TP ready signal received — advancing to SCI");
        k_work_cancel_delayable(&split_lc.step_work);
        struct bt_conn *conn = split_lc.conn;
        if (split_lc.fsu_done) {
            lc_transition(&split_lc, CS_TP_WAIT, CS_CRR_PENDING, conn);
            do_crr(&split_lc);
        } else {
            lc_transition(&split_lc, CS_TP_WAIT, CS_FSU_PENDING, conn);
            do_fsu(&split_lc);
        }
    }
}

/* ──── BLE connection callbacks ──── */

static void le_phy_updated_cb(struct bt_conn *conn,
                              struct bt_conn_le_phy_info *param) {
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    LOG_INF("PHY [%s]: tx=%u, rx=%u", addr_str, param->tx_phy, param->rx_phy);

    if (param->tx_phy != BT_GAP_LE_PHY_2M) return;

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) return;

    /* Defense-in-depth: PHY update should only arrive after encryption
     * (we initiate it from security_changed). If somehow it arrives
     * before encryption, ignore it — security_changed will re-trigger
     * the PHY update and we'll get here again. */
    if (!split_lc.encrypted) {
        LOG_WRN("PHY 2M arrived before encryption — deferring");
        return;
    }

    /* Only handle the split link */
    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    bool idle = (split_lc.state == CS_IDLE || split_lc.state == CS_PHY_PENDING);
    k_spin_unlock(&split_lc.lock, key);
    if (!idle && split_lc.conn == conn) return;

    /* Take ref if first time */
    if (!split_lc.conn) {
        split_lc.conn = bt_conn_ref(conn);
    }
    split_lc.retries = 0;

    /* PHY ready → enter TP_WAIT (or skip if tp already ready) */
    if (split_lc.tp_ready) {
        LOG_INF("PHY 2M + TP already ready — starting SCI");
        lc_transition(&split_lc, CS_PHY_PENDING, CS_FSU_PENDING, conn);
        do_fsu(&split_lc);
    } else {
        LOG_INF("PHY 2M ready — waiting for TP init (%d ms timeout)",
                SCI_PHY_SWITCH_DELAY_MS);
        lc_transition(&split_lc, CS_PHY_PENDING, CS_TP_WAIT, conn);
        /* Fallback timeout if TP_READY never arrives */
        k_work_schedule(&split_lc.step_work,
                        K_MSEC(SCI_PHY_SWITCH_DELAY_MS));
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
    if (conn == split_lc.conn) {
        k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
        split_lc.state = CS_IDLE;
        split_lc.fsu_done = false;
        split_lc.tp_ready = false;
        split_lc.encrypted = false;
        split_lc.achieved_ci_us = 0;
        struct bt_conn *old = split_lc.conn;
        split_lc.conn = NULL;
        k_spin_unlock(&split_lc.lock, key);

        k_work_cancel_delayable(&split_lc.step_work);
#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
        /* Cancel stale subrating workqueues — tier_retry_work or dormant_work
         * could fire after disconnect and call bt_conn_foreach on dead conns.
         * Reset all subrating state so the next connection starts clean. */
        k_work_cancel_delayable(&tier_retry_work);
        k_work_cancel_delayable(&dormant_work);
        tier_retry_count = 0;
        tier_confirmed = true;
        has_pending_tier = false;
        last_confirmed_factor = 1;
        current_tier = TIER_IDLE;
#endif
        if (old) bt_conn_unref(old);
    }
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
    if (err || level < BT_SECURITY_L2) {
        return;
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    split_lc.encrypted = true;
    k_spin_unlock(&split_lc.lock, key);

    LOG_INF("Split link encrypted — conn_lifecycle armed");
}

static bool le_param_req_cb(struct bt_conn *conn,
                            struct bt_le_conn_param *param) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_CENTRAL) {
        return true;  /* always accept host link param updates */
    }

    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    enum conn_state state = split_lc.state;
    k_spin_unlock(&split_lc.lock, key);

    if (state == CS_OPERATIONAL || state == CS_SCI_ACTIVE ||
        state == CS_CRR_PENDING || state == CS_CRR_VERIFYING) {
        LOG_INF("Rejecting split CPUQ (CI=%u-%u) — SCI active/pending",
                param->interval_min, param->interval_max);
        return false;
    }
    return true;
}

static void le_param_updated_cb(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) return;

    uint32_t ci_us = (uint32_t)interval * 1250;
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    LOG_INF("Params [%s]: CI=%u.%02ums, lat=%u, to=%u",
            addr_str, (interval * 125) / 100, (interval * 125) % 100,
            latency, timeout);

    if (info.role != BT_CONN_ROLE_CENTRAL || conn != split_lc.conn) return;

    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    enum conn_state state = split_lc.state;
    k_spin_unlock(&split_lc.lock, key);

    /* CRR_VERIFYING: le_param_updated is our confirmation signal.
     * If CI dropped to sub-7.5ms, CRR succeeded. */
    if (state == CS_CRR_VERIFYING && ci_us <= 10000) {
        k_work_cancel_delayable(&split_lc.step_work);
        split_lc.achieved_ci_us = ci_us;
        LOG_INF("SCI confirmed: CI=%u us", ci_us);
        lc_transition(&split_lc, CS_CRR_VERIFYING, CS_SCI_ACTIVE, conn);
        sci_activate(&split_lc);
        return;
    }

    /* CRR_VERIFYING but CI didn't drop — CRR failed silently.
     * Retry on the next step_work timeout (already scheduled). */
    if (state == CS_CRR_VERIFYING && ci_us > 10000) {
        LOG_WRN("CRR did not take effect (CI=%u us) — will retry", ci_us);
        return;
    }

    /* OPERATIONAL: keyboard parked us (CI went up) */
    if (state == CS_OPERATIONAL && ci_us > 10000 &&
        split_lc.achieved_ci_us < 7500) {
        split_lc.achieved_ci_us = ci_us;
        LOG_INF("SCI deactivated — parked at CI=%u us", ci_us);
    }

    /* OPERATIONAL IDLE: keyboard woke us — re-establish SCI */
    if (state == CS_OPERATIONAL && ci_us <= 10000 && latency == 0 &&
        split_lc.achieved_ci_us >= 7500) {
        LOG_INF("Keyboard active — re-triggering SCI");
        split_lc.retries = 0;
        enum conn_state next = split_lc.fsu_done ? CS_CRR_PENDING : CS_FSU_PENDING;
        lc_transition(&split_lc, CS_OPERATIONAL, next, conn);
        k_work_schedule(&split_lc.step_work, K_MSEC(SCI_RETRIGGER_DELAY_MS));
    }
}

/* ──── Subrating (split central only) ──── */

#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static void apply_subrate_to_conn(struct bt_conn *conn, void *data) {
    const struct bt_conn_le_subrate_param *params = data;
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) return;
    if (info.role == BT_CONN_ROLE_CENTRAL && info.state == BT_CONN_STATE_CONNECTED) {
        int err = bt_conn_le_subrate_request(conn, params);
        if (err) {
            LOG_WRN("Subrate request failed: %d", err);
        }
    }
}

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_CONN_PARAM_DORMANT) && !IS_ENABLED(CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY)
static void apply_conn_param_to_host(struct bt_conn *conn, void *data) {
    const struct bt_le_conn_param *params = data;
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) return;
    if (info.role == BT_CONN_ROLE_PERIPHERAL && info.state == BT_CONN_STATE_CONNECTED) {
        int err = bt_conn_le_param_update(conn, params);
        if (err && err != -EALREADY) {
            LOG_WRN("Host param update failed: %d", err);
        }
    }
}
#endif

static void set_tier(enum subrate_tier tier) {
    /* Only allow subrating changes when SCI is settled */
    k_spinlock_key_t key = k_spin_lock(&split_lc.lock);
    enum conn_state state = split_lc.state;
    k_spin_unlock(&split_lc.lock, key);

    if (state != CS_OPERATIONAL && state != CS_IDLE) {
        /* Store desired tier for replay by on_operational().
         * Last-writer-wins: multiple changes during SCI setup collapse
         * to the most recent activity state, which is correct. */
        LOG_DBG("Deferring subrate tier %d — state=%s", tier, state_name(state));
        pending_tier = tier;
        has_pending_tier = true;
        return;
    }

    if (tier == current_tier && tier_confirmed) return;

    enum subrate_tier prev_tier = current_tier;
    current_tier = tier;
    tier_confirmed = false;

    const struct bt_conn_le_subrate_param *params;
    const char *tier_name;
    switch (tier) {
    case TIER_ACTIVE:  params = &active_params;  tier_name = "ACTIVE";  break;
    case TIER_IDLE:    params = &idle_params;    tier_name = "IDLE";    break;
    case TIER_DORMANT: params = &dormant_params; tier_name = "DORMANT"; break;
    default: return;
    }

    LOG_INF("Subrating: %s (factor=%d-%d, lat=%d, cn=%d)",
            tier_name, params->subrate_min, params->subrate_max,
            params->max_latency, params->continuation_number);

    bt_conn_le_subrate_set_defaults(params);
    bt_conn_foreach(BT_CONN_TYPE_LE, apply_subrate_to_conn, (void *)params);

    if (!tier_confirmed) {
        uint16_t target_factor = params->subrate_max;
        uint16_t retry_ms;
        if (target_factor <= last_confirmed_factor) {
            retry_ms = 20;  /* aggressive → short retry */
        } else {
            retry_ms = last_confirmed_factor * 12 + 20;
        }
        k_work_schedule(&tier_retry_work, K_MSEC(retry_ms));
    }

#if IS_ENABLED(CONFIG_ZMK_BLE_HOST_CONN_PARAM_DORMANT)
    if (tier == TIER_DORMANT) {
#if IS_ENABLED(CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY)
        raise_zmk_ble_host_param_request((struct zmk_ble_host_param_request){
            .interval_min = HOST_DORMANT_INT_MIN, .interval_max = HOST_DORMANT_INT_MAX,
            .latency = HOST_DORMANT_LATENCY, .timeout = HOST_DORMANT_TIMEOUT, .restore = false,
        });
#else
        bt_conn_foreach(BT_CONN_TYPE_LE, apply_conn_param_to_host, (void *)&host_dormant_params);
#endif
    } else if (prev_tier == TIER_DORMANT) {
#if IS_ENABLED(CONFIG_ZMK_BLE_DYNAMIC_HID_LATENCY)
        raise_zmk_ble_host_param_request((struct zmk_ble_host_param_request){ .restore = true });
#else
        bt_conn_foreach(BT_CONN_TYPE_LE, apply_conn_param_to_host, (void *)&host_active_params);
#endif
    }
#endif
}

static void dormant_timer_handler(struct k_work *work) {
    set_tier(TIER_DORMANT);
}

static void tier_retry_handler(struct k_work *work) {
    if (!tier_confirmed) {
        /* Bounded retries — give up after TIER_MAX_RETRIES to avoid
         * infinite loop on permanent controller errors. */
        if (++tier_retry_count > TIER_MAX_RETRIES) {
            LOG_WRN("Subrate tier change failed after %d retries — accepting current state",
                    TIER_MAX_RETRIES);
            tier_confirmed = true;
            tier_retry_count = 0;
            return;
        }
        LOG_INF("Retrying subrate tier (%d/%d)", tier_retry_count, TIER_MAX_RETRIES);
        /* Don't set tier_confirmed = true before set_tier().
         * set_tier() checks (tier == current_tier && tier_confirmed)
         * and would no-op if we set it true first. */
        set_tier(current_tier);
    }
}

static void subrate_active(void) {
    k_work_cancel_delayable(&dormant_work);
    set_tier(TIER_ACTIVE);
}

static void subrate_idle(void) {
    k_work_cancel_delayable(&dormant_work);
    set_tier(TIER_IDLE);
    k_work_schedule(&dormant_work, K_MSEC(SUBRATE_DORMANT_DELAY_MS));
}

/* Apply correct subrating tier when CS_OPERATIONAL is reached.
 * Replays any deferred tier change first, then falls back to
 * activity-based inference for the cold-boot case. */
static void on_operational(void) {
    /* Replay deferred tier from set_tier() that was gated during SCI setup */
    if (has_pending_tier) {
        enum subrate_tier deferred = pending_tier;
        has_pending_tier = false;
        LOG_INF("Replaying deferred subrate tier: %d", deferred);
        set_tier(deferred);
        return;
    }
    /* Fallback: infer from current activity state (cold-boot case) */
    enum zmk_activity_state act = zmk_activity_get_state();
    if (act == ZMK_ACTIVITY_ACTIVE) {
        subrate_active();
    }
    /* IDLE/SLEEP: defaults are already idle_params from init — no action needed */
}

static void subrate_changed_cb(struct bt_conn *conn,
                               const struct bt_conn_le_subrate_changed *params) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) return;

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (params->status == BT_HCI_ERR_SUCCESS) {
        LOG_INF("Subrating [%s %s]: factor=%d, cn=%d",
                info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral",
                addr_str, params->factor, params->continuation_number);
        if (info.role == BT_CONN_ROLE_CENTRAL) {
            last_confirmed_factor = params->factor;
            tier_confirmed = true;
            tier_retry_count = 0;  /* Fix 2: reset on success */
            k_work_cancel_delayable(&tier_retry_work);
        }
    } else {
        LOG_WRN("Subrating failed [%s]: 0x%02x", addr_str, params->status);
        /* Fix 2: give up immediately on permanent errors */
        if (params->status == BT_HCI_ERR_UNSUPP_REMOTE_FEATURE ||
            params->status == BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL) {
            LOG_ERR("Subrating permanently unsupported by peer — disabling retries");
            tier_confirmed = true;
            tier_retry_count = 0;
            k_work_cancel_delayable(&tier_retry_work);
        }
    }
}

static int activity_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) return -ENOTSUP;

    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        subrate_active();
        break;
    case ZMK_ACTIVITY_IDLE:
        subrate_idle();
        break;
    case ZMK_ACTIVITY_SLEEP:
        k_work_cancel_delayable(&dormant_work);
        set_tier(TIER_DORMANT);
        break;
    default:
        break;
    }
    return 0;
}

ZMK_LISTENER(conn_lifecycle_subrating, activity_listener);
ZMK_SUBSCRIPTION(conn_lifecycle_subrating, zmk_activity_state_changed);

#endif /* BT_SUBRATING && ZMK_SPLIT_ROLE_CENTRAL */

/* ──── Combined BT_CONN_CB_DEFINE ──── */

BT_CONN_CB_DEFINE(conn_lifecycle_cb) = {
    .le_phy_updated = le_phy_updated_cb,
    .disconnected = disconnected_cb,
    .security_changed = security_changed_cb,
    .le_param_req = le_param_req_cb,
    .le_param_updated = le_param_updated_cb,
#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    .subrate_changed = subrate_changed_cb,
#endif
};

/* ──── Public API: flush timeout for host connections ──── */

int sci_set_flush_timeout(struct bt_conn *conn) {
    if (!IS_ENABLED(CONFIG_BT_CTLR_LE_FLUSHABLE_ACL_DATA) || MIN_FLUSH_SLOTS == 0)
        return 0;

    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info)) return -EINVAL;

    /* interval_us is in microseconds; convert to 0.625ms slots (divide by 625). */
    uint16_t ci_slots = (uint16_t)(info.le.interval_us / 625);
    uint16_t slots = MAX(MIN_FLUSH_SLOTS, ci_slots * 3);
    return send_flush_timeout(conn, slots);
}

/* ──── Set default rate params at init ──── */

static void set_default_rate_params(void) {
    uint16_t ci_val = (uint16_t)(SCI_TARGET_US / 125);
    struct net_buf *buf = bt_hci_cmd_create(
        SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_RATE_PARAMS,
        sizeof(sdc_hci_cmd_le_set_default_rate_params_t));
    if (!buf) return;

    sdc_hci_cmd_le_set_default_rate_params_t *cmd =
        net_buf_add(buf, sizeof(*cmd));
    cmd->conn_interval_min = ci_val;
    cmd->conn_interval_max = ci_val;
#if IS_ENABLED(CONFIG_BT_SUBRATING)
    cmd->subrate_min = SUBRATE_ACTIVE_MIN;
    cmd->subrate_max = SUBRATE_ACTIVE_MAX;
#else
    cmd->subrate_min = 1;
    cmd->subrate_max = 1;
#endif
    cmd->max_latency = 0;
    cmd->continuation_number = 0;
    cmd->supervision_timeout = SCI_SUPERVISION_TO;
    cmd->min_ce_length = 0;
    cmd->max_ce_length = 0;

    int err = bt_hci_cmd_send(SDC_HCI_OPCODE_CMD_LE_SET_DEFAULT_RATE_PARAMS, buf);
    if (err) LOG_ERR("Set default rate params failed: %d", err);
    else LOG_INF("Default rate params: CI=%u us", SCI_TARGET_US);
}

/* ──── Initialization ──── */

static int conn_lifecycle_init(void) {
    k_work_init_delayable(&split_lc.step_work, split_step_handler);
    set_default_rate_params();

#if IS_ENABLED(CONFIG_BT_SUBRATING) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    bt_conn_le_subrate_set_defaults(&idle_params);
    LOG_INF("Subrating: active=%d-%d, idle=%d-%d, dormant=%d-%d (delay=%ds)",
            SUBRATE_ACTIVE_MIN, SUBRATE_ACTIVE_MAX,
            SUBRATE_IDLE_MIN, SUBRATE_IDLE_MAX,
            SUBRATE_DORMANT_MIN, SUBRATE_DORMANT_MAX,
            SUBRATE_DORMANT_DELAY_MS / 1000);
#endif

    LOG_INF("conn_lifecycle initialized (SCI target=%u us, TP delay=%u ms)",
            SCI_TARGET_US, SCI_PHY_SWITCH_DELAY_MS);
    return 0;
}

SYS_INIT(conn_lifecycle_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
