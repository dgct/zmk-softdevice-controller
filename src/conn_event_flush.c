/*
 * Copyright (c) 2026 dgct
 *
 * SPDX-License-Identifier: MIT
 *
 * Connection-event-aligned flush callback for ZMK + SoftDevice Controller.
 *
 * Uses SDC's anchor point update reports (VS subevent 0x82) to signal
 * each BLE connection event boundary. A registered callback can use this
 * to flush accumulated input data in sync with the radio schedule,
 * eliminating the beat-pattern jitter between input sample rate and CI.
 *
 * Only one VS event callback can be registered system-wide via
 * bt_hci_register_vnd_evt_cb (single pointer). This module is intended
 * for the peripheral (right half) where qos.c does not run. If both
 * need to coexist on the same half, a dispatcher wrapper is needed.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/net_buf.h>

#include <sdc_hci_vs.h>

#include <zmk/sdc/conn_event_flush.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_ce_flush, CONFIG_ZMK_CE_FLUSH_LOG_LEVEL);

static void (*flush_cb)(void);
static bool anchor_reports_enabled;

/*
 * VS HCI event callback — called from the BT RX thread whenever the
 * SoftDevice Controller generates a vendor-specific event.
 *
 * We handle SDC_HCI_SUBEVENT_VS_CONN_ANCHOR_POINT_UPDATE_REPORT:
 * it fires after each connection event where the peripheral received
 * a packet from the central. We call the registered flush callback
 * to drain accumulated input data for the next connection event.
 */
static bool on_vs_evt(struct net_buf_simple *buf)
{
	uint8_t *subevent_code;

	subevent_code = net_buf_simple_pull_mem(buf, sizeof(*subevent_code));

	switch (*subevent_code) {
	case SDC_HCI_SUBEVENT_VS_CONN_ANCHOR_POINT_UPDATE_REPORT:
		if (flush_cb) {
			flush_cb();
		}
		return true;
	default:
		return false;
	}
}

/*
 * Enable anchor point update event reporting via VS HCI command.
 *
 * Uses bt_hci_cmd_send (async, no response wait) because this is called
 * from bt_conn_cb.connected which runs on the BT RX thread —
 * bt_hci_cmd_send_sync would deadlock waiting for the response to be
 * processed on the same thread.
 */
static int enable_anchor_point_reporting(void)
{
	struct net_buf *buf;

	buf = bt_hci_cmd_create(
		SDC_HCI_OPCODE_CMD_VS_CONN_ANCHOR_POINT_UPDATE_EVENT_REPORT_ENABLE,
		sizeof(sdc_hci_cmd_vs_conn_anchor_point_update_event_report_enable_t));
	if (!buf) {
		LOG_ERR("Failed to allocate HCI command buffer");
		return -ENOMEM;
	}

	sdc_hci_cmd_vs_conn_anchor_point_update_event_report_enable_t *cmd =
		net_buf_add(buf, sizeof(*cmd));
	cmd->enable = 1;

	/* Async send — fire and forget, safe from BT RX thread */
	return bt_hci_cmd_send(
		SDC_HCI_OPCODE_CMD_VS_CONN_ANCHOR_POINT_UPDATE_EVENT_REPORT_ENABLE,
		buf);
}

/*
 * Connection callback — enable anchor point reporting on first connection.
 * The VS enable command persists across connections; only need to send once.
 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err || anchor_reports_enabled) {
		return;
	}

	int ret = enable_anchor_point_reporting();
	if (!ret) {
		anchor_reports_enabled = true;
		LOG_INF("Anchor point reporting enabled");
	} else {
		LOG_ERR("Failed to enable anchor point reporting: %d", ret);
	}
}

BT_CONN_CB_DEFINE(ce_flush_conn_cb) = {
	.connected = connected_cb,
};

int zmk_sdc_conn_event_flush_register(void (*cb)(void))
{
	int err;

	flush_cb = cb;

	err = bt_hci_register_vnd_evt_cb(on_vs_evt);
	if (err) {
		LOG_ERR("Failed to register VS event callback: %d", err);
		return err;
	}

	LOG_INF("Connection event flush callback registered");
	return 0;
}
