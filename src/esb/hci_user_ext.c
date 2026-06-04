/*
 * HCI User Extension — stub implementation for ESB ticker node
 *
 * ESB does not generate proprietary HCI events or control PDUs.
 * These functions satisfy the BT_CTLR_USER_EXT linkage requirement.
 *
 * The declarations are in zephyr's hci_user_ext.h
 * (subsys/bluetooth/controller/hci/hci_user_ext.h).
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>

/* Forward declarations — keep this stub minimal */
struct node_rx_pdu;
struct pdu_data;
struct net_buf;

int8_t hci_user_ext_get_class(struct node_rx_pdu *node_rx)
{
	/* No proprietary HCI event class */
	return -1;
}

void hci_user_ext_encode_control(struct node_rx_pdu *node_rx,
				 struct pdu_data *pdu_data,
				 struct net_buf *buf)
{
	/* Nothing to encode — ESB doesn't generate HCI control events */
}
