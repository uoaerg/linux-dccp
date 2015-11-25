/*
 * TFRC library initialisation
 *
 * Copyright (c) 2009 Federal University of Campina Grande,
 * Embedded Systems and Pervasive Computing Lab
 *     Almost copied from tfrc.c, only renamed symbols
 *
 * Copyright (c) 2007 The University of Aberdeen, Scotland, UK
 * Copyright (c) 2007 Arnaldo Carvalho de Melo <acme@redhat.com>
 */
#include "tfrc_sp.h"

#ifdef CONFIG_IP_DCCP_TFRC_SP_DEBUG
bool tfrc_sp_debug;
module_param(tfrc_sp_debug, bool, 0644);
MODULE_PARM_DESC(tfrc_sp_debug, "Enable TFRC-SP debug messages");
#endif

int __init tfrc_sp_lib_init(void)
{
	int rc = tfrc_sp_li_init();

	if (rc)
		goto out;

	rc = tfrc_sp_tx_packet_history_init();
	if (rc)
		goto out_free_loss_intervals;

	rc = tfrc_sp_rx_packet_history_init();
	if (rc)
		goto out_free_tx_history;
	return 0;

out_free_tx_history:
	tfrc_sp_tx_packet_history_exit();
out_free_loss_intervals:
	tfrc_sp_li_exit();
out:
	return rc;
}

void tfrc_sp_lib_exit(void)
{
	tfrc_sp_rx_packet_history_exit();
	tfrc_sp_tx_packet_history_exit();
	tfrc_sp_li_exit();
}
