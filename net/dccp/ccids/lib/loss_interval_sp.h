#ifndef _DCCP_LI_HIST_SP_
#define _DCCP_LI_HIST_SP_
/*
 *  Copyright (c) 2009 Federal University of Campina Grande, Embedded
 *  Copyright (c) 2007   The University of Aberdeen, Scotland, UK
 *  Copyright (c) 2005-7 The University of Waikato, Hamilton, New Zealand.
 *  Copyright (c) 2005-7 Ian McDonald <ian.mcdonald@jandi.co.nz>
 *  Copyright (c) 2005 Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 */
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/slab.h>

#ifndef _DCCP_LI_HIST_
/*
 * Number of loss intervals (RFC 4342, 8.6.1). The history size is one more than
 * NINTERVAL, since the `open' interval I_0 is always stored as the first entry.
 */
#define NINTERVAL	8
#define LIH_SIZE	(NINTERVAL + 1)

/**
 *  tfrc_loss_interval  -  Loss history record for TFRC-based protocols
 *  @li_seqno:		Highest received seqno before the start of loss
 *  @li_ccval:		The CCVal belonging to @li_seqno
 *  @li_is_closed:	Whether @li_seqno is older than 1 RTT
 *  @li_is_short:	Whether this interval is no longer that 2 RTT
 *  @li_length:		Loss interval sequence length
 *  @li_losses:        Number of losses counted on this interval
 */
struct tfrc_loss_interval {
	u64		 li_seqno:48,
			 li_ccval:4,
			 li_is_closed:1,
			 li_is_short:1;
	u32		 li_length;
	u32              li_losses;
};

/**
 *  tfrc_loss_hist  -  Loss record database
 *  @ring:	Circular queue managed in LIFO manner
 *  @counter:	Current count of entries (can be more than %LIH_SIZE)
 *  @i_mean:	Current Average Loss Interval [RFC 3448, 5.4]
 */
struct tfrc_loss_hist {
	struct tfrc_loss_interval	*ring[LIH_SIZE];
	u8				counter;
	u32				i_mean;
};

static inline void tfrc_lh_init(struct tfrc_loss_hist *lh)
{
	memset(lh, 0, sizeof(struct tfrc_loss_hist));
}

static inline u8 tfrc_lh_is_initialised(struct tfrc_loss_hist *lh)
{
	return lh->counter > 0;
}

static inline u8 tfrc_lh_length(struct tfrc_loss_hist *lh)
{
	return min(lh->counter, (u8)LIH_SIZE);
}
#endif	/* _DCCP_LI_HIST_ */

/*
 *  tfrc_loss_data_entry  -  Holds info about one loss interval
 *  @next:             next entry on this linked list
 *  @lossless_length:  length of lossless sequence
 *  @ecn_nonce_sum:    ecn nonce sum for this interval
 *  @loss_length:      length of lossy part
 *  @data_length:      data length on lossless part
 *  @drop_count:       count of dopped packets
 */
struct tfrc_loss_data_entry {
	struct tfrc_loss_data_entry	*next;
	u32				lossless_length:24;
	u8				ecn_nonce_sum:1;
	u32				loss_length:23;
	u32				data_length:24;
	u32				drop_count:24;
};

/* As defined at section 8.6.1. of RFC 4342 */
#define TFRC_LOSS_INTERVALS_OPT_MAX_LENGTH	28
/* Specified on section 8.7. of CCID4 draft */
#define TFRC_DROP_OPT_MAX_LENGTH		84
#define TFRC_LI_OPT_SZ	\
	(2 + TFRC_LOSS_INTERVALS_OPT_MAX_LENGTH*9)
#define TFRC_DROPPED_OPT_SZ \
	(1 + TFRC_DROP_OPT_MAX_LENGTH*3)

/*
 *  tfrc_loss_data  -  loss interval data
 *  used by loss intervals and dropped packets options
 *  @head:                     linked list containing loss interval data
 *  @counter:                  number of entries
 *  @loss_intervals_opts:      space necessary for writing temporary option
 *                             data for loss intervals option
 *  @drop_opts:                        same for dropped packets option
 *  @last_loss_count:          last loss count (num. of packets
 *                             after hole on transmission) observed
 *  @sto_ecn:                  ecn's observed while waiting for hole
 *                             to be filled or accepted as missing
 *  @sto_is_data:              flags about if packets saw were data packets
 */
struct tfrc_loss_data {
	struct tfrc_loss_data_entry	*head;
	u16				counter;
	u8				loss_intervals_opts[TFRC_LI_OPT_SZ];
	u8				drop_opts[TFRC_DROPPED_OPT_SZ];
	u8				last_loss_count;
	u8				sto_ecn;
	u8				sto_is_data;
};

static inline void tfrc_ld_init(struct tfrc_loss_data *ld)
{
	memset(ld, 0, sizeof(*ld));
}

/*
 * tfrc_ecn_echo_sum_entry  -  store sent ecn codepoint info
 * ecn_echo_sum:		ecn echo sum up to that packet
 * seq_num:			sequence number of packet
 * previous:			previous sent packet info
 */
struct tfrc_ecn_echo_sum_entry {
	u8				ecn_echo_sum:1;
	u64				seq_num:48;
	struct tfrc_ecn_echo_sum_entry	*previous;
};

/*
 * tfrc_tx_li_data  -  data about sent ecn and parsed options
 * ecn_sums_head:		ecn data list
 * seq_num:			sequence number of packet
 * previous:			previous sent packet info
 */
struct tfrc_tx_li_data {
	struct tfrc_ecn_echo_sum_entry	*ecn_sums_head;
	u32				dropped_packets_data[1 + 9];
	u32				loss_interval_data[1 + 9];
	u8				skip_length;
};

struct tfrc_rx_hist;

bool tfrc_sp_lh_interval_add(struct tfrc_loss_hist *, struct tfrc_rx_hist *,
			     u32 (*first_li)(struct sock *), struct sock *);
void tfrc_sp_update_li_data(struct tfrc_loss_data *, struct tfrc_rx_hist *,
			    struct sk_buff *, bool new_loss, bool new_event);
void tfrc_sp_lh_update_i_mean(struct tfrc_loss_hist *lh, struct sk_buff *);
void tfrc_sp_lh_cleanup(struct tfrc_loss_hist *lh);
void tfrc_sp_ld_cleanup(struct tfrc_loss_data *ld);
void tfrc_sp_ld_prepare_data(u8 loss_count, struct tfrc_loss_data *ld);
int  tfrc_sp_get_random_ect(struct tfrc_tx_li_data *li_data, u64 seqn);
bool tfrc_sp_check_ecn_sum(struct tfrc_tx_li_data *li_data, u8 *optval,
			   u8 optlen, struct sk_buff *skb);
struct tfrc_tx_hist_entry;
u32 tfrc_sp_p_from_loss_intervals_opt(struct tfrc_tx_li_data *li_data,
				      struct tfrc_tx_hist_entry *head,
				      u8 curr_ccval, u64 seqno);
void tfrc_sp_parse_dropped_packets_opt(struct tfrc_tx_li_data *li_data,
				       u8 *optval, u8 optlen);
void tfrc_sp_parse_loss_intervals_opt(struct tfrc_tx_li_data *li_data,
				      u8 *optval, u8 optlen);
void tfrc_sp_tx_ld_cleanup(struct tfrc_ecn_echo_sum_entry **);

#endif	/* _DCCP_LI_HIST_SP_ */
