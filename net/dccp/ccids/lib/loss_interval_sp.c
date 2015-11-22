/*
 *  Copyright (c) 2009 Federal University of Campina Grande, Embedded
 *  Copyright (c) 2007   The University of Aberdeen, Scotland, UK
 *  Copyright (c) 2005-7 The University of Waikato, Hamilton, New Zealand.
 *  Copyright (c) 2005-7 Ian McDonald <ian.mcdonald@jandi.co.nz>
 *  Copyright (c) 2005 Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */
#include <net/sock.h>
#include "tfrc_sp.h"

static struct kmem_cache  *tfrc_lh_slab  __read_mostly;
/* Loss Interval weights from [RFC 3448, 5.4], scaled by 10 */
static const int tfrc_lh_weights[NINTERVAL] = { 10, 10, 10, 10, 8, 6, 4, 2 };

/* implements LIFO semantics on the array */
static inline u8 LIH_INDEX(const u8 ctr)
{
	return LIH_SIZE - 1 - (ctr % LIH_SIZE);
}

/* the `counter' index always points at the next entry to be populated */
static inline struct tfrc_loss_interval *tfrc_lh_peek(struct tfrc_loss_hist *lh)
{
	return lh->counter ? lh->ring[LIH_INDEX(lh->counter - 1)] : NULL;
}

/* given i with 0 <= i <= k, return I_i as per the rfc3448bis notation */
static inline u32 tfrc_lh_get_interval(struct tfrc_loss_hist *lh, const u8 i)
{
	BUG_ON(i >= lh->counter);
	return lh->ring[LIH_INDEX(lh->counter - i - 1)]->li_length;
}

static inline u32
tfrc_lh_loss_interval_losses(struct tfrc_loss_hist *lh, const u8 i)
{
	BUG_ON(i >= lh->counter);
	return lh->ring[LIH_INDEX(lh->counter - i - 1)]->li_losses;
}

static inline u8
tfrc_lh_interval_is_short(struct tfrc_loss_hist *lh, const u8 i)
{
	BUG_ON(i >= lh->counter);
	return lh->ring[LIH_INDEX(lh->counter - i - 1)]->li_is_short;
}

static inline u8
tfrc_lh_loss_interval_ccval(struct tfrc_loss_hist *lh, const u8 i)
{
	BUG_ON(i >= lh->counter);
	return lh->ring[LIH_INDEX(lh->counter - i - 1)]->li_ccval;
}

/*
 *	On-demand allocation and de-allocation of entries
 */
static struct tfrc_loss_interval *tfrc_lh_demand_next(struct tfrc_loss_hist *lh)
{
	if (lh->ring[LIH_INDEX(lh->counter)] == NULL)
		lh->ring[LIH_INDEX(lh->counter)] =
			kmem_cache_alloc(tfrc_lh_slab, GFP_ATOMIC);

	return lh->ring[LIH_INDEX(lh->counter)];
}

void tfrc_sp_lh_cleanup(struct tfrc_loss_hist *lh)
{
	if (!tfrc_lh_is_initialised(lh))
		return;

	for (lh->counter = 0; lh->counter < LIH_SIZE; lh->counter++)
		if (lh->ring[LIH_INDEX(lh->counter)] != NULL) {
			kmem_cache_free(tfrc_lh_slab,
					lh->ring[LIH_INDEX(lh->counter)]);
			lh->ring[LIH_INDEX(lh->counter)] = NULL;
		}
}

static void tfrc_sp_lh_calc_i_mean(struct tfrc_loss_hist *lh, __u8 curr_ccval)
{
	u32 i_i, i_tot0 = 0, i_tot1 = 0, w_tot = 0;
	int i, k = tfrc_lh_length(lh) - 1; /* k is as in rfc3448bis, 5.4 */
	u32 losses;

	if (k <= 0)
		return;

	for (i = 0; i <= k; i++) {
		i_i = tfrc_lh_get_interval(lh, i);

		if (tfrc_lh_interval_is_short(lh, i)) {

			losses = tfrc_lh_loss_interval_losses(lh, i);

			if (losses > 0)
				i_i = DIV_ROUND_UP(i_i, losses);
		}

		if (i < k) {
			i_tot0 += i_i * tfrc_lh_weights[i];
			w_tot  += tfrc_lh_weights[i];
		}
		if (i > 0)
			i_tot1 += i_i * tfrc_lh_weights[i-1];
	}

	lh->i_mean = max(i_tot0, i_tot1) / w_tot;
	BUG_ON(w_tot == 0);
	if (SUB16(curr_ccval, tfrc_lh_loss_interval_ccval(lh, 0) > 8))
		lh->i_mean = max(i_tot0, i_tot1) / w_tot;
	else
		lh->i_mean = i_tot1 / w_tot;
}

/**
 * tfrc_lh_update_i_mean  -  Update the `open' loss interval I_0
 * This updates I_mean as the sequence numbers increase. As a consequence, the
 * open loss interval I_0 increases, hence p = W_tot/max(I_tot0, I_tot1)
 * decreases, and thus there is no need to send renewed feedback.
 */
void tfrc_sp_lh_update_i_mean(struct tfrc_loss_hist *lh, struct sk_buff *skb)
{
	struct tfrc_loss_interval *cur = tfrc_lh_peek(lh);
	s64 len;

	if (cur == NULL)			/* not initialised */
		return;

	/* FIXME: should probably also count non-data packets (RFC 4342, 6.1) */
	if (!dccp_data_packet(skb))
		return;

	len = dccp_delta_seqno(cur->li_seqno, DCCP_SKB_CB(skb)->dccpd_seq) + 1;

	if (len - (s64)cur->li_length <= 0)	/* duplicate or reordered */
		return;

	if (SUB16(dccp_hdr(skb)->dccph_ccval, cur->li_ccval) > 4)
		/*
		 * Implements RFC 4342, 10.2:
		 * If a packet S (skb) exists whose seqno comes `after' the one
		 * starting the current loss interval (cur) and if the modulo-16
		 * distance from C(cur) to C(S) is greater than 4, consider all
		 * subsequent packets as belonging to a new loss interval. This
		 * test is necessary since CCVal may wrap between intervals.
		 */
		cur->li_is_closed = 1;

	if (tfrc_lh_length(lh) == 1)		/* due to RFC 3448, 6.3.1 */
		return;

	cur->li_length = len;
	tfrc_sp_lh_calc_i_mean(lh, dccp_hdr(skb)->dccph_ccval);
}

/* RFC 4342, 10.2: test for the existence of packet with sequence number S */
static bool tfrc_lh_closed_check(struct tfrc_loss_interval *cur, const u8 ccval)
{
	if (SUB16(ccval, cur->li_ccval) > 4)
		cur->li_is_closed = true;
	return cur->li_is_closed;
}

/**
 * tfrc_lh_interval_add  -  Insert new record into the Loss Interval database
 * @lh:		   Loss Interval database
 * @rh:		   Receive history containing a fresh loss event
 * @calc_first_li: Caller-dependent routine to compute length of first interval
 * @sk:		   Used by @calc_first_li in caller-specific way (subtyping)
 * Updates I_mean and returns 1 if a new interval has in fact been added to @lh.
 */
bool tfrc_sp_lh_interval_add(struct tfrc_loss_hist *lh,
			     struct tfrc_rx_hist *rh,
			     u32 (*calc_first_li)(struct sock *),
			     struct sock *sk)
{
	struct tfrc_loss_interval *cur = tfrc_lh_peek(lh);
	struct tfrc_rx_hist_entry *cong_evt;
	u64 cong_evt_seqno;

	/*
	 * Determine if the new event is caused by a lost or ECN-marked packet.
	 * Both events can coincide (e.g. if the third packet after a loss is
	 * marked as CE). We avoid the complexity caused by such mixed cases:
	 *   1) if the cause is a lost packet, we do not check whether it is
	 *      also an ECN-marked packet (not necessary);
	 *   2) calling this routine with a loss_count of 0..NDUPACK-1 implies
	 *      that the cause is an ECN-marked-CE packet.
	 *      FIXME: if in this case the loss_count is not 0, loss tracking is
	 *      reset. This is a complex corner case (see packet_history.c) and
	 *      hence currently not supported.
	 */
	if (rh->loss_count == TFRC_NDUPACK) {
		/*
		 * The sequence number of the first packet known to be lost is
		 * the successor of the last packet received before the gap.
		 */
		cong_evt = tfrc_rx_hist_loss_prev(rh);
		cong_evt_seqno = ADD48(cong_evt->tfrchrx_seqno, 1);
	} else {
		/*
		 * ECN-marked packet. Since ECN-marks are reported as soon as a
		 * packet is delivered, it is stored in the last-received entry.
		 */
		cong_evt = tfrc_rx_hist_last_rcv(rh);
		cong_evt_seqno = cong_evt->tfrchrx_seqno;
	}

	/* Test if this event starts a new loss interval */
	if (cur != NULL) {
		s64 len = dccp_delta_seqno(cur->li_seqno, cong_evt_seqno);
		if (len <= 0)
			return false;

		if (!tfrc_lh_closed_check(cur, cong_evt->tfrchrx_ccval)) {
			cur->li_losses += rh->num_losses;
			rh->num_losses = 0;
			return false;
		}

		/* RFC 5348, 5.3: length between subsequent intervals */
		cur->li_length = len;

		if (SUB16(cong_evt->tfrchrx_ccval, cur->li_ccval) <= 8)
			cur->li_is_short = 1;
	}

	/* Make the new interval the current one */
	cur = tfrc_lh_demand_next(lh);
	if (unlikely(cur == NULL)) {
		DCCP_CRIT("Cannot allocate/add loss record.");
		return false;
	}

	cur->li_seqno	  = cong_evt_seqno;
	cur->li_ccval	  = cong_evt->tfrchrx_ccval;
	cur->li_is_closed = false;
	cur->li_is_short  = 0;

	cur->li_losses = rh->num_losses;
	rh->num_losses = 0;

	if (++lh->counter == 1)
		lh->i_mean = cur->li_length = (*calc_first_li)(sk);
	else {
		/* RFC 5348, 5.3: length of the open loss interval I_0 */
		cur->li_length = dccp_delta_seqno(cur->li_seqno,
				 tfrc_rx_hist_last_rcv(rh)->tfrchrx_seqno) + 1;

		if (lh->counter > (2*LIH_SIZE))
			lh->counter -= LIH_SIZE;

		tfrc_sp_lh_calc_i_mean(lh, cong_evt->tfrchrx_ccval);
	}
	return true;
}

int __init tfrc_sp_li_init(void)
{
	tfrc_lh_slab = kmem_cache_create("tfrc_sp_li_hist",
					 sizeof(struct tfrc_loss_interval), 0,
					 SLAB_HWCACHE_ALIGN, NULL);
	return tfrc_lh_slab == NULL ? -ENOBUFS : 0;
}

void tfrc_sp_li_exit(void)
{
	if (tfrc_lh_slab != NULL) {
		kmem_cache_destroy(tfrc_lh_slab);
		tfrc_lh_slab = NULL;
	}
}
