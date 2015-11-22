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
static struct kmem_cache  *tfrc_ld_slab  __read_mostly;
static struct kmem_cache  *tfrc_ecn_echo_sum_slab  __read_mostly;
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

/*
 * tfrc_sp_get_random_ect  -  return random ect codepoint
 * li_data:		data where to register ect sent
 * seqn:		packet's sequence number
 */
int tfrc_sp_get_random_ect(struct tfrc_tx_li_data *li_data, u64 seqn)
{
	int ect;
	struct tfrc_ecn_echo_sum_entry *sum;

	/* TODO: implement random ect*/
	ect = INET_ECN_ECT_0;

	sum = kmem_cache_alloc(tfrc_ecn_echo_sum_slab, GFP_ATOMIC);

	sum->previous = li_data->ecn_sums_head;
	sum->ecn_echo_sum = (sum->previous->ecn_echo_sum) ? !ect : ect;
	sum->seq_num = seqn;

	li_data->ecn_sums_head = sum;

	return ect;
}

/*
 * tfrc_sp_tx_ld_cleanup  -  free all entries
 * echo_sums_data:		head of the list
 */
void tfrc_sp_tx_ld_cleanup(struct tfrc_ecn_echo_sum_entry **echo_sums_data)
{
	struct tfrc_ecn_echo_sum_entry *e, *previous;
	e = *echo_sums_data;

	while (e != NULL) {
		previous = e->previous;
		kmem_cache_free(tfrc_ecn_echo_sum_slab, e);
		e = previous;
	}

	*echo_sums_data = NULL;
}

/*
 * Allocation routine for new entries of loss interval data
 */
static struct tfrc_loss_data_entry *tfrc_ld_add_new(struct tfrc_loss_data *ld)
{
       struct tfrc_loss_data_entry *new =
			kmem_cache_alloc(tfrc_ld_slab, GFP_ATOMIC);

	if (new == NULL)
		return NULL;

	memset(new, 0, sizeof(struct tfrc_loss_data_entry));

	new->next = ld->head;
	ld->head = new;
	ld->counter++;

	return new;
}

void tfrc_sp_ld_cleanup(struct tfrc_loss_data *ld)
{
	struct tfrc_loss_data_entry *next, *h = ld->head;

	while (h) {
		next = h->next;
		kmem_cache_free(tfrc_ld_slab, h);
		h = next;
	}

	ld->head = NULL;
	ld->counter = 0;
}

/*
 *  tfrc_sp_ld_prepare_data  -  updates arrays on tfrc_loss_data
 *                             so they can be sent as options
 *  @loss_count:       current loss count (packets after hole on transmission),
 *                     used to determine skip length for loss intervals option
 *  @ld:               loss intervals data being updated
 */
void tfrc_sp_ld_prepare_data(u8 loss_count, struct tfrc_loss_data *ld)
{
	u8 *li_ofs, *d_ofs;
	struct tfrc_loss_data_entry *e;
	u16 count;

	li_ofs = &ld->loss_intervals_opts[0];
	d_ofs = &ld->drop_opts[0];

	count = 0;
	e = ld->head;

	*li_ofs = loss_count + 1;
	li_ofs++;

	while (e != NULL) {

		if (count < TFRC_LOSS_INTERVALS_OPT_MAX_LENGTH) {
			*li_ofs = ((htonl(e->lossless_length) & 0xFFFFFF)<<8);
			li_ofs += 3;
			*li_ofs = ((e->ecn_nonce_sum&0x1) << 31) |
				(htonl((e->loss_length & 0x7FFFFF))<<8);
			li_ofs += 3;
			*li_ofs = ((htonl(e->data_length) & 0xFFFFFF)<<8);
			li_ofs += 3;
		} else
			break;

		if (count < TFRC_DROP_OPT_MAX_LENGTH) {
			*d_ofs = (htonl(e->drop_count) & 0xFFFFFF)<<8;
			d_ofs += 3;
		} else
			break;

		count++;
		e = e->next;
       }
}

/*
 *  tfrc_sp_update_li_data  -  Update tfrc_loss_data upon
 *			       packet receiving or loss detection
 *  @ld:			tfrc_loss_data being updated
 *  @rh:			loss event record
 *  @skb:			received packet
 *  @new_loss:			dictates if new loss was detected
 *				upon receiving current packet
 *  @new_event:			...and if the loss starts new loss interval
 */
void tfrc_sp_update_li_data(struct tfrc_loss_data *ld,
			    struct tfrc_rx_hist *rh,
			    struct sk_buff *skb,
			    bool new_loss, bool new_event)
{
	struct tfrc_loss_data_entry *new, *h;

	if (!dccp_data_packet(skb))
		return;

	if (ld->head == NULL) {
		new = tfrc_ld_add_new(ld);
		if (unlikely(new == NULL)) {
			DCCP_CRIT("Cannot allocate new loss data registry.");
			return;
		}

		if (new_loss) {
			new->drop_count = rh->num_losses;
			new->lossless_length = 1;
			new->loss_length = rh->num_losses;

			new->data_length = 1;

			if (dccp_skb_is_ecn_ect1(skb))
				new->ecn_nonce_sum = 1;
			else
				new->ecn_nonce_sum = 0;
		} else {
			new->drop_count = 0;
			new->lossless_length = 1;
			new->loss_length = 0;

			new->data_length = 1;

			if (dccp_skb_is_ecn_ect1(skb))
				new->ecn_nonce_sum = 1;
			else
				new->ecn_nonce_sum = 0;
		}

		return;
	}

	if (new_event) {
		new = tfrc_ld_add_new(ld);
		if (unlikely(new == NULL)) {
			DCCP_CRIT("Cannot allocate new loss data registry. \
					Cleaning up.");
			tfrc_sp_ld_cleanup(ld);
			return;
		}

		new->drop_count = rh->num_losses;
		new->lossless_length = (ld->last_loss_count - rh->loss_count);
		new->loss_length = rh->num_losses;

		new->ecn_nonce_sum = 0;
		new->data_length = 0;

		while (ld->last_loss_count > rh->loss_count) {
			ld->last_loss_count--;

			if (ld->sto_is_data & (1 << (ld->last_loss_count))) {
				new->data_length++;

				if (ld->sto_ecn & (1 << (ld->last_loss_count)))
					new->ecn_nonce_sum =
						!new->ecn_nonce_sum;
			}
		}

		return;
	}

	h = ld->head;

	if (rh->loss_count > ld->last_loss_count) {
		ld->last_loss_count = rh->loss_count;

		ld->sto_is_data |= (1 << (ld->last_loss_count - 1));

		if (dccp_skb_is_ecn_ect1(skb))
			ld->sto_ecn |= (1 << (ld->last_loss_count - 1));

		return;
	}

	if (new_loss) {
		h->drop_count += rh->num_losses;
		h->lossless_length = (ld->last_loss_count - rh->loss_count);
		h->loss_length += h->lossless_length + rh->num_losses;

		h->ecn_nonce_sum = 0;
		h->data_length = 0;

		while (ld->last_loss_count > rh->loss_count) {
			ld->last_loss_count--;

			if (ld->sto_is_data&(1 << (ld->last_loss_count))) {
				h->data_length++;

				if (ld->sto_ecn & (1 << (ld->last_loss_count)))
					h->ecn_nonce_sum = !h->ecn_nonce_sum;
			}
		}

		return;
	}

	if (ld->last_loss_count > rh->loss_count) {
		while (ld->last_loss_count > rh->loss_count) {
			ld->last_loss_count--;

			h->lossless_length++;

			if (ld->sto_is_data & (1 << (ld->last_loss_count))) {
				h->data_length++;

				if (ld->sto_ecn & (1 << (ld->last_loss_count)))
					h->ecn_nonce_sum = !h->ecn_nonce_sum;
			}
		}

		return;
	}

	h->lossless_length++;
	h->data_length++;

	if (dccp_skb_is_ecn_ect1(skb))
		h->ecn_nonce_sum = !h->ecn_nonce_sum;
}

/*
 * tfrc_sp_check_ecn_sum  -  check received ecn sum parsed from
 *			     loss interval option
 * li_data:		data parsed from options
 * optval:		data from option
 * optlen:		option data length
 * skb:		last received packet
 */
bool tfrc_sp_check_ecn_sum(struct tfrc_tx_li_data *li_data, u8 *optval,
			   u8 optlen, struct sk_buff *skb)
{
	u8 skip_length;
	u32 data_length;
	u8 sum0, sum1, li_sum;
	struct tfrc_ecn_echo_sum_entry *entry;
	u64 seqn;

	if ((optlen < 10) || ((optlen - 1)%3 != 0))
		return false;

	if (li_data == NULL)
		return true;

	entry = li_data->ecn_sums_head;

	if (entry == NULL)
		return true;

	seqn = dccp_hdr_ack_seq(skb);

	while (entry->seq_num != seqn) {
		entry = entry->previous;

		if (entry == NULL)
			return true;
	}

	skip_length = *optval;
	optval++;
	optlen--;

	while (skip_length--) {
		entry = entry->previous;

		if (entry == NULL)
			return false;
	}

	optval += 3;
	optlen -= 3;

	do {
		sum0 = entry->ecn_echo_sum;

		li_sum = (*((u32 *)optval)&0x80000000) ? 1 : 0;

		optval += 3;
		optlen -= 3;

		data_length = ntohl((*((u32 *)optval) & 0xFFFFFF00) >> 8);

		while (--data_length) {
			entry = entry->previous;

			if (entry == NULL)
				return true;
		}

		sum1 = entry->ecn_echo_sum;

		if (((sum0+sum1) & 0x1) != li_sum)
			return false;

		optval += 6;
		optlen -= 6;

	} while (optlen >= 9);

	if (entry != NULL)
		tfrc_sp_tx_ld_cleanup(&entry->previous);

	return true;
}

static struct tfrc_tx_hist_entry*
	tfrc_sp_seek_tx_entry(struct tfrc_tx_hist_entry *head,
			      u64 seqno, u32 backward)
{
	if (head == NULL)
		return NULL;

	while (head->seqno != seqno) {
		head = head->next;

		if (head == NULL)
			return NULL;
	}

	while (backward-- > 0) {
		head = head->next;

		if (head == NULL)
			return NULL;
	}

	return head;
}

/*
 * tfrc_sp_p_from_loss_intervals_opt  -  calcs p from loss interval option
 * li_data:		data parsed from options
 * head:		contains ccval info
 * curr_ccval:		current ccval
 * seqno:		last acked seqno
 */
u32 tfrc_sp_p_from_loss_intervals_opt(struct tfrc_tx_li_data *li_data,
				      struct tfrc_tx_hist_entry *head,
				      u8 curr_ccval, u64 seqno)
{
	int i, k;
	u8 ccval;
	u32 i_i, i_tot0, i_tot1, w_tot, i_totl, losses, mean;
	i_tot0 = i_tot1 = w_tot = i_totl = 0;

	if (li_data->loss_interval_data[0] == 0)
		return 0;

	if (li_data->loss_interval_data[0] < li_data->dropped_packets_data[0])
		return 0;

	k = li_data->loss_interval_data[0];

	for (i = 1; i <= k; i++) {
		i_i = li_data->loss_interval_data[i];
		i_totl += i_i;
		ccval = tfrc_sp_seek_tx_entry(head, seqno, i_totl - 1)->ccval;

		if (SUB16(curr_ccval, ccval) <= 8) {
			losses = li_data->dropped_packets_data[i];

			if (losses > 0)
				i_i = div64_u64(i_i, losses);
		}

		if (i != k) {
			i_tot0 += i_i * tfrc_lh_weights[i-1];
			w_tot  += tfrc_lh_weights[i-1];
		}

		if (i > 1)
			i_tot1 += i_i * tfrc_lh_weights[i-2];
	}

	ccval = tfrc_sp_seek_tx_entry(head, seqno,
			li_data->loss_interval_data[1] - 1)->ccval;

	if (SUB16(curr_ccval, ccval) > 8)
		mean = max(i_tot0, i_tot1) / w_tot;
	else
		mean = i_tot1 / w_tot;

	return tfrc_sp_invert_loss_event_rate(mean);

	return 0;
}

/*
 * tfrc_sp_parse_dropped_packets_opt  -  parses received dropped packets option
 * li_data:		used to store parsed data
 * optval:		option data
 * optlen:		option length
 */
void tfrc_sp_parse_dropped_packets_opt(struct tfrc_tx_li_data *li_data,
				       u8 *optval, u8 optlen)
{
	u8 pos;
	u32 dropped;

	if ((optlen%3) != 0) {
		li_data->dropped_packets_data[0] = 0;
		return;
	}

	pos = 0;

	while (pos < optlen) {
		dropped = ntohl(((*((u32 *)(optval + pos))) & 0xFFFFFF00) >> 8);
		li_data->dropped_packets_data[1 + (pos/3)] = dropped;

		if ((pos/3) == 9) {
			li_data->dropped_packets_data[0] = 9;
			return;
		}

		pos += 3;
	}

	li_data->dropped_packets_data[0] = optlen/3;
}

/*
 * tfrc_sp_parse_loss_intervals_opt  -  parses loss interval option
 * li_data:		used to store parsed data
 * optval:		option data
 * optlen:		option length
 */
void tfrc_sp_parse_loss_intervals_opt(struct tfrc_tx_li_data *li_data,
				      u8 *optval, u8 optlen)
{
	u8 pos;
	u32 length;

	if ((optlen%9) != 1) {
		li_data->loss_interval_data[0] = 0;
		return;
	}

	pos = 1;
	optval++;

	while (pos < optlen) {
		length = ntohl(((*((u32 *)optval)) & 0xFFFFFF00) >> 8);
		pos += 3;
		optval += 3;

		length += ntohl(((*((u32 *)optval))&0x7FFFFF00) >> 8);
		pos += 6;
		optval += 6;

		li_data->loss_interval_data[(pos-1)%9] = length;

		if ((pos/9) == 9) {
			li_data->loss_interval_data[0] = 9;
			return;
		}
	}

	li_data->loss_interval_data[0] = (optlen-1)/9;
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

	/*
	 * FIXME: should probably also count non-data packets
	 * (RFC 4342, 6.1)
	 */
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

	if (++lh->counter == 1) {
		lh->i_mean = cur->li_length = (*calc_first_li)(sk);
		cur->li_is_closed = true;
		cur = tfrc_lh_demand_next(lh);
		if (unlikely(cur == NULL)) {
			DCCP_CRIT("Cannot allocate/add loss record.");
			return false;
		}
		++lh->counter;
		cur->li_seqno	  = cong_evt_seqno;
		cur->li_ccval	  = cong_evt->tfrchrx_ccval;
		cur->li_is_closed = false;
		cur->li_length    = dccp_delta_seqno(cur->li_seqno,
				 tfrc_rx_hist_last_rcv(rh)->tfrchrx_seqno) + 1;
	} else {
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
	tfrc_ld_slab = kmem_cache_create("tfrc_sp_li_data",
					 sizeof(struct tfrc_loss_data_entry), 0,
					 SLAB_HWCACHE_ALIGN, NULL);
	tfrc_ecn_echo_sum_slab = kmem_cache_create("tfrc_sp_ecn_echo_sum",
				sizeof(struct tfrc_ecn_echo_sum_entry), 0,
						SLAB_HWCACHE_ALIGN, NULL);

	if ((tfrc_lh_slab != NULL) &&
	    (tfrc_ld_slab != NULL) &&
	    (tfrc_ecn_echo_sum_slab != NULL))
		return 0;

	if (tfrc_lh_slab != NULL) {
		kmem_cache_destroy(tfrc_lh_slab);
		tfrc_lh_slab = NULL;
	}

	if (tfrc_ld_slab != NULL) {
		kmem_cache_destroy(tfrc_ld_slab);
		tfrc_ld_slab = NULL;
	}

	if (tfrc_ecn_echo_sum_slab != NULL) {
		kmem_cache_destroy(tfrc_ecn_echo_sum_slab);
		tfrc_ecn_echo_sum_slab = NULL;
	}

	return -ENOBUFS;
}

void tfrc_sp_li_exit(void)
{
	if (tfrc_lh_slab != NULL) {
		kmem_cache_destroy(tfrc_lh_slab);
		tfrc_lh_slab = NULL;
	}

	if (tfrc_ld_slab != NULL) {
		kmem_cache_destroy(tfrc_ld_slab);
		tfrc_ld_slab = NULL;
	}

	if (tfrc_ecn_echo_sum_slab != NULL) {
		kmem_cache_destroy(tfrc_ecn_echo_sum_slab);
		tfrc_ecn_echo_sum_slab = NULL;
	}
}
