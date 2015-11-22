/*
 *  Copyright (c) 2007   Federal University of Campina Grande, Paraiba, BR.
 *  Copyright (c) 2007   University of Helsinki, Finland.
 *  Copyright (c) 2007   The University of Aberdeen, Scotland, UK
 *  Copyright (c) 2005-7 The University of Waikato, Hamilton, New Zealand.
 *
 *  An implementation of the DCCP protocol
 *
 *  Copyright (c) 2009 Ivo Calado, Erivaldo Xavier, Leandro Sales
 *
 *  This code has been developed by the Federal University of Campina Grande
 *  Embedded Systems and Pervasive Computing Lab. For further information
 *  please see http://embedded.ufcg.edu.br/
 *  <ivocalado@embedded.ufcg.edu.br> <desadoc@gmail.com> <leandroal@gmail.com>
 *
 *  Copyright (c) 2007 Leandro Sales, Tommi Saviranta
 *
 *  This code has been developed by the Federal University of Campina Grande
 *  Embedded Systems and Pervasive Computing Lab and the Department of Computer
 *  Science at the University of Helsinki. For further information please see
 *  http://embedded.ufcg.edu.br/ <leandroal@gmail.com>
 *  http://www.iki.fi/ <wnd@iki.fi>
 *
 *  Copyright (c) 2005-7 Ian McDonald
 *
 *  This code is based on code developed by the University of Waikato WAND
 *  research group. For further information please see http://www.wand.net.nz/
 *  or e-mail Ian McDonald - ian.mcdonald@jandi.co.nz
 *
 *  This code also uses code from Lulea University, rereleased as GPL by its
 *  authors:
 *  Copyright (c) 2003 Nils-Erik Mattsson, Joacim Haggmark, Magnus Erixzon
 *
 *  Changes to meet Linux coding standards, to make it meet latest ccid4 draft
 *  and to make it work as a loadable module in the DCCP stack written by
 *  Arnaldo Carvalho de Melo <acme@conectiva.com.br>.
 *
 *  Copyright (c) 2005 Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include "../dccp.h"
#include "ccid4.h"

#include <asm/unaligned.h>

#ifdef CONFIG_IP_DCCP_CCID4_DEBUG
static bool ccid4_debug;
#define ccid4_pr_debug(format, a...)	DCCP_PR_DEBUG(ccid4_debug, format, ##a)
#else
#define ccid4_pr_debug(format, a...)
#endif

/*
 *	Transmitter Half-Connection Routines
 */
/* Oscillation Prevention/Reduction: recommended by rfc3448bis, on by default */
static bool ccid4_osc_prev = true;

/**
 * ccid4_update_send_interval  -  Calculate new t_ipi = s / X
 * This respects the granularity of X (64 * bytes/second) and enforces the
 * scaled minimum of s * 64 / t_mbi = `s' bytes/second as per RFC 3448/4342.
 */
static void ccid4_update_send_interval(struct tfrc_hc_tx_sock *hc)
{
	if (unlikely(hc->tx_x <= hc->tx_s))
		hc->tx_x = hc->tx_s;
	hc->tx_t_ipi = scaled_div32(((u64)hc->tx_s) << 6, hc->tx_x);
	DCCP_BUG_ON(hc->tx_t_ipi == 0);

	/* TFRC-SP enforces a minimum interval of 10 milliseconds.  */
	if (hc->tx_t_ipi < MIN_SEND_RATE)
		hc->tx_t_ipi = MIN_SEND_RATE;
}

/**
 * ccid4_hc_tx_update_x  -  Update allowed sending rate X
 * @stamp: most recent time if available - can be left NULL.
 *
 * This function tracks draft rfc3448bis, check there for latest details.
 *
 * Note: X and X_recv are both stored in units of 64 * bytes/second, to support
 *       fine-grained resolution of sending rates. This requires scaling by 2^6
 *       throughout the code. Only X_calc is unscaled (in bytes/second).
 *
 */
static void ccid4_hc_tx_update_x(struct sock *sk, ktime_t *stamp)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	__u64 min_rate = 2 * hc->tx_x_recv;
	const __u64 old_x = hc->tx_x;
	ktime_t now = stamp ? *stamp : ktime_get_real();

	/*
	 * Handle IDLE periods: do not reduce below RFC3390 initial sending rate
	 * when idling [RFC 4342, 5.1]. Definition of idling is from rfc3448bis:
	 * a sender is idle if it has not sent anything over a 2-RTT-period.
	 * For consistency with X and X_recv, min_rate is also scaled by 2^6.
	 */
	if (tfrc_hc_tx_idle_rtt(hc, now) >= 2) {
		min_rate = rfc3390_initial_rate(sk);
		min_rate = max(min_rate, 2 * hc->tx_x_recv);
	}

	if (hc->tx_p > 0) {

		hc->tx_x = min(((__u64)hc->tx_x_calc) << 6, min_rate);
		/*
		 * CCID-4 Header Penalty:
		 * Adjust sending rate according to (TFRC-SP, Section 5)
		 */
		hc->tx_x = div_u64(hc->tx_x * hc->tx_s, hc->tx_s + CCID4HCTX_H);

	} else if (ktime_us_delta(now, hc->tx_t_ld) - (s64)hc->tx_rtt >= 0) {

		hc->tx_x = min(2 * hc->tx_x, min_rate);
		hc->tx_x = max(hc->tx_x,
			       scaled_div(((__u64)hc->tx_s) << 6, hc->tx_rtt));
		hc->tx_t_ld = now;
	}

	if (hc->tx_x != old_x) {
		ccid4_pr_debug("X_prev=%u, X_now=%u, X_calc=%u, "
			       "X_recv=%u\n", (unsigned int)(old_x >> 6),
			       (unsigned int)(hc->tx_x >> 6), hc->tx_x_calc,
			       (unsigned int)(hc->tx_x_recv >> 6));

		ccid4_update_send_interval(hc);
	}
}

/**
 *	ccid4_hc_tx_measure_packet_size  -  Measuring the packet size `s'
 *	@new_len: DCCP payload size in bytes (not used by all methods)
 *
 *	See ccid3.c for details.
 */
static u32 ccid4_hc_tx_measure_packet_size(struct sock *sk, const u16 new_len)
{
#if   defined(CONFIG_IP_DCCP_CCID4_MEASURE_S_AS_AVG)
	return tfrc_ewma(tfrc_hc_tx_sk(sk)->tx_s, new_len, 9);
#elif defined(CONFIG_IP_DCCP_CCID4_MEASURE_S_AS_MAX)
	return max(tfrc_hc_tx_sk(sk)->tx_s, new_len);
#else /* CONFIG_IP_DCCP_CCID4_MEASURE_S_AS_MPS	*/
	return dccp_sk(sk)->dccps_mss_cache;
#endif
}

static void ccid4_hc_tx_no_feedback_timer(unsigned long data)
{
	struct sock *sk = (struct sock *)data;
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	unsigned long t_nfb = USEC_PER_SEC / 5;

	bh_lock_sock(sk);
	if (sock_owned_by_user(sk)) {
		/* Try again later. */
		/* XXX: set some sensible MIB */
		goto restart_timer;
	}

	ccid4_pr_debug("%s(%p) entry with%s feedback\n", dccp_role(sk), sk,
		       hc->tx_feedback ? "" : "out");

	/* Ignore and do not restart after leaving the established state */
	if ((1 << sk->sk_state) & ~(DCCPF_OPEN | DCCPF_PARTOPEN))
		goto out;

	/* Reset feedback state to "no feedback received" */
	hc->tx_feedback = false;

	/*
	 * Determine new allowed sending rate X as per draft rfc3448bis-00, 4.4
	 * RTO is 0 if and only if no feedback has been received yet.
	 */
	if (hc->tx_t_rto == 0 || hc->tx_p == 0) {

		/* halve send rate directly */
		hc->tx_x /= 2;
		ccid4_update_send_interval(hc);

	} else {
		/*
		 *  Modify the cached value of X_recv
		 *
		 *  If (X_calc > 2 * X_recv)
		 *    X_recv = max(X_recv / 2, s / (2 * t_mbi));
		 *  Else
		 *    X_recv = X_calc / 4;
		 *
		 *  Note that X_recv is scaled by 2^6 while X_calc is not
		 */
		if (hc->tx_x_calc > (hc->tx_x_recv >> 5))
			hc->tx_x_recv /= 2;
		else {
			hc->tx_x_recv = hc->tx_x_calc;
			hc->tx_x_recv <<= 4;
		}
		ccid4_hc_tx_update_x(sk, NULL);
	}
	ccid4_pr_debug("Reduced X to %llu/64 bytes/sec\n",
			(unsigned long long)hc->tx_x);

	/*
	 * Set new timeout for the nofeedback timer.
	 * See comments in packet_recv() regarding the value of t_RTO.
	 */
	if (unlikely(hc->tx_t_rto == 0))	/* no feedback received yet */
		t_nfb = TFRC_INITIAL_TIMEOUT;
	else
		t_nfb = max(hc->tx_t_rto, 2 * hc->tx_t_ipi);

restart_timer:
	sk_reset_timer(sk, &hc->tx_no_feedback_timer,
			   jiffies + usecs_to_jiffies(t_nfb));
out:
	bh_unlock_sock(sk);
	sock_put(sk);
}

/**
 * ccid4_hc_tx_send_packet  -  Delay-based dequeueing of TX packets
 * @skb: next packet candidate to send on @sk
 *
 * This function uses the convention of ccid_packet_dequeue_eval() and
 * returns a millisecond-delay value between 0 and t_mbi = 64000 msec.
 */
static int ccid4_hc_tx_send_packet(struct sock *sk, struct sk_buff *skb)
{
	struct dccp_sock *dp = dccp_sk(sk);
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	ktime_t now = ktime_get_real();
	s64 delay;

	/*
	 * This function is called only for Data and DataAck packets. Sending
	 * zero-sized Data(Ack)s is theoretically possible, but for congestion
	 * control this case is pathological - ignore it.
	 */
	if (unlikely(skb->len == 0))
		return -EBADMSG;

	if (hc->tx_s == 0) {
		sk_reset_timer(sk, &hc->tx_no_feedback_timer, (jiffies +
			       usecs_to_jiffies(TFRC_INITIAL_TIMEOUT)));
		hc->tx_last_win_count	= 0;
		hc->tx_t_last_win_count = now;

		/* Set t_0 for initial packet */
		hc->tx_t_nom = now;

		/*
		 * Use initial RTT sample when available: recommended by erratum
		 * to RFC 4342. This implements the initialisation procedure of
		 * draft rfc3448bis, section 4.2. Remember, X is scaled by 2^6.
		 */
		if (dp->dccps_syn_rtt) {
			ccid4_pr_debug("SYN RTT = %uus\n", dp->dccps_syn_rtt);
			hc->tx_rtt  = dp->dccps_syn_rtt;
			hc->tx_x    = rfc3390_initial_rate(sk);
			hc->tx_t_ld = now;
		} else {
			/*
			 * Sender does not have RTT sample:
			 * - set fallback RTT (RFC 4340, 3.4) since a RTT value
			 *   is needed in several parts (e.g.  window counter);
			 * - set sending rate X_pps = 1pps as per RFC 3448, 4.2.
			 */
			hc->tx_rtt = DCCP_FALLBACK_RTT;
			hc->tx_x   = dp->dccps_mss_cache;
			hc->tx_x <<= 6;
		}

		/* Compute t_ipi = s / X */
		hc->tx_s = ccid4_hc_tx_measure_packet_size(sk, skb->len);
		ccid4_update_send_interval(hc);

		/* Seed value for Oscillation Prevention (sec. 4.5) */
		hc->tx_r_sqmean = tfrc_scaled_sqrt(hc->tx_rtt);

	} else {
		delay = ktime_us_delta(hc->tx_t_nom, now);
		ccid4_pr_debug("delay=%ld\n", (long)delay);
		/*
		 *	Scheduling of packet transmissions (RFC 5348, 8.3)
		 *
		 * if (t_now > t_nom - delta)
		 *       // send the packet now
		 * else
		 *       // send the packet in (t_nom - t_now) milliseconds.
		 */
		if (delay >= TFRC_T_DELTA)
			return (u32)delay / USEC_PER_MSEC;

		tfrc_sp_hc_tx_update_win_count(hc, now);
	}

	if (dccp_data_packet(skb))
		DCCP_SKB_CB(skb)->dccpd_ecn =
			tfrc_sp_get_random_ect(&hc->tx_li_data,
					       DCCP_SKB_CB(skb)->dccpd_seq);

	/* prepare to send now (add options etc.) */
	dp->dccps_hc_tx_insert_options = 1;
	DCCP_SKB_CB(skb)->dccpd_ccval  = hc->tx_last_win_count;

	/* set the nominal send time for the next following packet */
	hc->tx_t_nom = ktime_add_us(hc->tx_t_nom, hc->tx_t_ipi);
	return CCID_PACKET_SEND_AT_ONCE;
}

static void ccid4_hc_tx_packet_sent(struct sock *sk, unsigned int len)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);

	/* Changes to s will become effective the next time X is computed */
	hc->tx_s = ccid4_hc_tx_measure_packet_size(sk, len);

	if (tfrc_sp_tx_hist_add(&hc->tx_hist, dccp_sk(sk)->dccps_gss,
		hc->tx_last_win_count))
			DCCP_CRIT("packet history - out of memory!");
}

static void ccid4_hc_tx_packet_recv(struct sock *sk, struct sk_buff *skb)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	struct tfrc_tx_hist_entry *acked, *old;
	ktime_t now;
	unsigned long t_nfb;
	u32 r_sample;

	/* we are only interested in ACKs */
	if (!(DCCP_SKB_CB(skb)->dccpd_type == DCCP_PKT_ACK ||
	      DCCP_SKB_CB(skb)->dccpd_type == DCCP_PKT_DATAACK))
		return;
	/*
	 * Locate the acknowledged packet in the TX history.
	 *
	 * Returning "entry not found" here can for instance happen when
	 *  - the host has not sent out anything (e.g. a passive server),
	 *  - the Ack is outdated (packet with higher Ack number was received),
	 *  - it is a bogus Ack (for a packet not sent on this connection).
	 */
	acked = tfrc_tx_hist_find_entry(hc->tx_hist, dccp_hdr_ack_seq(skb));
	if (acked == NULL)
		return;
	/* For the sake of RTT sampling, ignore/remove all older entries */
	old = tfrc_tx_hist_two_rtt_old(hc->tx_hist,
				       DCCP_SKB_CB(skb)->dccpd_ccval);
	if (old != NULL)
		tfrc_sp_tx_hist_purge(&old->next);

	/* Update the moving average for the RTT estimate (RFC 3448, 4.3) */
	now	  = ktime_get_real();
	r_sample  = dccp_sample_rtt(sk, ktime_us_delta(now, acked->stamp));
	hc->tx_rtt = tfrc_ewma(hc->tx_rtt, r_sample, 9);

	/*
	 * Update allowed sending rate X as per draft rfc3448bis-00, 4.2/3
	 */
	if (!hc->tx_feedback) {
		hc->tx_feedback = true;

		if (hc->tx_t_rto == 0) {
			/*
			 * Initial feedback packet: Larger Initial Windows (4.2)
			 */
			hc->tx_x    = rfc3390_initial_rate(sk);
			hc->tx_t_ld = now;

			ccid4_update_send_interval(hc);

			goto done_computing_x;
		} else if (hc->tx_p == 0) {
			/*
			 * First feedback after nofeedback timer expiry (4.3)
			 */
			goto done_computing_x;
		}
	}

	/* Update sending rate (step 4 of [RFC 3448, 4.3]) */
	if (hc->tx_p > 0)
		hc->tx_x_calc = tfrc_sp_calc_x(NOM_PACKET_SIZE, hc->tx_rtt, hc->tx_p);
	ccid4_hc_tx_update_x(sk, &now);

done_computing_x:
	ccid4_pr_debug("%s(%p), RTT=%uus (sample=%uus), s=%u, "
			       "p=%u, X_calc=%u, X_recv=%u, X=%u\n",
			       dccp_role(sk), sk, hc->tx_rtt, r_sample,
			       hc->tx_s, hc->tx_p, hc->tx_x_calc,
			       (unsigned int)(hc->tx_x_recv >> 6),
			       (unsigned int)(hc->tx_x >> 6));
	/*
	 * Oscillation Reduction (RFC 3448, 4.5) - modifying t_ipi according to
	 * RTT changes, multiplying by X/X_inst = sqrt(R_sample)/R_sqmean. This
	 * can be useful if few connections share a link, avoiding that buffer
	 * fill levels (RTT) oscillate as a result of frequent adjustments to X.
	 * A useful presentation with background information is in
	 *    Joerg Widmer, "Equation-Based Congestion Control",
	 *    MSc Thesis, University of Mannheim, Germany, 2000
	 * (sec. 3.6.4), who calls this ISM ("Inter-packet Space Modulation").
	 */
	if (ccid4_osc_prev) {
		r_sample = tfrc_scaled_sqrt(r_sample);
		/*
		 * The modulation can work in both ways: increase/decrease t_ipi
		 * according to long-term increases/decreases of the RTT. The
		 * former is a useful measure, since it works against queue
		 * build-up. The latter temporarily increases the sending rate,
		 * so that buffers fill up more quickly. This in turn causes
		 * the RTT to increase, so that either later reduction becomes
		 * necessary or the RTT stays at a very high level. Decreasing
		 * t_ipi is therefore not supported.
		 * Furthermore, during the initial slow-start phase the RTT
		 * naturally increases, where using the algorithm would cause
		 * delays. Hence it is disabled during the initial slow-start.
		 */
		if (r_sample > hc->tx_r_sqmean && hc->tx_p > 0)
			hc->tx_t_ipi = div_u64((u64)hc->tx_t_ipi * (u64)r_sample,
					       hc->tx_r_sqmean);
		hc->tx_t_ipi = min_t(u32, hc->tx_t_ipi, TFRC_T_MBI);
		/* update R_sqmean _after_ computing the modulation factor */
		hc->tx_r_sqmean = tfrc_ewma(hc->tx_r_sqmean, r_sample, 9);
	}

	/* unschedule no feedback timer */
	sk_stop_timer(sk, &hc->tx_no_feedback_timer);

	/*
	 * As we have calculated new ipi, delta, t_nom it is possible
	 * that we now can send a packet, so wake up dccp_wait_for_ccid
	 */
	sk->sk_write_space(sk);

	/*
	 * Update timeout interval for the nofeedback timer. In order to control
	 * rate halving on networks with very low RTTs (<= 1 ms), use per-route
	 * tunable RTAX_RTO_MIN value as the lower bound.
	 */
	hc->tx_t_rto = max_t(u32, 4 * hc->tx_rtt,
				  USEC_PER_SEC/HZ * tcp_rto_min(sk));
	/*
	 * Schedule no feedback timer to expire in
	 * max(t_RTO, 2 * s/X)  =  max(t_RTO, 2 * t_ipi)
	 */
	t_nfb = max(hc->tx_t_rto, 2 * hc->tx_t_ipi);

	ccid4_pr_debug("%s(%p), Scheduled no feedback timer to "
		       "expire in %lu jiffies (%luus)\n",
		       dccp_role(sk), sk, usecs_to_jiffies(t_nfb), t_nfb);

	sk_reset_timer(sk, &hc->tx_no_feedback_timer,
			   jiffies + usecs_to_jiffies(t_nfb));
}

static int ccid4_hc_tx_parse_options(struct sock *sk, u8 packet_type,
				     u8 option, u8 *optval, u8 optlen)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	struct sk_buff *skb;
	u32 new_p;
	__be32 opt_val;

	switch (option) {
	case TFRC_OPT_RECEIVE_RATE:
	case TFRC_OPT_LOSS_EVENT_RATE:
		/* Must be ignored on Data packets, cf. RFC 4342 8.3 and 8.5 */
		if (packet_type == DCCP_PKT_DATA)
			break;
		if (unlikely(optlen != 4)) {
			DCCP_WARN("%s(%p), invalid len %d for %u\n",
				  dccp_role(sk), sk, optlen, option);
			return -EINVAL;
		}
		opt_val = ntohl(get_unaligned((__be32 *)optval));

		if (option == TFRC_OPT_RECEIVE_RATE) {
			/* Receive Rate is kept in units of 64 bytes/second */
			hc->tx_x_recv = opt_val;
			hc->tx_x_recv <<= 6;

			ccid4_pr_debug("%s(%p), RECEIVE_RATE=%u\n",
				       dccp_role(sk), sk, opt_val);
		} else {
			/* Update the fixpoint Loss Event Rate fraction */
			hc->tx_p = tfrc_sp_invert_loss_event_rate(opt_val);

			ccid4_pr_debug("%s(%p), LOSS_EVENT_RATE=%u\n",
				       dccp_role(sk), sk, opt_val);
		}
		break;
	case TFRC_OPT_DROPPED_PACKETS:
		tfrc_sp_parse_dropped_packets_opt(&hc->tx_li_data,
						  optval, optlen);

		skb = skb_peek(&sk->sk_receive_queue);

		if (skb == NULL)
			break;

		if (!tfrc_sp_check_ecn_sum(&hc->tx_li_data,
					   optval, optlen, skb)) {
			/*
			 * TODO: consider ecn sum test fail
			 * and update allowed sending rate
			 */
		}

		new_p =
		tfrc_sp_p_from_loss_intervals_opt(&hc->tx_li_data,
						  hc->tx_hist,
						  hc->tx_last_win_count,
						  DCCP_SKB_CB(skb)->dccpd_seq);
		if (hc->tx_p != new_p) {
			/*
			 * TODO: use p value obtained
			 * from loss intervals option
			 */
		}

		break;
	case TFRC_OPT_LOSS_INTERVALS:

		hc->tx_li_data.skip_length = *optval;
		tfrc_sp_parse_loss_intervals_opt(&hc->tx_li_data,
						 optval, optlen);

		skb = skb_peek(&sk->sk_receive_queue);

		if (skb == NULL)
			break;

		if (!tfrc_sp_check_ecn_sum(&hc->tx_li_data,
					   optval, optlen, skb)) {
			/*
			 * TODO: consider ecn sum test fail
			 * and update allowed sending rate
			 */
		}

		new_p =
		tfrc_sp_p_from_loss_intervals_opt(&hc->tx_li_data,
						  hc->tx_hist,
						  hc->tx_last_win_count,
						  DCCP_SKB_CB(skb)->dccpd_seq);
		if (hc->tx_p != new_p) {
			/*
			 * TODO: use p value obtained
			 * from loss intervals option
			 */
		}
		break;
	}
	return 0;
}

static int ccid4_hc_tx_init(struct ccid *ccid, struct sock *sk)
{
	struct tfrc_hc_tx_sock *hc = ccid_priv(ccid);

	hc->tx_hist  = NULL;
	setup_timer(&hc->tx_no_feedback_timer,
			ccid4_hc_tx_no_feedback_timer, (unsigned long)sk);
	return 0;
}

static void ccid4_hc_tx_exit(struct sock *sk)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);

	sk_stop_timer(sk, &hc->tx_no_feedback_timer);
	tfrc_sp_tx_hist_purge(&hc->tx_hist);
	tfrc_sp_tx_ld_cleanup(&hc->tx_li_data.ecn_sums_head);
}

static void ccid4_hc_tx_get_info(struct sock *sk, struct tcp_info *info)
{
	info->tcpi_rto = tfrc_hc_tx_sk(sk)->tx_t_rto;
	info->tcpi_rtt = tfrc_hc_tx_sk(sk)->tx_rtt;
}

static int ccid4_hc_tx_getsockopt(struct sock *sk, const int optname, int len,
				  u32 __user *optval, int __user *optlen)
{
	const struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);
	struct tfrc_tx_info tfrc;
	const void *val;

	switch (optname) {
	case DCCP_SOCKOPT_CCID_TX_INFO:
		if (len < sizeof(tfrc))
			return -EINVAL;
		tfrc.tfrctx_x	   = hc->tx_x;
		tfrc.tfrctx_x_recv = hc->tx_x_recv;
		tfrc.tfrctx_x_calc = hc->tx_x_calc;
		tfrc.tfrctx_rtt	   = hc->tx_rtt;
		tfrc.tfrctx_p	   = hc->tx_p;
		tfrc.tfrctx_rto	   = hc->tx_t_rto;
		tfrc.tfrctx_ipi	   = hc->tx_t_ipi;
		len = sizeof(tfrc);
		val = &tfrc;
		break;
	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen) || copy_to_user(optval, val, len))
		return -EFAULT;

	return 0;
}

/*
 *	Receiver Half-Connection Routines
 */
static void ccid4_hc_rx_send_feedback(struct sock *sk,
				      const struct sk_buff *skb,
				      enum tfrc_fback_type fbtype)
{
	struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);

	switch (fbtype) {
	case TFRC_FBACK_INITIAL:
		hc->rx_x_recv = 0;
		hc->rx_pinv   = ~0U;   /* see RFC 4342, 8.5 */
		break;
	case TFRC_FBACK_PARAM_CHANGE:
		if (unlikely(hc->rx_feedback == TFRC_FBACK_NONE)) {
			/*
			 * rfc3448bis-06, 6.3.1: First packet(s) lost or marked
			 * FIXME: in rfc3448bis the receiver returns X_recv=0
			 * here as it normally would in the first feedback packet.
			 * However this is not possible yet, since the code still
			 * uses RFC 3448, i.e.
			 *    If (p > 0)
			 *      Calculate X_calc using the TCP throughput equation.
			 *      X = max(min(X_calc, 2*X_recv), s/t_mbi);
			 * would bring X down to s/t_mbi. That is why we return
			 * X_recv according to rfc3448bis-06 for the moment.
			 */
			u32 s = tfrc_rx_hist_packet_size(&hc->rx_hist),
			    rtt = tfrc_rx_hist_rtt(&hc->rx_hist);

			hc->rx_x_recv = scaled_div32(s, 2 * rtt);
			break;
		}
		/*
		 * When parameters change (new loss or p > p_prev), we do not
		 * have a reliable estimate for R_m of [RFC 3448, 6.2] and so
		 * always check whether at least RTT time units were covered.
		 */
		hc->rx_x_recv = tfrc_sp_rx_hist_x_recv(&hc->rx_hist, hc->rx_x_recv);
		break;
	case TFRC_FBACK_PERIODIC:
		/*
		 * Step (2) of rfc3448bis-06, 6.2:
		 * - if no data packets have been received, just restart timer
		 * - if data packets have been received, re-compute X_recv
		 */
		if (hc->rx_hist.bytes_recvd == 0)
			goto prepare_for_next_time;
		hc->rx_x_recv = tfrc_sp_rx_hist_x_recv(&hc->rx_hist, hc->rx_x_recv);
		break;
	default:
		return;
	}

	ccid4_pr_debug("X_recv=%u, 1/p=%u\n", hc->rx_x_recv, hc->rx_pinv);

	dccp_sk(sk)->dccps_hc_rx_insert_options = 1;
	dccp_send_ack(sk);

prepare_for_next_time:
	tfrc_rx_hist_restart_byte_counter(&hc->rx_hist);
	hc->rx_last_counter = dccp_hdr(skb)->dccph_ccval;
	hc->rx_feedback	    = fbtype;
}

static int ccid4_hc_rx_insert_options(struct sock *sk, struct sk_buff *skb)
{
	u16 dropped_length, loss_intervals_length;
	struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);
	__be32 x_recv, pinv;

	if (!(sk->sk_state == DCCP_OPEN || sk->sk_state == DCCP_PARTOPEN))
		return 0;

	if (dccp_packet_without_ack(skb))
		return 0;

	x_recv = htonl(hc->rx_x_recv);
	pinv   = htonl(hc->rx_pinv);

	loss_intervals_length	=
		(hc->rx_li_data.counter > TFRC_LOSS_INTERVALS_OPT_MAX_LENGTH) ?
		 TFRC_LOSS_INTERVALS_OPT_MAX_LENGTH : hc->rx_li_data.counter;
	dropped_length		=
		(hc->rx_li_data.counter > TFRC_DROP_OPT_MAX_LENGTH) ?
		 TFRC_DROP_OPT_MAX_LENGTH : hc->rx_li_data.counter;

	tfrc_sp_ld_prepare_data(hc->rx_hist.loss_count, &hc->rx_li_data);

	if (dccp_insert_option(skb, TFRC_OPT_LOSS_EVENT_RATE,
			       &pinv, sizeof(pinv)) ||
	    dccp_insert_option(skb, TFRC_OPT_RECEIVE_RATE,
			       &x_recv, sizeof(x_recv)) ||
	    dccp_insert_option(skb, TFRC_OPT_LOSS_INTERVALS,
			       &hc->rx_li_data.loss_intervals_opts[0],
			       1 + loss_intervals_length*9) ||
	    dccp_insert_option(skb, TFRC_OPT_DROPPED_PACKETS,
			       &hc->rx_li_data.drop_opts[0], dropped_length*3))
		return -1;

	return 0;
}

/**
 * ccid4_first_li  -  Implements [RFC 5348, 6.3.1]
 *
 * Determine the length of the first loss interval via inverse lookup.
 * Assume that X_recv can be computed by the throughput equation
 *		    s
 *	X_recv = --------
 *		 R * fval
 * Find some p such that f(p) = fval; return 1/p (scaled).
 */
static u32 ccid4_first_li(struct sock *sk)
{
	struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);
	u32 rtt = tfrc_rx_hist_rtt(&hc->rx_hist), x_recv, p;
	u64 fval;

	/*
	 * rfc3448bis-06, 6.3.1: First data packet(s) are marked or lost. Set p
	 * to give the equivalent of X_target = s/(2*R). Thus fval = 2 and so p
	 * is about 20.64%. This yields an interval length of 4.84 (rounded up).
	 */
	if (unlikely(hc->rx_feedback == TFRC_FBACK_NONE))
		return 5;

	x_recv = tfrc_sp_rx_hist_x_recv(&hc->rx_hist, hc->rx_x_recv);
	if (x_recv == 0)
		goto failed;

	fval = scaled_div32(scaled_div(NOM_PACKET_SIZE, rtt), x_recv);
	p = tfrc_sp_calc_x_reverse_lookup(fval);

	ccid4_pr_debug("%s(%p), receive rate=%u bytes/s, implied "
		       "loss rate=%u\n", dccp_role(sk), sk, x_recv, p);

	if (p > 0)
		return scaled_div(1, p);
failed:
	return UINT_MAX;
}

static void ccid4_hc_rx_packet_recv(struct sock *sk, struct sk_buff *skb)
{
	struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);
	const u64 ndp = dccp_sk(sk)->dccps_options_received.dccpor_ndp;
	const bool is_data_packet = dccp_data_packet(skb);

	/*
	 * Perform loss detection and handle pending losses
	 */
	if (tfrc_sp_rx_congestion_event(&hc->rx_hist, &hc->rx_li_hist,
					&hc->rx_li_data,
					skb, ndp, ccid4_first_li, sk))
		ccid4_hc_rx_send_feedback(sk, skb, TFRC_FBACK_PARAM_CHANGE);
	/*
	 * Feedback for first non-empty data packet (RFC 3448, 6.3)
	 */
	else if (unlikely(hc->rx_feedback == TFRC_FBACK_NONE && is_data_packet))
		ccid4_hc_rx_send_feedback(sk, skb, TFRC_FBACK_INITIAL);
	/*
	 * Check if the periodic once-per-RTT feedback is due; RFC 4342, 10.3
	 */
	else if (!tfrc_rx_hist_loss_pending(&hc->rx_hist) && is_data_packet &&
		 SUB16(dccp_hdr(skb)->dccph_ccval, hc->rx_last_counter) > 3)
		ccid4_hc_rx_send_feedback(sk, skb, TFRC_FBACK_PERIODIC);
}

static int ccid4_hc_rx_init(struct ccid *ccid, struct sock *sk)
{
	struct tfrc_hc_rx_sock *hc = ccid_priv(ccid);

	tfrc_lh_init(&hc->rx_li_hist);
	tfrc_ld_init(&hc->rx_li_data);

	return tfrc_sp_rx_hist_init(&hc->rx_hist, sk);
}

static void ccid4_hc_rx_exit(struct sock *sk)
{
	struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);

	tfrc_sp_rx_hist_purge(&hc->rx_hist);
	tfrc_sp_lh_cleanup(&hc->rx_li_hist);
	tfrc_sp_ld_cleanup(&hc->rx_li_data);
}

static void ccid4_hc_rx_get_info(struct sock *sk, struct tcp_info *info)
{
	info->tcpi_options  |= TCPI_OPT_TIMESTAMPS;
	info->tcpi_rcv_rtt  = tfrc_rx_hist_rtt(&tfrc_hc_rx_sk(sk)->rx_hist);
}

static int ccid4_hc_rx_getsockopt(struct sock *sk, const int optname, int len,
				  u32 __user *optval, int __user *optlen)
{
	const struct tfrc_hc_rx_sock *hc = tfrc_hc_rx_sk(sk);
	struct tfrc_rx_info rx_info;
	const void *val;

	switch (optname) {
	case DCCP_SOCKOPT_CCID_RX_INFO:
		if (len < sizeof(rx_info))
			return -EINVAL;
		rx_info.tfrcrx_x_recv = hc->rx_x_recv;
		rx_info.tfrcrx_rtt    = tfrc_rx_hist_rtt(&hc->rx_hist);
		rx_info.tfrcrx_p      = tfrc_sp_invert_loss_event_rate(hc->rx_pinv);
		len = sizeof(rx_info);
		val = &rx_info;
		break;
	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen) || copy_to_user(optval, val, len))
		return -EFAULT;

	return 0;
}

struct ccid_operations ccid4_ops = {
	.ccid_id		   = DCCPC_CCID4,
	.ccid_name		   = "TCP-Friendly Rate Control (Small-Packet variant)",
	.ccid_hc_tx_obj_size	   = sizeof(struct tfrc_hc_tx_sock),
	.ccid_hc_tx_init	   = ccid4_hc_tx_init,
	.ccid_hc_tx_exit	   = ccid4_hc_tx_exit,
	.ccid_hc_tx_send_packet	   = ccid4_hc_tx_send_packet,
	.ccid_hc_tx_packet_sent	   = ccid4_hc_tx_packet_sent,
	.ccid_hc_tx_probe	   = tfrc_sp_hc_tx_probe,
	.ccid_hc_tx_packet_recv	   = ccid4_hc_tx_packet_recv,
	.ccid_hc_tx_parse_options  = ccid4_hc_tx_parse_options,
	.ccid_hc_rx_obj_size	   = sizeof(struct tfrc_hc_rx_sock),
	.ccid_hc_rx_init	   = ccid4_hc_rx_init,
	.ccid_hc_rx_exit	   = ccid4_hc_rx_exit,
	.ccid_hc_rx_insert_options = ccid4_hc_rx_insert_options,
	.ccid_hc_rx_packet_recv	   = ccid4_hc_rx_packet_recv,
	.ccid_hc_rx_get_info	   = ccid4_hc_rx_get_info,
	.ccid_hc_tx_get_info	   = ccid4_hc_tx_get_info,
	.ccid_hc_rx_getsockopt	   = ccid4_hc_rx_getsockopt,
	.ccid_hc_tx_getsockopt	   = ccid4_hc_tx_getsockopt,
};

module_param(ccid4_osc_prev, bool, 0644);
MODULE_PARM_DESC(ccid4_osc_prev, "Oscillation Prevention for CCID-4 (RFC 3448, 4.5)");

#ifdef CONFIG_IP_DCCP_CCID4_DEBUG
module_param(ccid4_debug, bool, 0644);
MODULE_PARM_DESC(ccid4_debug, "Enable CCID-4 debug messages");
#endif
