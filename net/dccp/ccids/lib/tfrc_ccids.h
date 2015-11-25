/*
 *  Copyright (c) 2007 Leandro Melo de Sales <leandroal@gmail.com>
 *  Copyright (c) 2005 Ian McDonald <ian.mcdonald@jandi.co.nz>
 *  Copyright (c) 2005 Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *  Copyright (c) 2003 Nils-Erik Mattsson, Joacim Haggmark, Magnus Erixzon
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */
#ifndef _TFRC_CCIDS_H_
#define _TFRC_CCIDS_H_

#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/tfrc.h>
#include <asm/unaligned.h>
#include "tfrc.h"
#include "../../ccid.h"

/* Two seconds as per RFC 5348, 4.2 */
#define TFRC_INITIAL_TIMEOUT	   (2 * USEC_PER_SEC)

/* Maximum backoff interval t_mbi (RFC 3448, 4.3) */
#define TFRC_T_MBI		   (64 * USEC_PER_SEC)

/*
 * The t_delta parameter (RFC 5348, 8.3): delays of less than %USEC_PER_MSEC are
 * rounded down to 0, since sk_reset_timer() here uses millisecond granularity.
 * Hence we can use a constant t_delta = %USEC_PER_MSEC when HZ >= 500. A coarse
 * resolution of HZ < 500 means that the error is below one timer tick (t_gran)
 * when using the constant t_delta  =  t_gran / 2  =  %USEC_PER_SEC / (2 * HZ).
 */
#if (HZ >= 500)
# define TFRC_T_DELTA		   USEC_PER_MSEC
#else
# define TFRC_T_DELTA		   (USEC_PER_SEC / (2 * HZ))
#endif

enum tfrc_options {
	TFRC_OPT_LOSS_EVENT_RATE = 192,
	TFRC_OPT_LOSS_INTERVALS	 = 193,
	TFRC_OPT_RECEIVE_RATE	 = 194,
	TFRC_OPT_DROPPED_PACKETS = 195,	/* as per ccid-4 draft, section 8 */
};

/**
 *
 * struct tfrc_hc_tx_sock - CCID3/4 sender half-connection socket
 * @tx_x:		  Current sending rate in 64 * bytes per second
 * @tx_x_recv:		  Receive rate in 64 * bytes per second
 * @tx_x_calc:		  Calculated rate in bytes per second
 * @tx_rtt:		  Estimate of current round trip time in usecs
 * @tx_r_sqmean:	  Estimate of long-term RTT (RFC 5348, 4.5)
 * @tx_p:		  Current loss event rate (0-1) scaled by 1000000
 * @tx_t_rto:		  Nofeedback Timer setting in usecs
 * @tx_t_ipi:		  Interpacket (send) interval (RFC 3448, 4.6) in usecs
 * @tx_s:		  Packet size in bytes
 * @tx_feedback:	  Whether feedback has been received or not
 * @tx_last_win_count:	  Last window counter sent
 * @tx_t_last_win_count:  Timestamp of earliest packet
 *			  with last_win_count value sent
 * @tx_no_feedback_timer: Handle to no feedback timer
 * @tx_t_ld:		  Time last doubled during slow start
 * @tx_t_nom:		  Nominal send time of next packet
 * @tx_hist:		  Packet history
 */
struct tfrc_hc_tx_sock {
	u64				tx_x;
	u64				tx_x_recv;
	u32				tx_x_calc;
	u32				tx_rtt;
	u16				tx_r_sqmean;
	u32				tx_p;
	u32				tx_t_rto;
	u32				tx_t_ipi;
	u16				tx_s;
	bool				tx_feedback:1;
	u8				tx_last_win_count;
	ktime_t				tx_t_last_win_count;
	struct timer_list		tx_no_feedback_timer;
	ktime_t				tx_t_ld;
	ktime_t				tx_t_nom;
	struct tfrc_tx_hist_entry	*tx_hist;
};

static inline struct tfrc_hc_tx_sock *tfrc_hc_tx_sk(const struct sock *sk)
{
	struct tfrc_hc_tx_sock *hctx = ccid_priv(dccp_sk(sk)->dccps_hc_tx_ccid);
	BUG_ON(hctx == NULL);
	return hctx;
}

/*
 * Compute the initial sending rate X_init in the manner of RFC 3390:
 *
 *	X_init  =  min(4 * MPS, max(2 * MPS, 4380 bytes)) / RTT
 *
 * For consistency with other parts of the code, X_init is scaled by 2^6.
 */
static inline u64 rfc3390_initial_rate(struct sock *sk)
{
	const u32 mps = dccp_sk(sk)->dccps_mss_cache,
	       w_init = clamp(4380U, 2 * mps, 4 * mps);

	return scaled_div(w_init << 6, tfrc_hc_tx_sk(sk)->tx_rtt);
}

void tfrc_hc_tx_update_win_count(struct tfrc_hc_tx_sock *hc, ktime_t now);

size_t tfrc_hc_tx_probe(struct sock *sk, char *buf, const size_t maxlen);

/* CCID3/4 feedback types */
enum tfrc_fback_type {
	TFRC_FBACK_NONE = 0,
	TFRC_FBACK_INITIAL,
	TFRC_FBACK_PERIODIC,
	TFRC_FBACK_PARAM_CHANGE
};

/**
 *
 * struct tfrc_hc_rx_sock - CCID3/4 receiver half-connection socket
 * @rx_last_counter:	     Tracks window counter (RFC 4342, 8.1)
 * @rx_feedback:	     The type of the feedback last sent
 * @rx_x_recv:		     Receiver estimate of send rate (RFC 3448, sec. 4.3)
 * @rx_tstamp_last_feedback: Time at which last feedback was sent
 * @rx_hist:		     Packet history (loss detection + RTT sampling)
 * @rx_li_hist:		     Loss Interval database
 * @rx_pinv:		     Inverse of Loss Event Rate (RFC 4342, sec. 8.5)
 */
struct tfrc_hc_rx_sock {
	u8				rx_last_counter:4;
	enum tfrc_fback_type		rx_feedback:4;
	u32				rx_x_recv;
	ktime_t				rx_tstamp_last_feedback;
	struct tfrc_rx_hist		rx_hist;
	struct tfrc_loss_hist		rx_li_hist;
#define rx_pinv				rx_li_hist.i_mean
};

static inline struct tfrc_hc_rx_sock *tfrc_hc_rx_sk(const struct sock *sk)
{
	struct tfrc_hc_rx_sock *hcrx = ccid_priv(dccp_sk(sk)->dccps_hc_rx_ccid);
	BUG_ON(hcrx == NULL);
	return hcrx;
}

static inline u32 tfrc_hc_tx_idle_rtt(struct tfrc_hc_tx_sock *hc, ktime_t now)
{
	u32 delta = ktime_us_delta(now, hc->tx_t_last_win_count);

	return delta / hc->tx_rtt;
}
#endif /* _TFRC_CCIDS_H_ */
