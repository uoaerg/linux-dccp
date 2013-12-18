/*
 *  Copyright (c) 2011   Federal University of Campina Grande, Paraiba, BR.
 *
 *  An implementation of the DCCP protocol
 *
 *  Copyright (c) 2011 Ivo Calado, Leandro Sales
 *
 *  This code has been developed by the Federal University of Campina Grande
 *  Embedded Systems and Pervasive Computing Lab. For further information
 *  please see http://embeddedlab.org/
 *  <ivocalado@ee.ufcg.edu.br> <leandroal@gmail.com>
 *
 *  This code is a version of the Cubic algorithm to the DCCP protocol.
 *  Due to that, it copies as much code as possible from net/ipv4/tcp_cubic.c
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
#ifndef _DCCP_CCID5_H_
#define _DCCP_CCID5_H_

#define NUMDUPACK	3

/*
 * CCID-5 timestamping faces the same issues as TCP timestamping.
 * Hence we reuse/share as much of the code as possible.
 */
#define ccid5_time_stamp	tcp_time_stamp

struct ccid5_seq {
	u64			ccid5s_seq;
	u32			ccid5s_sent;
	int			ccid5s_acked;
	struct ccid5_seq	*ccid5s_prev;
	struct ccid5_seq	*ccid5s_next;
};

#define CCID5_SEQBUF_LEN 1024
#define CCID5_SEQBUF_MAX 128

/*
 * Multiple of congestion window to keep the sequence window at
 * (RFC 4340 7.5.2)
 */
#define CCID5_WIN_CHANGE_FACTOR 5

struct ccid5_hc_tx_sock {

	/* increase cwnd by 1 after ACKs */
	u32			tx_cnt;
	/* last maximum tx_cwnd */
	u32			tx_last_max_cwnd;
	/* congestion window at last loss */
	u32			tx_loss_cwnd;
	/* the last snd_cwnd */
	u32			tx_last_cwnd;
	/* time when updated tx_last_cwnd */
	u32			tx_last_time;
	/* origin point of bic function */
	u32			tx_bic_origin_point;
	/* time to origin point from the beginning of the current epoch */
	u32			tx_bic_K;
	/* min delay */
	u32			tx_delay_min;
	/* beginning of an epoch */
	u32			tx_epoch_start;
	/* number of acks */
	u32			tx_ack_cnt;
	/* estimated tcp cwnd */
	u32			tcp_cwnd;
	/* simulates tcp's cwnd */
	u32			tx_cwnd;
	/* simulates tcp's cwnd count */
	u32			tx_cwnd_cnt;
	/* slow start threshold */
	u32			tx_ssthresh;
	/* number of packets sent and not acked */
	u32			tx_pipe;

	/* Implementation of Hystart algorithm */
	/* the exit point is found? */
	u8			tx_found;
	/* beginning of each round */
	u32			tx_round_start;
	/* last time when the ACK spacing is close */
	u32			tx_last_jiffies;
	/* the minimum rtt of current round */
	u32			tx_curr_rtt;
	/* number of samples to decide curr_rtt */
	u8			tx_sample_cnt;


	/* RTT measurement: variables/principles are the same as in TCP */
	u32			tx_srtt,
				tx_mdev,
				tx_mdev_max,
				tx_rttvar,
				tx_rto;
	u64			tx_rtt_seq:48;
	struct timer_list	tx_rtotimer;

	u64			tx_rpseq;
	int			tx_rpdupack;
	u32			tx_last_cong;
	u64			tx_high_ack;
	struct list_head	tx_av_chunks;

	/* Congestion Window validation (optional, RFC 2861) */
	u32			tx_cwnd_used,
				tx_expected_wnd,
				tx_cwnd_stamp,
				tx_lsndtime;

	u32			tx_packets_acked;
	struct ccid5_seq	*tx_seqbuf[CCID5_SEQBUF_MAX];
	int			tx_seqbufc;
	struct ccid5_seq	*tx_seqh;
	struct ccid5_seq	*tx_seqt;
};

/**
 * struct ccid5_hc_rx_sock  -  Receiving end of CCID-2 half-connection
 * @rx_num_data_pkts: number of data packets received since last feedback
 */
struct ccid5_hc_rx_sock {
	u32	rx_num_data_pkts;
};

static inline bool ccid5_cwnd_network_limited(struct ccid5_hc_tx_sock *hc)
{
	return hc->tx_pipe >= hc->tx_cwnd;
}

/*
 * Convert RFC 3390 larger initial window into an equivalent number of packets.
 * This is based on the numbers specified in RFC 5681, 3.1.
 */
static inline u32 rfc3390_bytes_to_packets(const u32 smss)
{
	return smss <= 1095 ? 4 : (smss > 2190 ? 2 : 3);
}

static inline struct ccid5_hc_tx_sock *ccid5_hc_tx_sk(const struct sock *sk)
{
	return ccid_priv(dccp_sk(sk)->dccps_hc_tx_ccid);
}

static inline struct ccid5_hc_rx_sock *ccid5_hc_rx_sk(const struct sock *sk)
{
	return ccid_priv(dccp_sk(sk)->dccps_hc_rx_ccid);
}

#endif /* _DCCP_CCID5_H_ */
