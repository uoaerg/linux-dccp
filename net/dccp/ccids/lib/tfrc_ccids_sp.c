/*
 *  Copyright (c) 2009 Federal University of Campina Grande, Embedded
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
#include "tfrc_ccids_sp.h"

/*
 *	Update Window Counter using the algorithm from [RFC 4342, 8.1].
 *	As elsewhere, RTT > 0 is assumed by using dccp_sample_rtt().
 */
void tfrc_sp_hc_tx_update_win_count(struct tfrc_hc_tx_sock *hc, ktime_t now)
{
	u32 delta = ktime_us_delta(now, hc->tx_t_last_win_count),
	    quarter_rtts = (4 * delta) / hc->tx_rtt;

	if (quarter_rtts > 0) {
		hc->tx_t_last_win_count = now;
		hc->tx_last_win_count  += min(quarter_rtts, 5U);
		hc->tx_last_win_count  &= 0xF;		/* mod 16 */
	}
}

size_t tfrc_sp_hc_tx_probe(struct sock *sk, char *buf, const size_t maxlen)
{
	struct tfrc_hc_tx_sock *hc = tfrc_hc_tx_sk(sk);

	/* Specific field numbering:
			5   6     7   8        9        10   11
			s   rtt   p   X_calc   X_recv   X    t_ipi  */
	return snprintf(buf, maxlen, " %d %d %d %u %u %u %d",
			hc->tx_s, hc->tx_rtt, hc->tx_p, hc->tx_x_calc,
			(unsigned int)(hc->tx_x_recv >> 6),
			(unsigned int)(hc->tx_x >> 6), hc->tx_t_ipi);
}
