#ifndef _TFRC_SP_H_
#define _TFRC_SP_H_
/*
 *  Copyright (c) 2009 Federal University of Campina Grande, Embedded
 *  Copyright (c) 2007   The University of Aberdeen, Scotland, UK
 *  Copyright (c) 2005-6 The University of Waikato, Hamilton, New Zealand.
 *  Copyright (c) 2005-6 Ian McDonald <ian.mcdonald@jandi.co.nz>
 *  Copyright (c) 2005   Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *  Copyright (c) 2003   Nils-Erik Mattsson, Joacim Haggmark, Magnus Erixzon
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */
#include <linux/types.h>
#include <linux/math64.h>
#include "../../dccp.h"

/* internal includes that this library exports: */
#include "loss_interval_sp.h"
#include "packet_history_sp.h"

#ifdef CONFIG_IP_DCCP_TFRC_SP_DEBUG
extern int tfrc_sp_debug;
#define tfrc_sp_pr_debug(format, a...)	\
	DCCP_PR_DEBUG(tfrc_sp_debug, format, ##a)
#else
#define tfrc_sp_pr_debug(format, a...)
#endif

#ifndef _TFRC_H_
/* integer-arithmetic divisions of type (a * 1000000)/b */
static inline u64 scaled_div(u64 a, u64 b)
{
	BUG_ON(b == 0);
	return div64_u64(a * 1000000, b);
}

static inline u32 scaled_div32(u64 a, u64 b)
{
	u64 result = scaled_div(a, b);

	if (result > UINT_MAX) {
		DCCP_CRIT("Overflow: %llu/%llu > UINT_MAX",
			  (unsigned long long)a, (unsigned long long)b);
		return UINT_MAX;
	}
	return result;
}

/**
 * tfrc_scaled_sqrt  -  Compute scaled integer sqrt(x) for 0 < x < 2^22-1
 * Uses scaling to improve accuracy of the integer approximation of sqrt(). The
 * scaling factor of 2^10 limits the maximum @sample to 4e6; this is okay for
 * clamped RTT samples (dccp_sample_rtt).
 * Should best be used for expressions of type sqrt(x)/sqrt(y), since then the
 * scaling factor is neutralised. For this purpose, it avoids returning zero.
 */
static inline u16 tfrc_scaled_sqrt(const u32 sample)
{
	const unsigned long non_zero_sample = sample ? : 1;

	return int_sqrt(non_zero_sample << 10);
}

/**
 * tfrc_ewma  -  Exponentially weighted moving average
 * @weight: Weight to be used as damping factor, in units of 1/10
 */
static inline u32 tfrc_ewma(const u32 avg, const u32 newval, const u8 weight)
{
	return avg ? (weight * avg + (10 - weight) * newval) / 10 : newval;
}
#endif

u32  tfrc_sp_calc_x(u16 s, u32 R, u32 p);
u32  tfrc_sp_calc_x_reverse_lookup(u32 fvalue);
u32  tfrc_sp_invert_loss_event_rate(u32 loss_event_rate);

int  tfrc_sp_tx_packet_history_init(void);
void tfrc_sp_tx_packet_history_exit(void);
int  tfrc_sp_rx_packet_history_init(void);
void tfrc_sp_rx_packet_history_exit(void);

int  tfrc_sp_li_init(void);
void tfrc_sp_li_exit(void);

#ifdef CONFIG_IP_DCCP_TFRC_SP_LIB
int  tfrc_sp_lib_init(void);
void tfrc_sp_lib_exit(void);
#else
#define tfrc_sp_lib_init() (0)
#define tfrc_sp_lib_exit()
#endif
#endif /* _TFRC_SP_H_ */
