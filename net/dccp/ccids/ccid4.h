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
#ifndef _DCCP_CCID4_H_
#define _DCCP_CCID4_H_

#include "lib/tfrc_ccids_sp.h"

/* The nominal packet size to be used into TFRC equation as per CCID-4 draft*/
#define NOM_PACKET_SIZE            1460

/* Mininum sending rate as per CCID-4 draft */
#define MIN_SEND_RATE              10000

/* The header size on data packets is estimated as 36 bytes as per CCID-4
 * draft, [Section 5].
 */
#define CCID4HCTX_H	36

#endif /* _DCCP_CCID4_H_ */
