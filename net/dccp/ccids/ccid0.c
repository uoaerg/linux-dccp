/*
 *  CCID-ZERO - UDP-like congestion control
 *
 *  This is a sample "no-op" CCID kernel module. It is meant for development
 *  and testing, but not for actual deployment.
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
#include "../ccid.h"

struct ccid_operations ccid0_ops = {
	.ccid_id	= DCCPC_CCID_ZERO,
	.ccid_name	= "UDP-like"
};
