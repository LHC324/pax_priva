/*****************************************************************************
 *
 * Copyright (C) 2001 Uppsala University & Ericsson AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Erik Nordstr�m, <erik.nordstrom@it.uu.se>
 *
 *****************************************************************************/
/* Definitions for Minimal IP Encapsulation (RFC 2004) */

#ifndef _KAODV_IPENC_H
#define _KAODV_IPENC_H

#include <linux/ip.h>
#include <linux/skbuff.h>
#include <asm/byteorder.h>

#define IPPROTO_MIPE 55

struct min_ipenc_hdr {
    unsigned char protocol;
#if defined(__LITTLE_ENDIAN)
    unsigned char res:7;
    unsigned char s:1;
#elif defined(__BIG_ENDIAN)
    unsigned char s:1;
    unsigned char res:7;
#else 
#error  "Adjust your <asm/byteorder.h> defines"
#endif
    u_int16_t check;
    unsigned int daddr;
 /*    unsigned int saddr; */
};
struct sk_buff *ip_pkt_encapsulate(struct sk_buff *skb, __u32 dest);
struct sk_buff *ip_pkt_decapsulate(struct sk_buff *skb);
#endif				/* _KAODV_IPENC_H */
