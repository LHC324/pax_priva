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
 *
 *****************************************************************************/
#ifndef _AODV_RREP_H
#define _AODV_RREP_H

#ifndef NS_NO_GLOBALS
#include "endian.h"

#include "defs.h"
#include "routing_table.h"

/* RREP Flags: */

#define RREP_ACK       0x1
#define RREP_REPAIR    0x2

typedef struct {
    unsigned char type;
#if defined(__LITTLE_ENDIAN)
    unsigned short  res1:6;
    unsigned short  a:1;
    unsigned short  r:1;
    unsigned short  prefix:5;
    unsigned short  res2:3;
#elif defined(__BIG_ENDIAN)
    unsigned short  r:1;
    unsigned short  a:1;
    unsigned short  res1:6;
    unsigned short  res2:3;
    unsigned short  prefix:5;
#else
#error "Adjust your <bits/endian.h> defines"
#endif
    unsigned char hcnt;
    unsigned int dest_addr;
    unsigned int dest_seqno;
    unsigned int orig_addr;
    unsigned int lifetime;
} RREP;

#define RREP_SIZE sizeof(RREP)

typedef struct {
    unsigned char type;
    unsigned char reserved;
} RREP_ack;

#define RREP_ACK_SIZE sizeof(RREP_ack)
#endif				/* NS_NO_GLOBALS */

#ifndef NS_NO_DECLARATIONS
RREP *rrep_create(unsigned char flags,
		  unsigned char prefix,
		  unsigned char hcnt,
		  struct in_addr dest_addr,
		  unsigned int dest_seqno,
		  struct in_addr orig_addr, unsigned int life);

RREP_ack *rrep_ack_create();
AODV_ext *rrep_add_ext(RREP * rrep, int type, unsigned int offset,
		       int len, char *data);
void rrep_send(RREP * rrep, rt_table_t * rev_rt, rt_table_t * fwd_rt, int size);
void rrep_forward(RREP * rrep, int size, rt_table_t * rev_rt,
		  rt_table_t * fwd_rt, int ttl);
void rrep_process(RREP * rrep, int rreplen, struct in_addr ip_src,
		  struct in_addr ip_dst, int ip_ttl, unsigned int ifindex);
void rrep_ack_process(RREP_ack * rrep_ack, int rreplen, struct in_addr ip_src,
		      struct in_addr ip_dst);
#endif				/* NS_NO_DECLARATIONS */

#endif				/* AODV_RREP_H */
