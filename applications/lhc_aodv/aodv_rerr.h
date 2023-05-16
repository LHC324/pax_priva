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
#ifndef _AODV_RERR_H
#define _AODV_RERR_H

#ifndef NS_NO_GLOBALS
#include "endian.h"

#include "defs.h"
#include "routing_table.h"

/* RERR Flags: */
#define RERR_NODELETE 0x1

typedef struct {
    unsigned char type;
#if defined(__LITTLE_ENDIAN)
    unsigned char res1:7;
    unsigned char n:1;
#elif defined(__BIG_ENDIAN)
    unsigned char n:1;
    unsigned char res1:7;
#else
#error "Adjust your <bits/endian.h> defines"
#endif
    unsigned char res2;
    unsigned char dest_count;
    unsigned int dest_addr;
    unsigned int dest_seqno;
} RERR;

#define RERR_SIZE sizeof(RERR)

/* Extra unreachable destinations... */
typedef struct {
    unsigned int dest_addr;
    unsigned int dest_seqno;
} RERR_udest;

#define RERR_UDEST_SIZE sizeof(RERR_udest)

/* Given the total number of unreachable destination this macro
   returns the RERR size */
#define RERR_CALC_SIZE(rerr) (RERR_SIZE + (rerr->dest_count-1)*RERR_UDEST_SIZE)
#define RERR_UDEST_FIRST(rerr) ((RERR_udest *)&rerr->dest_addr)
#define RERR_UDEST_NEXT(udest) ((RERR_udest *)((char *)udest + RERR_UDEST_SIZE))
#endif				/* NS_NO_GLOBALS */

#ifndef NS_NO_DECLARATIONS
RERR *rerr_create(unsigned char flags, struct in_addr dest_addr,
		  unsigned int dest_seqno);
void rerr_add_udest(RERR * rerr, struct in_addr udest, unsigned int udest_seqno);
void rerr_process(RERR * rerr, int rerrlen, struct in_addr ip_src,
		  struct in_addr ip_dst);
#endif				/* NS_NO_DECLARATIONS */

#endif				/* AODV_RERR_H */
