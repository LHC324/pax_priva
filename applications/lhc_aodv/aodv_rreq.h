/*****************************************************************************
 *
 * Copyright (C) 2001 Uppsala University and Ericsson AB.
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
#ifndef _AODV_RREQ_H
#define _AODV_RREQ_H

#ifndef NS_NO_GLOBALS
#include "endian.h"

#include "defs.h"
#include "seek_list.h"
#include "routing_table.h"

/* RREQ Flags: */
#define RREQ_JOIN          0x1
#define RREQ_REPAIR        0x2
#define RREQ_GRATUITOUS    0x4
#define RREQ_DEST_ONLY     0x8

typedef struct {
    unsigned char type;
#if defined(__LITTLE_ENDIAN)
    unsigned char res1:4;
    unsigned char d:1;
    unsigned char g:1;
    unsigned char r:1;
    unsigned char j:1;
#elif defined(__BIG_ENDIAN)
    unsigned char j:1;		/* Join flag (multicast) */
    unsigned char r:1;		/* Repair flag */
    unsigned char g:1;		/* Gratuitous RREP flag */
    unsigned char d:1;		/* Destination only respond */
    unsigned char res1:4;
#else
#error "Adjust your <bits/endian.h> defines"
#endif
    unsigned char res2;
    unsigned char hcnt;
    unsigned int rreq_id;
    unsigned int dest_addr;
    unsigned int dest_seqno;
    unsigned int orig_addr;
    unsigned int orig_seqno;
} RREQ;

#define RREQ_SIZE sizeof(RREQ)

/* A data structure to buffer information about received RREQ's */
struct rreq_record {
    list_t l;
    struct in_addr orig_addr;	/* Source of the RREQ */
    unsigned int rreq_id;		/* RREQ's broadcast ID */
    struct timer rec_timer;
};

struct blacklist {
    list_t l;
    struct in_addr dest_addr;
    struct timer bl_timer;
};
#endif				/* NS_NO_GLOBALS */

#ifndef NS_NO_DECLARATIONS
RREQ *rreq_create(unsigned char flags, struct in_addr dest_addr,
		  unsigned int dest_seqno, struct in_addr orig_addr);
void rreq_send(struct in_addr dest_addr, unsigned int dest_seqno, int ttl,
	       unsigned char flags);
void rreq_forward(RREQ * rreq, int size, int ttl);
void rreq_process(RREQ * rreq, int rreqlen, struct in_addr ip_src,
		  struct in_addr ip_dst, int ip_ttl, unsigned int ifindex);
void rreq_route_discovery(struct in_addr dest_addr, unsigned char flags,
			  struct ip_data *ipd);
void rreq_record_timeout(void *arg);
struct blacklist *rreq_blacklist_insert(struct in_addr dest_addr);
void rreq_blacklist_timeout(void *arg);
void rreq_local_repair(rt_table_t * rt, struct in_addr src_addr,
		       struct ip_data *ipd);

#ifdef NS_PORT
struct rreq_record *rreq_record_insert(struct in_addr orig_addr,
				       unsigned int rreq_id);
struct rreq_record *rreq_record_find(struct in_addr orig_addr,
				     unsigned int rreq_id);
struct blacklist *rreq_blacklist_find(struct in_addr dest_addr);
#endif				/* NS_PORT */

#endif				/* NS_NO_DECLARATIONS */

#endif				/* AODV_RREQ_H */
