#include "defs.h"

#ifndef NS_PORT
/* This will point to a struct containing information about the host */
struct host_info this_host;

/* Array of interface indexes */
unsigned int dev_indices[MAX_NR_INTERFACES];

/* Given a network interface index, return the index into the
   devs array, Necessary because ifindex is not always 0, 1,
   2... */
unsigned int ifindex2devindex(unsigned int ifindex)
{
    int i;

    for (i = 0; i < this_host.nif; i++)
	if (dev_indices[i] == ifindex)
	    return i;

    return MAX_NR_INTERFACES;
}

struct dev_info *devfromsock(int sock)
{
    int i;

    for (i = 0; i < this_host.nif; i++) {
	if (this_host.devs[i].sock == sock)
	    return &this_host.devs[i];
    }
    return NULL;
}

int name2index(char *name)
{
    int i;

    for (i = 0; i < this_host.nif; i++)
	if (strcmp(name, this_host.devs[i].ifname) == 0)
	    return this_host.devs[i].ifindex;

    return -1;
}

#endif