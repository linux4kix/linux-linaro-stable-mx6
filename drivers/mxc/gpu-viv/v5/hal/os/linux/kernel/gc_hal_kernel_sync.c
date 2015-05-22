/****************************************************************************
*
*    Copyright (C) 2005 - 2014 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/


#include <gc_hal.h>
#include <gc_hal_base.h>

#if gcdANDROID_NATIVE_FENCE_SYNC

#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include "gc_hal_kernel_sync.h"

static struct sync_pt *
viv_sync_pt_dup(
    struct sync_pt * sync_pt
    )
{
    gceSTATUS status;
    struct viv_sync_pt *pt;
    struct viv_sync_pt *src;
    struct viv_sync_timeline *obj;

    src = (struct viv_sync_pt *) sync_pt;
    obj = (struct viv_sync_timeline *) sync_pt->parent;

    /* Create the new sync_pt. */
    pt = (struct viv_sync_pt *)
        sync_pt_create(&obj->obj, sizeof(struct viv_sync_pt));

    pt->stamp = src->stamp;
    pt->sync = src->sync;

    /* Reference sync point. */
    status = gckOS_ReferenceSyncPoint(obj->os, pt->sync);

    if (gcmIS_ERROR(status))
    {
        sync_pt_free((struct sync_pt *)pt);
        return NULL;
    }

    return (struct sync_pt *)pt;
}

static int
viv_sync_pt_has_signaled(
    struct sync_pt * sync_pt
    )
{
    gceSTATUS status;
    gctBOOL state;
    struct viv_sync_pt * pt;
    struct viv_sync_timeline * obj;

    pt  = (struct viv_sync_pt *)sync_pt;
    obj = (struct viv_sync_timeline *)sync_pt->parent;

    status = gckOS_QuerySyncPoint(obj->os, pt->sync, &state);

    if (gcmIS_ERROR(status))
    {
        /* Error. */
        return -1;
    }

    return state;
}

static int
viv_sync_pt_compare(
    struct sync_pt * a,
    struct sync_pt * b
    )
{
    int ret;
    struct viv_sync_pt * pt1 = (struct viv_sync_pt *) a;
    struct viv_sync_pt * pt2 = (struct viv_sync_pt *) b;

    ret = (pt1->stamp <  pt2->stamp) ? -1
        : (pt1->stamp == pt2->stamp) ?  0
        : 1;

    return ret;
}

static void
viv_sync_pt_free(
    struct sync_pt * sync_pt
    )
{
    struct viv_sync_pt * pt;
    struct viv_sync_timeline * obj;

    pt  = (struct viv_sync_pt *) sync_pt;
    obj = (struct viv_sync_timeline *) sync_pt->parent;

    gckOS_DestroySyncPoint(obj->os, pt->sync);
}

static struct sync_timeline_ops viv_timeline_ops =
{
    .driver_name = "viv_sync",
    .dup = viv_sync_pt_dup,
    .has_signaled = viv_sync_pt_has_signaled,
    .compare = viv_sync_pt_compare,
    .free_pt = viv_sync_pt_free,
};

struct viv_sync_timeline *
viv_sync_timeline_create(
    const char * name,
    gckOS os
    )
{
    struct viv_sync_timeline * obj;

    obj = (struct viv_sync_timeline *)
        sync_timeline_create(&viv_timeline_ops, sizeof(struct viv_sync_timeline), name);

    obj->os    = os;
    obj->stamp = 0;

    return obj;
}

struct sync_pt *
viv_sync_pt_create(
    struct viv_sync_timeline * obj,
    gctSYNC_POINT SyncPoint
    )
{
    gceSTATUS status;
    struct viv_sync_pt * pt;

    pt = (struct viv_sync_pt *)
        sync_pt_create(&obj->obj, sizeof(struct viv_sync_pt));

    pt->stamp = obj->stamp++;
    pt->sync  = SyncPoint;

    /* Dup signal. */
    status = gckOS_ReferenceSyncPoint(obj->os, SyncPoint);

    if (gcmIS_ERROR(status))
    {
        sync_pt_free((struct sync_pt *)pt);
        return NULL;
    }

    return (struct sync_pt *) pt;
}

#endif
