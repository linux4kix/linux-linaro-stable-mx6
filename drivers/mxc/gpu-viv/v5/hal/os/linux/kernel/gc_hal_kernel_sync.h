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


#ifndef __gc_hal_kernel_sync_h_
#define __gc_hal_kernel_sync_h_

#include <linux/types.h>

/* sync.h is in drivers/staging/android/ for now. */
#include <sync.h>

#include <gc_hal.h>
#include <gc_hal_base.h>

struct viv_sync_timeline
{
    /* Parent object. */
    struct sync_timeline obj;

    /* Timestamp when sync_pt is created. */
    gctUINT stamp;

    /* Pointer to os struct. */
    gckOS os;
};


struct viv_sync_pt
{
    /* Parent object. */
    struct sync_pt pt;

    /* Reference sync point*/
    gctSYNC_POINT sync;

    /* Timestamp when sync_pt is created. */
    gctUINT stamp;
};

/* Create viv_sync_timeline object. */
struct viv_sync_timeline *
viv_sync_timeline_create(
    const char * Name,
    gckOS Os
    );

/* Create viv_sync_pt object. */
struct sync_pt *
viv_sync_pt_create(
    struct viv_sync_timeline * Obj,
    gctSYNC_POINT SyncPoint
    );

#endif /* __gc_hal_kernel_sync_h_ */
