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


#ifndef __gc_hal_kernel_context_h_
#define __gc_hal_kernel_context_h_

#include "gc_hal_kernel_buffer.h"

/* Exprimental optimization. */
#define REMOVE_DUPLICATED_COPY_FROM_USER 1

#ifdef __cplusplus
extern "C" {
#endif

/* Maps state locations within the context buffer. */
typedef struct _gcsSTATE_MAP * gcsSTATE_MAP_PTR;
typedef struct _gcsSTATE_MAP
{
    /* Index of the state in the context buffer. */
    gctUINT                     index;

    /* State mask. */
    gctUINT32                   mask;
}
gcsSTATE_MAP;

/* Context buffer. */
typedef struct _gcsCONTEXT * gcsCONTEXT_PTR;
typedef struct _gcsCONTEXT
{
    /* For debugging: the number of context buffer in the order of creation. */
#if gcmIS_DEBUG(gcdDEBUG_CODE)
    gctUINT                     num;
#endif

    /* Pointer to gckEVENT object. */
    gckEVENT                    eventObj;

    /* Context busy signal. */
    gctSIGNAL                   signal;

    /* Physical address of the context buffer. */
    gctPHYS_ADDR                physical;

    /* Logical address of the context buffer. */
    gctUINT32_PTR               logical;

    /* Hardware address of the context buffer. */
    gctUINT32                   address;

    /* Pointer to the LINK commands. */
    gctPOINTER                  link2D;
    gctPOINTER                  link3D;

    /* The number of pending state deltas. */
    gctUINT                     deltaCount;

    /* Pointer to the first delta to be applied. */
    gcsSTATE_DELTA_PTR          delta;

    /* Next context buffer. */
    gcsCONTEXT_PTR              next;
}
gcsCONTEXT;

typedef struct _gcsRECORD_ARRAY_MAP * gcsRECORD_ARRAY_MAP_PTR;
struct  _gcsRECORD_ARRAY_MAP
{
    /* User pointer key. */
    gctUINT64                   key;

    /* Kernel memory buffer. */
    gcsSTATE_DELTA_RECORD_PTR   kData;

    /* Next map. */
    gcsRECORD_ARRAY_MAP_PTR     next;

};

/* gckCONTEXT structure that hold the current context. */
struct _gckCONTEXT
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckHARDWARE object. */
    gckHARDWARE                 hardware;

    /* Command buffer alignment. */
    gctUINT32                   alignment;
    gctUINT32                   reservedHead;
    gctUINT32                   reservedTail;

    /* Context buffer metrics. */
    gctSIZE_T                   stateCount;
    gctUINT32                   totalSize;
    gctUINT32                   bufferSize;
    gctUINT32                   linkIndex2D;
    gctUINT32                   linkIndex3D;
    gctUINT32                   linkIndexXD;
    gctUINT32                   entryOffset3D;
    gctUINT32                   entryOffsetXDFrom2D;
    gctUINT32                   entryOffsetXDFrom3D;

    /* Dirty flags. */
    gctBOOL                     dirty;
    gctBOOL                     dirty2D;
    gctBOOL                     dirty3D;
    gcsCONTEXT_PTR              dirtyBuffer;

    /* State mapping. */
    gcsSTATE_MAP_PTR            map;

    /* List of context buffers. */
    gcsCONTEXT_PTR              buffer;

    /* A copy of the user record array. */
    gctUINT                     recordArraySize;
#if REMOVE_DUPLICATED_COPY_FROM_USER
    gcsRECORD_ARRAY_MAP_PTR     recordArrayMap;
#else
    gcsSTATE_DELTA_RECORD_PTR   recordArray;
#endif

    /* Requested pipe select for context. */
    gcePIPE_SELECT              entryPipe;
    gcePIPE_SELECT              exitPipe;

    /* Variables used for building state buffer. */
    gctUINT32                   lastAddress;
    gctSIZE_T                   lastSize;
    gctUINT32                   lastIndex;
    gctBOOL                     lastFixed;

    gctUINT32                   pipeSelectBytes;

    /* Hint array. */
#if gcdSECURE_USER
    gctBOOL_PTR                 hint;
#endif

#if VIVANTE_PROFILER_CONTEXT
    gcsPROFILER_COUNTERS        latestProfiler;
    gcsPROFILER_COUNTERS        histroyProfiler;
    gctUINT32                   prevVSInstCount;
    gctUINT32                   prevVSBranchInstCount;
    gctUINT32                   prevVSTexInstCount;
    gctUINT32                   prevVSVertexCount;
    gctUINT32                   prevPSInstCount;
    gctUINT32                   prevPSBranchInstCount;
    gctUINT32                   prevPSTexInstCount;
    gctUINT32                   prevPSPixelCount;
#endif
};

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_kernel_context_h_ */

