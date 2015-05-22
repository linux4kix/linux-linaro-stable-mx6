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


#include "gc_hal_kernel_precomp.h"




#define _GC_OBJ_ZONE    gcvZONE_KERNEL

#if gcdSECURITY

/*
** Open a security service channel.
*/
gceSTATUS
gckKERNEL_SecurityOpen(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPU,
    OUT gctUINT32 *Channel
    )
{
    gceSTATUS status;

    gcmkONERROR(gckOS_OpenSecurityChannel(Kernel->os, Kernel->core, Channel));
    gcmkONERROR(gckOS_InitSecurityChannel(*Channel));

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*
** Close a security service channel
*/
gceSTATUS
gckKERNEL_SecurityClose(
    IN gctUINT32 Channel
    )
{
    return gcvSTATUS_OK;
}

/*
** Security service interface.
*/
gceSTATUS
gckKERNEL_SecurityCallService(
    IN gctUINT32 Channel,
    IN OUT gcsTA_INTERFACE * Interface
)
{
    gceSTATUS status;
    gcmkHEADER();

    gcmkVERIFY_ARGUMENT(Interface != gcvNULL);

    gckOS_CallSecurityService(Channel, Interface);

    status = Interface->result;

    gcmkONERROR(status);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SecurityStartCommand(
    IN gckKERNEL Kernel
    )
{
    gceSTATUS status;
    gcsTA_INTERFACE iface;

    gcmkHEADER();

    iface.command = KERNEL_START_COMMAND;
    iface.u.StartCommand.gpu = Kernel->core;

    gcmkONERROR(gckKERNEL_SecurityCallService(Kernel->securityChannel, &iface));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SecurityAllocateSecurityMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 Bytes,
    OUT gctUINT32 * Handle
    )
{
    gceSTATUS status;
    gcsTA_INTERFACE iface;

    gcmkHEADER();

    iface.command = KERNEL_ALLOCATE_SECRUE_MEMORY;
    iface.u.AllocateSecurityMemory.bytes = Bytes;

    gcmkONERROR(gckKERNEL_SecurityCallService(Kernel->securityChannel, &iface));

    *Handle = iface.u.AllocateSecurityMemory.memory_handle;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SecurityExecute(
    IN gckKERNEL Kernel,
    IN gctPOINTER Buffer,
    IN gctUINT32 Bytes
    )
{
    gceSTATUS status;
    gcsTA_INTERFACE iface;

    gcmkHEADER();

    iface.command = KERNEL_EXECUTE;
    iface.u.Execute.command_buffer = (gctUINT32 *)Buffer;
    iface.u.Execute.gpu = Kernel->core;
    iface.u.Execute.command_buffer_length = Bytes;

#if defined(LINUX)
    gcmkONERROR(gckOS_GetPhysicalAddress(Kernel->os, Buffer,
            (gctUINT32 *)&iface.u.Execute.command_buffer));
#endif

    gcmkONERROR(gckKERNEL_SecurityCallService(Kernel->securityChannel, &iface));

    /* Update queue tail pointer. */
    gcmkONERROR(gckHARDWARE_UpdateQueueTail(
        Kernel->hardware, 0, 0
        ));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SecurityMapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 *PhysicalArray,
    IN gctUINT32 PageCount,
    OUT gctUINT32 * GPUAddress
    )
{
    gceSTATUS status;
    gcsTA_INTERFACE iface;

    gcmkHEADER();

    iface.command = KERNEL_MAP_MEMORY;

#if defined(LINUX)
    gcmkONERROR(gckOS_GetPhysicalAddress(Kernel->os, PhysicalArray,
            (gctUINT32 *)&iface.u.MapMemory.physicals));
#endif

    iface.u.MapMemory.pageCount = PageCount;

    gcmkONERROR(gckKERNEL_SecurityCallService(Kernel->securityChannel, &iface));

    *GPUAddress = iface.u.MapMemory.gpuAddress;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_SecurityUnmapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    IN gctUINT32 PageCount
    )
{
    gceSTATUS status;
    gcsTA_INTERFACE iface;

    gcmkHEADER();

    iface.command = KERNEL_UNMAP_MEMORY;

    iface.u.UnmapMemory.gpuAddress = GPUAddress;
    iface.u.UnmapMemory.pageCount  = PageCount;

    gcmkONERROR(gckKERNEL_SecurityCallService(Kernel->securityChannel, &iface));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

#endif
