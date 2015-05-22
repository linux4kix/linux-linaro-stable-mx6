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


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_device.h"

#include <linux/iommu.h>
#include <linux/platform_device.h>

#define _GC_OBJ_ZONE gcvZONE_OS

typedef struct _gcsIOMMU
{
    struct iommu_domain * domain;
    struct device *       device;
}
gcsIOMMU;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
static int
_IOMMU_Fault_Handler(
    struct iommu_domain * Domain,
    struct device * Dev,
    unsigned long DomainAddress,
    int flags,
    void * args
    )
#else
static int
_IOMMU_Fault_Handler(
    struct iommu_domain * Domain,
    struct device * Dev,
    unsigned long DomainAddress,
    int flags
    )
#endif
{
    return 0;
}

static int
_FlatMapping(
    IN gckIOMMU Iommu
    )
{
    gceSTATUS status;
    gctUINT32 physical;

    for (physical = 0; physical < 0x80000000; physical += PAGE_SIZE)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "Map %x => %x bytes = %d",
            physical, physical, PAGE_SIZE
            );

        gcmkONERROR(gckIOMMU_Map(Iommu, physical, physical, PAGE_SIZE));
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

void
gckIOMMU_Destory(
    IN gckOS Os,
    IN gckIOMMU Iommu
    )
{
    gcmkHEADER();

    if (Iommu->domain && Iommu->device)
    {
        iommu_attach_device(Iommu->domain, Iommu->device);
    }

    if (Iommu->domain)
    {
        iommu_domain_free(Iommu->domain);
    }

    if (Iommu)
    {
        gcmkOS_SAFE_FREE(Os, Iommu);
    }

    gcmkFOOTER_NO();
}

gceSTATUS
gckIOMMU_Construct(
    IN gckOS Os,
    OUT gckIOMMU * Iommu
    )
{
    gceSTATUS status;
    gckIOMMU iommu = gcvNULL;
    struct device *dev;
    int ret;

    gcmkHEADER();

    dev = &Os->device->platform->device->dev;

    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcsIOMMU), (gctPOINTER *)&iommu));

    gckOS_ZeroMemory(iommu, gcmSIZEOF(gcsIOMMU));

    iommu->domain = iommu_domain_alloc(&platform_bus_type);

    if (!iommu->domain)
    {
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_OS, "iommu_domain_alloc() fail");

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    iommu_set_fault_handler(iommu->domain, _IOMMU_Fault_Handler, dev);
#else
    iommu_set_fault_handler(iommu->domain, _IOMMU_Fault_Handler);
#endif

    ret = iommu_attach_device(iommu->domain, dev);

    if (ret)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS, "iommu_attach_device() fail %d", ret);

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    iommu->device = dev;

    _FlatMapping(iommu);

    *Iommu = iommu;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    gckIOMMU_Destory(Os, iommu);

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckIOMMU_Map(
    IN gckIOMMU Iommu,
    IN gctUINT32 DomainAddress,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes
    )
{
    gceSTATUS status;
    int ret;

    gcmkHEADER_ARG("DomainAddress=%#X, Physical=%#X, Bytes=%d",
                   DomainAddress, Physical, Bytes);

    ret = iommu_map(Iommu->domain, DomainAddress, Physical, Bytes, 0);

    if (ret)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    gcmkFOOTER();
    return status;

}

gceSTATUS
gckIOMMU_Unmap(
    IN gckIOMMU Iommu,
    IN gctUINT32 DomainAddress,
    IN gctUINT32 Bytes
    )
{
    gcmkHEADER();

    iommu_unmap(Iommu->domain, DomainAddress, Bytes);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

