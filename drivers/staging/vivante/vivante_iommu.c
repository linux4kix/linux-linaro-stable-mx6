/*
 * Copyright (C) 2014 Christian Gmeiner <christian.gmeiner@gmail.com>
  *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>

#include "vivante_gpu.h"
#include "state_hi.xml.h"

#define PT_SIZE 	SZ_256K
#define PT_ENTRIES	(PT_SIZE / sizeof(uint32_t))

#define GPU_MEM_START	0x80000000

struct vivante_iommu_domain_pgtable {
	uint32_t *pgtable;
	dma_addr_t handle;
};

struct vivante_iommu_domain
{
	struct vivante_iommu_domain_pgtable pgtable;
	spinlock_t map_lock;
};

static int pgtable_alloc(struct vivante_iommu_domain_pgtable *pgtable,
			 size_t size)
{
	pgtable->pgtable = dma_alloc_coherent(NULL, size, &pgtable->handle, GFP_KERNEL);
	if (!pgtable->pgtable)
		return -ENOMEM;

	return 0;
}

static void pgtable_free(struct vivante_iommu_domain_pgtable *pgtable,
			 size_t size)
{
	dma_free_coherent(NULL, size, pgtable->pgtable, pgtable->handle);
}

static uint32_t pgtable_read(struct vivante_iommu_domain_pgtable *pgtable,
			   unsigned long iova)
{
	/* calcuate index into page table */
	unsigned int index = (iova - GPU_MEM_START) / SZ_4K;
	phys_addr_t paddr;

	paddr = pgtable->pgtable[index];

	return paddr;
}

static void pgtable_write(struct vivante_iommu_domain_pgtable *pgtable,
			  unsigned long iova, phys_addr_t paddr)
{
	/* calcuate index into page table */
	unsigned int index = (iova - GPU_MEM_START) / SZ_4K;

	pgtable->pgtable[index] = paddr;
}

static int vivante_iommu_domain_init(struct iommu_domain *domain)
{
	struct vivante_iommu_domain *vivante_domain;
	int ret;

	vivante_domain = kmalloc(sizeof(*vivante_domain), GFP_KERNEL);
	if (!vivante_domain)
		return -ENOMEM;

	ret = pgtable_alloc(&vivante_domain->pgtable, PT_SIZE);
	if (ret < 0) {
		kfree(vivante_domain);
		return ret;
	}

	domain->priv = vivante_domain;
	return 0;
}

static void vivante_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct vivante_iommu_domain *vivante_domain = domain->priv;

	pgtable_free(&vivante_domain->pgtable, PT_SIZE);

	kfree(vivante_domain);
	domain->priv = NULL;
}

static int vivante_iommu_map(struct iommu_domain *domain, unsigned long iova,
	   phys_addr_t paddr, size_t size, int prot)
{
	struct vivante_iommu_domain *vivante_domain = domain->priv;

	if (size != SZ_4K)
		return -EINVAL;

	spin_lock(&vivante_domain->map_lock);
	pgtable_write(&vivante_domain->pgtable, iova, paddr);
	spin_unlock(&vivante_domain->map_lock);

	return 0;
}

static size_t vivante_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
	     size_t size)
{
	struct vivante_iommu_domain *vivante_domain = domain->priv;

	if (size != SZ_4K)
		return -EINVAL;

	spin_lock(&vivante_domain->map_lock);
	pgtable_write(&vivante_domain->pgtable, iova, ~0);
	spin_unlock(&vivante_domain->map_lock);

	return 0;
}

phys_addr_t vivante_iommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	struct vivante_iommu_domain *vivante_domain = domain->priv;

	return pgtable_read(&vivante_domain->pgtable, iova);
}

static struct iommu_ops vivante_iommu_ops = {
		.domain_init = vivante_iommu_domain_init,
		.domain_destroy = vivante_iommu_domain_destroy,
		.map = vivante_iommu_map,
		.unmap = vivante_iommu_unmap,
		.iova_to_phys = vivante_iommu_iova_to_phys,
		.pgsize_bitmap = SZ_4K,
};

struct iommu_domain *vivante_iommu_domain_alloc(struct msm_gpu *gpu)
{
	struct iommu_domain *domain;
	struct vivante_iommu_domain *vivante_domain;
	int ret;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->ops = &vivante_iommu_ops;

	ret = domain->ops->domain_init(domain);
	if (ret)
		goto out_free;

	/* set page table address in MC */
	vivante_domain = domain->priv;

	gpu_write(gpu, VIVS_MC_MMU_FE_PAGE_TABLE, (uint32_t)vivante_domain->pgtable.pgtable);
	gpu_write(gpu, VIVS_MC_MMU_TX_PAGE_TABLE, (uint32_t)vivante_domain->pgtable.pgtable);
	gpu_write(gpu, VIVS_MC_MMU_PE_PAGE_TABLE, (uint32_t)vivante_domain->pgtable.pgtable);
	gpu_write(gpu, VIVS_MC_MMU_PEZ_PAGE_TABLE, (uint32_t)vivante_domain->pgtable.pgtable);
	gpu_write(gpu, VIVS_MC_MMU_RA_PAGE_TABLE, (uint32_t)vivante_domain->pgtable.pgtable);

	return domain;

out_free:
	kfree(domain);
	return NULL;
}
