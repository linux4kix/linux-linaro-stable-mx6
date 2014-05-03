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

#include "vivante_gpu.h"

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
			 struct device *dev, size_t size)
{
	pgtable->pgtable = dma_alloc_coherent(dev, size, &pgtable->handle, GFP_KERNEL);
	if (!pgtable->pgtable)
		return -ENOMEM;

	return 0;
}

static void pgtable_free(struct vivante_iommu_domain_pgtable *pgtable,
			 struct device *dev, size_t size)
{
	dma_free_coherent(dev, size, pgtable->pgtable, pgtable->handle);
}

static uint32_t pgtable_read(struct vivante_iommu_domain_pgtable *pgtable,
			     unsigned int index)
{
	return pgtable->pgtable[index];
}

static void pgtable_write(struct vivante_iommu_domain_pgtable *pgtable,
			  unsigned int index, unsigned int count, uint32_t val)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		pgtable->pgtable[index + i] = val;
}

static int vivante_iommu_domain_init(struct iommu_domain *domain)
{
	struct vivante_iommu_domain *vivante_domain;

	vivante_domain = kmalloc(sizeof(*vivante_domain), GFP_KERNEL);
	if (!vivante_domain)
		return -ENOMEM;

	domain->priv = vivante_domain;
	return 0;
}

static void vivante_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct vivante_iommu_domain *vivante_domain = domain->priv;

	kfree(vivante_domain);
	domain->priv = NULL;
}

static int vivante_iommu_map(struct iommu_domain *domain, unsigned long iova,
	   phys_addr_t paddr, size_t size, int prot)
{
	/*struct vivante_iommu_domain *vivante_domain = domain->priv;*/

	return 0;
}

static size_t vivante_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
	     size_t size)
{
	/*struct vivante_iommu_domain *vivante_domain = domain->priv;*/

	return 0;
}

phys_addr_t vivante_iommu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	/*struct vivante_iommu_domain *vivante_domain = domain->priv;*/

	return 0;
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
	int ret;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->ops = &vivante_iommu_ops;

	ret = domain->ops->domain_init(domain);
	if (ret)
		goto out_free;

	return domain;

out_free:
	kfree(domain);
	return NULL;
}
