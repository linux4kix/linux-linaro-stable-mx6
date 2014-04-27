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

struct vivante_iommu_domain
{
	spinlock_t map_lock;
};

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

int vivante_iommu_init(void)
{
	bus_set_iommu(&platform_bus_type, &vivante_iommu_ops);
	return 0;
}
