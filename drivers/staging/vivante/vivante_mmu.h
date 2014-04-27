/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
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

#ifndef __VIVANTE_MMU_H__
#define __VIVANTE_MMU_H__

#include <linux/iommu.h>

struct vivante_iommu {
	struct drm_device *dev;
	struct iommu_domain *domain;
};

int vivante_iommu_attach(struct vivante_iommu *iommu, const char **names, int cnt);
int vivante_iommu_map(struct vivante_iommu *iommu, uint32_t iova, struct sg_table *sgt,
		unsigned len, int prot);
int vivante_iommu_unmap(struct vivante_iommu *iommu, uint32_t iova, struct sg_table *sgt,
		unsigned len);
void vivante_iommu_destroy(struct vivante_iommu *iommu);

struct vivante_iommu *vivante_iommu_new(struct drm_device *dev, struct iommu_domain *domain);

#endif /* __VIVANTE_MMU_H__ */
