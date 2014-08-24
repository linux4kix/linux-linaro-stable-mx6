/*
 * Copyright (C) 2012 Russell King
 *  With inspiration from the i915 driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef DRM_GALCORE_IOCTL_H
#define DRM_GALCORE_IOCTL_H

#define DRM_GALCORE_GEM_CREATE		0x00
#define DRM_GALCORE_GEM_MMAP		0x02
#define DRM_GALCORE_GEM_PWRITE		0x03

#define GALCORE_DRM_IOCTL(dir, name, str) \
	DRM_##dir(DRM_COMMAND_BASE + DRM_GALCORE_##name, struct drm_galcore_##str)

struct drm_galcore_gem_create {
	uint32_t handle;
	uint32_t size;
};
#define DRM_IOCTL_GALCORE_GEM_CREATE \
	GALCORE_DRM_IOCTL(IOWR, GEM_CREATE, gem_create)

struct drm_galcore_gem_mmap {
	uint32_t handle;
	uint32_t pad;
	uint64_t offset;
	uint64_t size;
	uint64_t addr;
};
#define DRM_IOCTL_GALCORE_GEM_MMAP \
	GALCORE_DRM_IOCTL(IOWR, GEM_MMAP, gem_mmap)

struct drm_galcore_gem_pwrite {
	uint64_t ptr;
	uint32_t handle;
	uint32_t offset;
	uint32_t size;
};
#define DRM_IOCTL_GALCORE_GEM_PWRITE \
	GALCORE_DRM_IOCTL(IOW, GEM_PWRITE, gem_pwrite)

#endif
