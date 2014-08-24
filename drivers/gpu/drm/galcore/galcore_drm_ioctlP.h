/*
 * Copyright (C) 2012 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef GALCORE_DRM_IOCTLP_H
#define GALCORE_DRM_IOCTLP_H

#define GALCORE_DRM_IOCTL_PROTO(name)\
extern int galcore_drm_##name##_ioctl(struct drm_device *, void *, struct drm_file *)

GALCORE_DRM_IOCTL_PROTO(gem_create);
GALCORE_DRM_IOCTL_PROTO(gem_mmap);
GALCORE_DRM_IOCTL_PROTO(gem_pwrite);

#endif
