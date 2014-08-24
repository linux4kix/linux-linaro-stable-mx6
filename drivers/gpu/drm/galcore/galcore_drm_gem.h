/*
 * Copyright (C) 2012 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef GALCORE_DRM_GEM_H
#define GALCORE_DRM_GEM_H

/* GEM */
struct galcore_drm_gem_object {
	struct drm_gem_object	obj;
	void			*addr;
	phys_addr_t		phys_addr;
	resource_size_t		dev_addr;
	struct drm_mm_node	*linear;	/* for linear backed */
	struct page		*page;		/* for page backed */
	struct sg_table		*sgt;		/* for imported */
	void			(*update)(void *);
	void			*update_data;
};

extern const struct vm_operations_struct galcore_drm_gem_vm_ops;

#define drm_to_galcore_drm_gem(o) container_of(o, struct galcore_drm_gem_object, obj)

void galcore_drm_gem_free_object(struct drm_gem_object *);
int galcore_drm_gem_linear_back(struct drm_device *, struct galcore_drm_gem_object *);
void *galcore_drm_gem_map_object(struct drm_device *, struct galcore_drm_gem_object *);
struct galcore_drm_gem_object *galcore_drm_gem_alloc_private_object(struct drm_device *,
	size_t);
#if 0
int galcore_drm_gem_dumb_create(struct drm_file *, struct drm_device *,
	struct drm_mode_create_dumb *);
int galcore_drm_gem_dumb_map_offset(struct drm_file *, struct drm_device *,
	uint32_t, uint64_t *);
int galcore_drm_gem_dumb_destroy(struct drm_file *, struct drm_device *,
	uint32_t);
#endif
struct dma_buf *galcore_drm_gem_prime_export(struct drm_device *dev,
	struct drm_gem_object *obj, int flags);
struct drm_gem_object *galcore_drm_gem_prime_import(struct drm_device *,
	struct dma_buf *);
int galcore_drm_gem_map_import(struct galcore_drm_gem_object *);

static inline struct galcore_drm_gem_object *galcore_drm_gem_object_lookup(
	struct drm_device *dev, struct drm_file *dfile, unsigned handle)
{
	struct drm_gem_object *obj = drm_gem_object_lookup(dev, dfile, handle);

	return obj ? drm_to_galcore_drm_gem(obj) : NULL;
}
#endif
