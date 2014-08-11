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

#include <linux/spinlock.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>

#include "vivante_drv.h"
#include "vivante_gem.h"
#include "vivante_gpu.h"
#include "vivante_mmu.h"

/* called with dev->struct_mutex held */
static struct page **get_pages(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	if (!vivante_obj->pages) {
		struct drm_device *dev = obj->dev;
		struct page **p;
		int npages = obj->size >> PAGE_SHIFT;

		p = drm_gem_get_pages(obj, 0);

		if (IS_ERR(p)) {
			dev_err(dev->dev, "could not get pages: %ld\n",
					PTR_ERR(p));
			return p;
		}

		vivante_obj->sgt = drm_prime_pages_to_sg(p, npages);
		if (IS_ERR(vivante_obj->sgt)) {
			dev_err(dev->dev, "failed to allocate sgt\n");
			return ERR_CAST(vivante_obj->sgt);
		}

		vivante_obj->pages = p;

		/* For non-cached buffers, ensure the new pages are clean
		 * because display controller, GPU, etc. are not coherent:
		 */
		if (vivante_obj->flags & (MSM_BO_WC|MSM_BO_UNCACHED))
			dma_map_sg(dev->dev, vivante_obj->sgt->sgl,
					vivante_obj->sgt->nents, DMA_BIDIRECTIONAL);
	}

	return vivante_obj->pages;
}

static void put_pages(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	if (vivante_obj->pages) {
		/* For non-cached buffers, ensure the new pages are clean
		 * because display controller, GPU, etc. are not coherent:
		 */
		if (vivante_obj->flags & (MSM_BO_WC|MSM_BO_UNCACHED))
			dma_unmap_sg(obj->dev->dev, vivante_obj->sgt->sgl,
					vivante_obj->sgt->nents, DMA_BIDIRECTIONAL);
		sg_free_table(vivante_obj->sgt);
		kfree(vivante_obj->sgt);

		drm_gem_put_pages(obj, vivante_obj->pages, true, false);

		vivante_obj->pages = NULL;
	}
}

struct page **vivante_gem_get_pages(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct page **p;
	mutex_lock(&dev->struct_mutex);
	p = get_pages(obj);
	mutex_unlock(&dev->struct_mutex);
	return p;
}

void msm_gem_put_pages(struct drm_gem_object *obj)
{
	/* when we start tracking the pin count, then do something here */
}

static int vivante_gem_mmap_cmd(struct drm_gem_object *obj,
	struct vm_area_struct *vma)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	int ret;

	/*
	 * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
	 * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
	 * the whole buffer.
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;

	ret = dma_mmap_coherent(obj->dev->dev, vma,
				vivante_obj->vaddr, vivante_obj->paddr,
				vma->vm_end - vma->vm_start);

	return ret;
}

static int vivante_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	if (vivante_obj->flags & MSM_BO_WC) {
		vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	} else if (vivante_obj->flags & MSM_BO_UNCACHED) {
		vma->vm_page_prot = pgprot_noncached(vm_get_page_prot(vma->vm_flags));
	} else {
		/*
		 * Shunt off cached objs to shmem file so they have their own
		 * address_space (so unmap_mapping_range does what we want,
		 * in particular in the case of mmap'd dmabufs)
		 */
		fput(vma->vm_file);
		get_file(obj->filp);
		vma->vm_pgoff = 0;
		vma->vm_file  = obj->filp;

		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	}

	return 0;
}

int vivante_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct vivante_gem_object *obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret) {
		DBG("mmap failed: %d", ret);
		return ret;
	}

	obj = to_vivante_bo(vma->vm_private_data);
	if (obj->flags & ETNA_BO_CMDSTREAM)
		ret = vivante_gem_mmap_cmd(vma->vm_private_data, vma);
	else
		ret = vivante_gem_mmap_obj(vma->vm_private_data, vma);

	return ret;
}

int msm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct drm_device *dev = obj->dev;
	struct page **pages;
	unsigned long pfn;
	pgoff_t pgoff;
	int ret;

	/* Make sure we don't parallel update on a fault, nor move or remove
	 * something from beneath our feet
	 */
	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		goto out;

	/* make sure we have pages attached now */
	pages = get_pages(obj);
	if (IS_ERR(pages)) {
		ret = PTR_ERR(pages);
		goto out_unlock;
	}

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;

	pfn = page_to_pfn(pages[pgoff]);

	VERB("Inserting %p pfn %lx, pa %lx", vmf->virtual_address,
			pfn, pfn << PAGE_SHIFT);

	ret = vm_insert_mixed(vma, (unsigned long)vmf->virtual_address, pfn);

out_unlock:
	mutex_unlock(&dev->struct_mutex);
out:
	switch (ret) {
	case -EAGAIN:
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		/*
		 * EBUSY is ok: this just means that another thread
		 * already did the job.
		 */
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

/** get mmap offset */
static uint64_t mmap_offset(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	int ret;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	/* Make it mmapable */
	ret = drm_gem_create_mmap_offset(obj);

	if (ret) {
		dev_err(dev->dev, "could not allocate mmap offset\n");
		return 0;
	}

	return drm_vma_node_offset_addr(&obj->vma_node);
}

uint64_t msm_gem_mmap_offset(struct drm_gem_object *obj)
{
	uint64_t offset;
	mutex_lock(&obj->dev->struct_mutex);
	offset = mmap_offset(obj);
	mutex_unlock(&obj->dev->struct_mutex);
	return offset;
}

/* should be called under struct_mutex.. although it can be called
 * from atomic context without struct_mutex to acquire an extra
 * iova ref if you know one is already held.
 *
 * That means when I do eventually need to add support for unpinning
 * the refcnt counter needs to be atomic_t.
 */
int vivante_gem_get_iova_locked(struct vivante_gpu * gpu, struct drm_gem_object *obj,
		uint32_t *iova)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	int ret = 0;

	if (!vivante_obj->iova) {
		struct vivante_drm_private *priv = obj->dev->dev_private;
		struct vivante_iommu *mmu = priv->mmu;
		struct page **pages = get_pages(obj);
		uint32_t offset;
		struct drm_mm_node *node = NULL;

		if (IS_ERR(pages))
			return PTR_ERR(pages);

		node = kzalloc(sizeof(*node), GFP_KERNEL);
		if (!node)
			return -ENOMEM;

		ret = drm_mm_insert_node(&gpu->mm, node, obj->size, 0,
				DRM_MM_SEARCH_DEFAULT);

		if (!ret) {
			offset = node->start;
			vivante_obj->iova = offset;
			vivante_obj->gpu_vram_node = node;

			ret = vivante_iommu_map(mmu, offset, vivante_obj->sgt,
					obj->size, IOMMU_READ | IOMMU_WRITE);
		} else
			kfree(node);
	}

	if (!ret)
		*iova = vivante_obj->iova;

	return ret;
}

int vivante_gem_get_iova(struct vivante_gpu *gpu, struct drm_gem_object *obj, int id, uint32_t *iova)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	int ret;

	/* this is safe right now because we don't unmap until the
	 * bo is deleted:
	 */
	if (vivante_obj->iova) {
		*iova = vivante_obj->iova;
		return 0;
	}

	mutex_lock(&obj->dev->struct_mutex);
	ret = vivante_gem_get_iova_locked(gpu, obj, iova);
	mutex_unlock(&obj->dev->struct_mutex);
	return ret;
}

void vivante_gem_put_iova(struct drm_gem_object *obj)
{
	// XXX TODO ..
	// NOTE: probably don't need a _locked() version.. we wouldn't
	// normally unmap here, but instead just mark that it could be
	// unmapped (if the iova refcnt drops to zero), but then later
	// if another _get_iova_locked() fails we can start unmapping
	// things that are no longer needed..
}

int msm_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args)
{
	args->pitch = align_pitch(args->width, args->bpp);
	args->size  = PAGE_ALIGN(args->pitch * args->height);
	/* TODO: re-check flags */
	return vivante_gem_new_handle(dev, file, args->size,
			MSM_BO_WC, &args->handle);
}

int msm_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	/* GEM does all our handle to object mapping */
	obj = drm_gem_object_lookup(dev, file, handle);
	if (obj == NULL) {
		ret = -ENOENT;
		goto fail;
	}

	*offset = msm_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

fail:
	return ret;
}

void *vivante_gem_vaddr_locked(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));
	if (!vivante_obj->vaddr) {
		struct page **pages = get_pages(obj);
		if (IS_ERR(pages))
			return ERR_CAST(pages);
		vivante_obj->vaddr = vmap(pages, obj->size >> PAGE_SHIFT,
				VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	}
	return vivante_obj->vaddr;
}

void *msm_gem_vaddr(struct drm_gem_object *obj)
{
	void *ret;
	mutex_lock(&obj->dev->struct_mutex);
	ret = vivante_gem_vaddr_locked(obj);
	mutex_unlock(&obj->dev->struct_mutex);
	return ret;
}

dma_addr_t vivante_gem_paddr_locked(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));

	return vivante_obj->paddr;
}

void vivante_gem_move_to_active(struct drm_gem_object *obj,
		struct vivante_gpu *gpu, bool write, uint32_t fence)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	vivante_obj->gpu = gpu;
	if (write)
		vivante_obj->write_fence = fence;
	else
		vivante_obj->read_fence = fence;
	list_del_init(&vivante_obj->mm_list);
	list_add_tail(&vivante_obj->mm_list, &gpu->active_list);
}

void vivante_gem_move_to_inactive(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct vivante_drm_private *priv = dev->dev_private;
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	vivante_obj->gpu = NULL;
	vivante_obj->read_fence = 0;
	vivante_obj->write_fence = 0;
	list_del_init(&vivante_obj->mm_list);
	list_add_tail(&vivante_obj->mm_list, &priv->inactive_list);
}

int vivante_gem_cpu_prep(struct drm_gem_object *obj, uint32_t op,
		struct timespec *timeout)
{
/*
	struct drm_device *dev = obj->dev;
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
*/
	int ret = 0;
	/* TODO */
#if 0
	if (is_active(vivante_obj)) {
		uint32_t fence = 0;

		if (op & MSM_PREP_READ)
			fence = vivante_obj->write_fence;
		if (op & MSM_PREP_WRITE)
			fence = max(fence, vivante_obj->read_fence);
		if (op & MSM_PREP_NOSYNC)
			timeout = NULL;

		ret = vivante_wait_fence_interruptable(dev, fence, timeout);
	}

	/* TODO cache maintenance */
#endif
	return ret;
}

int vivante_gem_cpu_fini(struct drm_gem_object *obj)
{
	/* TODO cache maintenance */
	return 0;
}

#ifdef CONFIG_DEBUG_FS
void msm_gem_describe(struct drm_gem_object *obj, struct seq_file *m)
{
	struct drm_device *dev = obj->dev;
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	uint64_t off = drm_vma_node_start(&obj->vma_node);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));
	seq_printf(m, "%08x: %c(r=%u,w=%u) %2d (%2d) %08llx %p %d\n",
			vivante_obj->flags, is_active(vivante_obj) ? 'A' : 'I',
			vivante_obj->read_fence, vivante_obj->write_fence,
			obj->name, obj->refcount.refcount.counter,
			off, vivante_obj->vaddr, obj->size);
}

void msm_gem_describe_objects(struct list_head *list, struct seq_file *m)
{
	struct vivante_gem_object *vivante_obj;
	int count = 0;
	size_t size = 0;

	list_for_each_entry(vivante_obj, list, mm_list) {
		struct drm_gem_object *obj = &vivante_obj->base;
		seq_printf(m, "   ");
		msm_gem_describe(obj, m);
		count++;
		size += obj->size;
	}

	seq_printf(m, "Total %d objects, %zu bytes\n", count, size);
}
#endif

static void vivante_free_cmd(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	drm_gem_free_mmap_offset(obj);

	dma_free_coherent(obj->dev->dev, obj->size,
		vivante_obj->vaddr, vivante_obj->paddr);

	drm_gem_object_release(obj);
}

static void vivante_free_obj(struct drm_gem_object *obj)
{
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);
	struct vivante_drm_private *priv = obj->dev->dev_private;
	struct vivante_iommu *mmu = priv->mmu;

	if (mmu && vivante_obj->iova) {
		uint32_t offset = vivante_obj->gpu_vram_node->start;
		vivante_iommu_unmap(mmu, offset, vivante_obj->sgt, obj->size);
		drm_mm_remove_node(vivante_obj->gpu_vram_node);
		kfree(vivante_obj->gpu_vram_node);
	}

	drm_gem_free_mmap_offset(obj);

	if (obj->import_attach) {
		if (vivante_obj->vaddr)
			dma_buf_vunmap(obj->import_attach->dmabuf, vivante_obj->vaddr);

		/* Don't drop the pages for imported dmabuf, as they are not
		 * ours, just free the array we allocated:
		 */
		if (vivante_obj->pages)
			drm_free_large(vivante_obj->pages);

	} else {
		if (vivante_obj->vaddr)
			vunmap(vivante_obj->vaddr);
		put_pages(obj);
	}

	if (vivante_obj->resv == &vivante_obj->_resv)
		reservation_object_fini(vivante_obj->resv);

	drm_gem_object_release(obj);
}

void vivante_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct vivante_gem_object *vivante_obj = to_vivante_bo(obj);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	/* object should not be on active list: */
	WARN_ON(is_active(vivante_obj));

	list_del(&vivante_obj->mm_list);

	if (vivante_obj->flags & ETNA_BO_CMDSTREAM)
		vivante_free_cmd(obj);
	else
		vivante_free_obj(obj);

	kfree(vivante_obj);
}

/* convenience method to construct a GEM buffer object, and userspace handle */
int vivante_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		uint32_t size, uint32_t flags, uint32_t *handle)
{
	struct drm_gem_object *obj;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	obj = vivante_gem_new(dev, size, flags);

	mutex_unlock(&dev->struct_mutex);

	if (IS_ERR(obj))
		return PTR_ERR(obj);

	ret = drm_gem_handle_create(file, obj, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int vivante_gem_new_impl(struct drm_device *dev,
		uint32_t size, uint32_t flags,
		struct drm_gem_object **obj)
{
	struct vivante_drm_private *priv = dev->dev_private;
	struct vivante_gem_object *vivante_obj;
	unsigned sz;

	switch (flags & MSM_BO_CACHE_MASK) {
	case MSM_BO_UNCACHED:
	case MSM_BO_CACHED:
	case MSM_BO_WC:
		break;
	default:
		dev_err(dev->dev, "invalid cache flag: %x\n",
				(flags & MSM_BO_CACHE_MASK));
		return -EINVAL;
	}

	sz = sizeof(*vivante_obj);

	vivante_obj = kzalloc(sz, GFP_KERNEL);
	if (!vivante_obj)
		return -ENOMEM;

	if (flags & ETNA_BO_CMDSTREAM) {
		vivante_obj->vaddr = dma_alloc_coherent(dev->dev, size,
				&vivante_obj->paddr, GFP_KERNEL);

		if (!vivante_obj->vaddr) {
			kfree(vivante_obj);
			return -ENOMEM;
		}
	}

	vivante_obj->flags = flags;

	vivante_obj->resv = &vivante_obj->_resv;
	reservation_object_init(vivante_obj->resv);

	INIT_LIST_HEAD(&vivante_obj->submit_entry);
	list_add_tail(&vivante_obj->mm_list, &priv->inactive_list);

	*obj = &vivante_obj->base;

	return 0;
}

struct drm_gem_object *vivante_gem_new(struct drm_device *dev,
		uint32_t size, uint32_t flags)
{
	struct drm_gem_object *obj = NULL;
	int ret;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	size = PAGE_ALIGN(size);

	ret = vivante_gem_new_impl(dev, size, flags, &obj);
	if (ret)
		goto fail;

	ret = 0;
	if (flags & ETNA_BO_CMDSTREAM)
		drm_gem_private_object_init(dev, obj, size);
	else
		ret = drm_gem_object_init(dev, obj, size);

	if (ret)
		goto fail;

	return obj;

fail:
	if (obj)
		drm_gem_object_unreference(obj);

	return ERR_PTR(ret);
}

struct drm_gem_object *msm_gem_import(struct drm_device *dev,
		uint32_t size, struct sg_table *sgt)
{
	struct vivante_gem_object *vivante_obj;
	struct drm_gem_object *obj;
	int ret, npages;

	size = PAGE_ALIGN(size);

	ret = vivante_gem_new_impl(dev, size, MSM_BO_WC, &obj);
	if (ret)
		goto fail;

	drm_gem_private_object_init(dev, obj, size);

	npages = size / PAGE_SIZE;

	vivante_obj = to_vivante_bo(obj);
	vivante_obj->sgt = sgt;
	vivante_obj->pages = drm_malloc_ab(npages, sizeof(struct page *));
	if (!vivante_obj->pages) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = drm_prime_sg_to_page_addr_arrays(sgt, vivante_obj->pages, NULL, npages);
	if (ret)
		goto fail;

	return obj;

fail:
	if (obj)
		drm_gem_object_unreference_unlocked(obj);

	return ERR_PTR(ret);
}
