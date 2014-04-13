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

#include "msm_drv.h"
#include "msm_gpu.h"

int msm_register_mmu(struct drm_device *dev, struct msm_mmu *mmu)
{
	struct msm_drm_private *priv = dev->dev_private;
	int idx = priv->num_mmus++;

	if (WARN_ON(idx >= ARRAY_SIZE(priv->mmus)))
		return -EINVAL;

	priv->mmus[idx] = mmu;

	return idx;
}

#ifdef CONFIG_DRM_VIVANTE_REGISTER_LOGGING
static bool reglog = false;
MODULE_PARM_DESC(reglog, "Enable register read/write logging");
module_param(reglog, bool, 0600);
#else
#define reglog 0
#endif

static char *vram;
MODULE_PARM_DESC(vram, "Configure VRAM size (for devices without IOMMU/GPUMMU");
module_param(vram, charp, 0);

void __iomem *vivante_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname)
{
	struct resource *res;
	unsigned long size;
	void __iomem *ptr;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource: %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	size = resource_size(res);

	ptr = devm_ioremap_nocache(&pdev->dev, res->start, size);
	if (!ptr) {
		dev_err(&pdev->dev, "failed to ioremap: %s\n", name);
		return ERR_PTR(-ENOMEM);
	}

	if (reglog)
		printk(KERN_DEBUG "IO:region %s %08x %08lx\n", dbgname, (u32)ptr, size);

	return ptr;
}

void vivante_writel(u32 data, void __iomem *addr)
{
	if (reglog)
		printk(KERN_DEBUG "IO:W %08x %08x\n", (u32)addr, data);
	writel(data, addr);
}

u32 vivante_readl(const void __iomem *addr)
{
	u32 val = readl(addr);
	if (reglog)
		printk(KERN_ERR "IO:R %08x %08x\n", (u32)addr, val);
	return val;
}

/*
 * DRM operations:
 */

static int vivante_unload(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	unsigned int i;

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	for (i = 0; i < VIVANTE_MAX_PIPES; i++) {
		struct msm_gpu *gpu = priv->gpu[i];
		if (gpu) {
			mutex_lock(&dev->struct_mutex);
			gpu->funcs->pm_suspend(gpu);
			gpu->funcs->destroy(gpu);
			mutex_unlock(&dev->struct_mutex);
		}
	}

	if (priv->vram.paddr) {
		DEFINE_DMA_ATTRS(attrs);
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
		drm_mm_takedown(&priv->vram.mm);
		dma_free_attrs(dev->dev, priv->vram.size, NULL,
				priv->vram.paddr, &attrs);
	}

	dev->dev_private = NULL;

	kfree(priv);

	return 0;
}

static struct msm_gpu *vivante_gpu_3d_init(struct drm_device *dev)
{
	int ret;
	struct msm_gpu *gpu;

	gpu = kzalloc(sizeof(*gpu), GFP_KERNEL);
	if (!gpu) {
		return NULL;
	}

	ret = msm_gpu_init(dev, gpu, NULL, "vivante_gpu_3d", "iobase-3d", "irq-3d", 0);
	if (ret < 0) {
		dev_err(dev->dev, "%s init failed: %d\n", __func__, ret);
		kfree(gpu);
		gpu = NULL;
	}

	return gpu;
}

static void load_gpu(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu[VIVANTE_MAX_PIPES];
	unsigned int i;

	mutex_lock(&dev->struct_mutex);

	gpu[VIVANTE_PIPE_2D] = NULL;
	gpu[VIVANTE_PIPE_3D] = vivante_gpu_3d_init(dev);
	gpu[VIVANTE_PIPE_VG] = NULL;

	mutex_unlock(&dev->struct_mutex);

	for (i = 0; i < VIVANTE_MAX_PIPES; i++) {
		struct msm_gpu *g = priv->gpu[i];
		if (g) {
			int ret;
			g->funcs->pm_resume(g);
			ret = g->funcs->hw_init(g);
			if (ret) {
				dev_err(dev->dev, "%s hw init failed: %d\n", g->name, ret);
				g->funcs->destroy(g);
				g = NULL;
			}
		}
	}

	priv->gpu[VIVANTE_PIPE_2D] = gpu[VIVANTE_PIPE_2D];
	priv->gpu[VIVANTE_PIPE_3D] = gpu[VIVANTE_PIPE_3D];
	priv->gpu[VIVANTE_PIPE_VG] = gpu[VIVANTE_PIPE_VG];
}

static int vivante_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct msm_drm_private *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("vivante", 0);
	init_waitqueue_head(&priv->fence_event);

	INIT_LIST_HEAD(&priv->inactive_list);
	INIT_LIST_HEAD(&priv->fence_cbs);

	/* if we have no IOMMU, then we need to use carveout allocator.
	 * Grab the entire CMA chunk carved out in early startup in
	 * mach-msm:
	 */
	if (!iommu_present(&platform_bus_type)) {
		DEFINE_DMA_ATTRS(attrs);
		unsigned long size;
		void *p;

		DBG("using %s VRAM carveout", vram);
		size = memparse(vram, NULL);
		priv->vram.size = size;

		drm_mm_init(&priv->vram.mm, 0, (size >> PAGE_SHIFT) - 1);

		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
		dma_set_attr(DMA_ATTR_WRITE_COMBINE, &attrs);

		/* note that for no-kernel-mapping, the vaddr returned
		 * is bogus, but non-null if allocation succeeded:
		 */
		p = dma_alloc_attrs(dev->dev, size,
				&priv->vram.paddr, 0, &attrs);
		if (!p) {
			dev_err(dev->dev, "failed to allocate VRAM\n");
			priv->vram.paddr = 0;
			ret = -ENOMEM;
			goto fail;
		}

		dev_info(dev->dev, "VRAM: %08x->%08x\n",
				(uint32_t)priv->vram.paddr,
				(uint32_t)(priv->vram.paddr + size));
	}

	platform_set_drvdata(pdev, dev);

	load_gpu(dev);

	return 0;

fail:
	vivante_unload(dev);
	return ret;
}


static int msm_open(struct drm_device *dev, struct drm_file *file)
{
	struct msm_file_private *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	file->driver_priv = ctx;

	return 0;
}

static void msm_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_file_private *ctx = file->driver_priv;

	mutex_lock(&dev->struct_mutex);
	if (ctx == priv->lastctx)
		priv->lastctx = NULL;
	mutex_unlock(&dev->struct_mutex);

	kfree(ctx);
}

/*
 * DRM debugfs:
 */

#ifdef CONFIG_DEBUG_FS
static int msm_gpu_show(struct drm_device *dev, struct seq_file *m)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu;
	unsigned int i;

	for (i = 0; i < VIVANTE_MAX_PIPES; i++) {
		gpu = priv->gpu[i];
		if (gpu) {
			seq_printf(m, "%s Status:\n", gpu->name);
			gpu->funcs->show(gpu, m);
		}
	}

	return 0;
}

static int msm_gem_show(struct drm_device *dev, struct seq_file *m)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu;
	unsigned int i;

	for (i = 0; i < VIVANTE_MAX_PIPES; i++) {
		gpu = priv->gpu[i];
		if (gpu) {
			seq_printf(m, "Active Objects (%s):\n", gpu->name);
			msm_gem_describe_objects(&gpu->active_list, m);
		}
	}

	seq_printf(m, "Inactive Objects:\n");
	msm_gem_describe_objects(&priv->inactive_list, m);

	return 0;
}
#endif
static int msm_mm_show(struct drm_device *dev, struct seq_file *m)
{
	return drm_mm_dump_table(m, &dev->vma_offset_manager->vm_addr_space_mm);
}

static int show_locked(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	int (*show)(struct drm_device *dev, struct seq_file *m) =
			node->info_ent->data;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = show(dev, m);

	mutex_unlock(&dev->struct_mutex);

	return ret;
}

static struct drm_info_list msm_debugfs_list[] = {
		{"gpu", show_locked, 0, msm_gpu_show},
		{"gem", show_locked, 0, msm_gem_show},
		{ "mm", show_locked, 0, msm_mm_show },
};

static int msm_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	ret = drm_debugfs_create_files(msm_debugfs_list,
			ARRAY_SIZE(msm_debugfs_list),
			minor->debugfs_root, minor);

	if (ret) {
		dev_err(dev->dev, "could not install msm_debugfs_list\n");
		return ret;
	}

	return ret;
}

static void msm_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(msm_debugfs_list,
			ARRAY_SIZE(msm_debugfs_list), minor);
}


/*
 * Fences:
 */
int msm_wait_fence_interruptable(struct drm_device *dev, uint32_t fence,
		struct timespec *timeout)
{
#if 0
	struct msm_drm_private *priv = dev->dev_private;
	int ret;
#endif

	/* TODO */
	return 0;
#if 0
	if (!priv->gpu)
		return 0;

	if (fence > priv->gpu->submitted_fence) {
		DRM_ERROR("waiting on invalid fence: %u (of %u)\n",
				fence, priv->gpu->submitted_fence);
		return -EINVAL;
	}

	if (!timeout) {
		/* no-wait: */
		ret = fence_completed(dev, fence) ? 0 : -EBUSY;
	} else {
		unsigned long timeout_jiffies = timespec_to_jiffies(timeout);
		unsigned long start_jiffies = jiffies;
		unsigned long remaining_jiffies;

		if (time_after(start_jiffies, timeout_jiffies))
			remaining_jiffies = 0;
		else
			remaining_jiffies = timeout_jiffies - start_jiffies;

		ret = wait_event_interruptible_timeout(priv->fence_event,
				fence_completed(dev, fence),
				remaining_jiffies);

		if (ret == 0) {
			DBG("timeout waiting for fence: %u (completed: %u)",
					fence, priv->completed_fence);
			ret = -ETIMEDOUT;
		} else if (ret != -ERESTARTSYS) {
			ret = 0;
		}
	}

	return ret;
#endif
}

/* called from workqueue */
void msm_update_fence(struct drm_device *dev, uint32_t fence)
{
#if 0
	struct msm_drm_private *priv = dev->dev_private;

	mutex_lock(&dev->struct_mutex);
	priv->completed_fence = max(fence, priv->completed_fence);

	while (!list_empty(&priv->fence_cbs)) {
		struct msm_fence_cb *cb;

		cb = list_first_entry(&priv->fence_cbs,
				struct msm_fence_cb, work.entry);

		if (cb->fence > priv->completed_fence)
			break;

		list_del_init(&cb->work.entry);
		queue_work(priv->wq, &cb->work);
	}

	mutex_unlock(&dev->struct_mutex);

	wake_up_all(&priv->fence_event);
#endif
}

#if 0
void __msm_fence_worker(struct work_struct *work)
{
	struct msm_fence_cb *cb = container_of(work, struct msm_fence_cb, work);
	cb->func(cb);
}
#endif
/*
 * DRM ioctls:
 */

static int vivante_ioctl_get_param(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_vivante_param *args = data;
	struct msm_gpu *gpu;

	if (args->pipe > VIVANTE_PIPE_VG)
		return -EINVAL;

	gpu = priv->gpu[args->pipe];
	if (!gpu)
		return -ENXIO;

	return gpu->funcs->get_param(gpu, args->param, &args->value);
}

static int msm_ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_new *args = data;
	return msm_gem_new_handle(dev, file, args->size,
			args->flags, &args->handle);
}

#define TS(t) ((struct timespec){ .tv_sec = (t).tv_sec, .tv_nsec = (t).tv_nsec })

static int msm_ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_prep(obj, args->op, &TS(args->timeout));

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_fini(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_info *args = data;
	struct drm_gem_object *obj;
	int ret = 0;

	if (args->pad)
		return -EINVAL;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	args->offset = msm_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int msm_ioctl_wait_fence(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_wait_fence *args = data;
	return msm_wait_fence_interruptable(dev, args->fence, &TS(args->timeout));
}
static const struct drm_ioctl_desc vivante_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MSM_GET_PARAM,    vivante_ioctl_get_param,    DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_NEW,      msm_ioctl_gem_new,      DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_INFO,     msm_ioctl_gem_info,     DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_PREP, msm_ioctl_gem_cpu_prep, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_FINI, msm_ioctl_gem_cpu_fini, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_SUBMIT,   msm_ioctl_gem_submit,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_WAIT_FENCE,   msm_ioctl_wait_fence,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
};

static const struct vm_operations_struct vm_ops = {
	.fault = msm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = msm_gem_mmap,
};

static struct drm_driver vivante_driver = {
	.driver_features    = DRIVER_HAVE_IRQ |
				DRIVER_GEM |
				DRIVER_PRIME |
				DRIVER_RENDER,
	.load               = vivante_load,
	.unload             = vivante_unload,
	.open               = msm_open,
	.preclose           = msm_preclose,
	.gem_free_object    = msm_gem_free_object,
	.gem_vm_ops         = &vm_ops,
	.dumb_create        = msm_gem_dumb_create,
	.dumb_map_offset    = msm_gem_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export   = drm_gem_prime_export,
	.gem_prime_import   = drm_gem_prime_import,
	.gem_prime_pin      = msm_gem_prime_pin,
	.gem_prime_unpin    = msm_gem_prime_unpin,
	.gem_prime_get_sg_table = msm_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = msm_gem_prime_import_sg_table,
	.gem_prime_vmap     = msm_gem_prime_vmap,
	.gem_prime_vunmap   = msm_gem_prime_vunmap,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init       = msm_debugfs_init,
	.debugfs_cleanup    = msm_debugfs_cleanup,
#endif
	.ioctls             = vivante_ioctls,
	.num_ioctls         = DRM_MSM_NUM_IOCTLS,
	.fops               = &fops,
	.name               = "vivante",
	.desc               = "Vivante DRM",
	.date               = "20130625",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */

static int vivante_pdev_probe(struct platform_device *pdev)
{
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	return drm_platform_init(&vivante_driver, pdev);
}

static int vivante_pdev_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "vivante" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver vivante_platform_driver = {
	.probe      = vivante_pdev_probe,
	.remove     = vivante_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "vivante",
		.of_match_table = dt_match,
	},
};

module_platform_driver(vivante_platform_driver);

MODULE_AUTHOR("Rob Clark <robdclark@gmail.com");
MODULE_DESCRIPTION("Vivante DRM Driver");
MODULE_LICENSE("GPL");
