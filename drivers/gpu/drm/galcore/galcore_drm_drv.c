/****************************************************************************
*
*    Copyright (C) 2014 by Jon Nettleton <jon.nettleton@gmail.com> 
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


/* galcore_drm_drv.c -- galcore_drm driver -*- linux-c -*-
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *    Rickard E. (Rik) Faith <faith@valinux.com>
 *    Daryll Strauss <daryll@valinux.com>
 *    Gareth Hughes <gareth@valinux.com>
 */

#include <linux/module.h>
#include <drm/drmP.h>

#include "galcore_drm_gem.h"
#include <drm/galcore_drm.h>
#include "galcore_drm_ioctlP.h"

static struct platform_device *galcore_drm_pdev;

static struct drm_ioctl_desc galcore_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(GALCORE_GEM_CREATE, galcore_drm_gem_create_ioctl,
		DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(GALCORE_GEM_MMAP, galcore_drm_gem_mmap_ioctl,
		DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(GALCORE_GEM_PWRITE, galcore_drm_gem_pwrite_ioctl,
		DRM_UNLOCKED),
};

static const struct file_operations galcore_drm_driver_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.poll			= drm_poll,
	.unlocked_ioctl 	= drm_ioctl,
	.mmap			= drm_gem_mmap,
	.open			= drm_open,
	.release		= drm_release,
};

static struct drm_driver galcore_drm_driver = {
	.fops			= &galcore_drm_driver_fops,
	.name			= "galcore-drm",
	.desc			= "Vivante galcore DRM",
	.date			= "20140824",
	.gem_free_object	= galcore_drm_gem_free_object,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= galcore_drm_gem_prime_export,
	.gem_prime_import	= galcore_drm_gem_prime_import,
#if 0
	.dumb_create		= galcore_drm_gem_dumb_create,
	.dumb_map_offset	= galcore_drm_gem_dumb_map_offset,
	.dumb_destroy		= galcore_drm_gem_dumb_destroy,
#endif
	.gem_vm_ops		= &galcore_drm_gem_vm_ops,
	.major			= 1,
	.minor			= 0,
	.driver_features	= DRIVER_GEM | DRIVER_PRIME,
	.ioctls			= galcore_drm_ioctls,
};

static int galcore_drm_platform_probe(struct platform_device *pdev)
{
	int ret;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	return drm_platform_init(&galcore_drm_driver, pdev);
}

static int galcore_drm_platform_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));

	return 0;
}

static struct platform_driver galcore_drm_platform_driver = {
	.probe		= galcore_drm_platform_probe,
	.remove		= galcore_drm_platform_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "galcore-drm",
	},
};

static int __init galcore_drm_init(void)
{
	int ret;

	ret = platform_driver_register(&galcore_drm_platform_driver);
	if (ret < 0)
		goto out;

	galcore_drm_pdev = platform_device_register_simple("galcore-drm", -1,
					NULL, 0);
	if (galcore_drm_pdev == NULL)
		printk(KERN_ERR"Platform device is null\n");

out:
	return ret;
}

static void __exit galcore_drm_exit(void)
{
	platform_device_unregister(galcore_drm_pdev);

	platform_driver_unregister(&galcore_drm_platform_driver);
}

module_init(galcore_drm_init);
module_exit(galcore_drm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");
