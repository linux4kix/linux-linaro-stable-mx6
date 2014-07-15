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

#include <linux/component.h>
#include <linux/of_device.h>
#include "vivante_gpu.h"
#include "vivante_gem.h"
#include "vivante_mmu.h"
#include "vivante_iommu.h"
#include "common.xml.h"
#include "state.xml.h"
#include "state_hi.xml.h"
#include "cmdstream.xml.h"


/*
 * Driver functions:
 */

int vivante_gpu_get_param(struct vivante_gpu *gpu, uint32_t param, uint64_t *value)
{
	switch (param) {
	case VIVANTE_PARAM_GPU_MODEL:
		*value = gpu->identity.model;
		break;

	case VIVANTE_PARAM_GPU_REVISION:
		*value = gpu->identity.revision;
		break;

	case VIVANTE_PARAM_GPU_FEATURES_0:
		*value = gpu->identity.features;
		break;

	case VIVANTE_PARAM_GPU_FEATURES_1:
		*value = gpu->identity.minor_features;
		break;

	case VIVANTE_PARAM_GPU_FEATURES_2:
		*value = gpu->identity.minor_features1;
		break;

	case VIVANTE_PARAM_GPU_FEATURES_3:
		*value = gpu->identity.minor_features2;
		break;

	default:
		DBG("%s: invalid param: %u", gpu->name, param);
		return -EINVAL;
	}

	return 0;
}

static void vivante_hw_identify(struct vivante_gpu *gpu)
{
	u32 chipIdentity;

	chipIdentity = gpu_read(gpu, VIVS_HI_CHIP_IDENTITY);

	/* Special case for older graphic cores. */
	if (VIVS_HI_CHIP_IDENTITY_FAMILY(chipIdentity) ==  0x01) {
		gpu->identity.model    = 0x500; /* gc500 */
		gpu->identity.revision = VIVS_HI_CHIP_IDENTITY_REVISION(chipIdentity);
	} else {

		gpu->identity.model = gpu_read(gpu, VIVS_HI_CHIP_MODEL);
		gpu->identity.revision = gpu_read(gpu, VIVS_HI_CHIP_REV);

		/* !!!! HACK ALERT !!!! */
		/* Because people change device IDs without letting software know
		** about it - here is the hack to make it all look the same.  Only
		** for GC400 family.  Next time - TELL ME!!! */
		if (((gpu->identity.model & 0xFF00) == 0x0400)
		&& (gpu->identity.model != 0x0420)) {
			gpu->identity.model = gpu->identity.model & 0x0400;
		}

		/* An other special case */
		if ((gpu->identity.model    == 0x300)
		&&  (gpu->identity.revision == 0x2201)) {
			u32 chipDate = gpu_read(gpu, VIVS_HI_CHIP_DATE);
			u32 chipTime = gpu_read(gpu, VIVS_HI_CHIP_TIME);

			if ((chipDate == 0x20080814) && (chipTime == 0x12051100)) {
				/* This IP has an ECO; put the correct revision in it. */
				gpu->identity.revision = 0x1051;
			}
		}
	}

	dev_info(gpu->dev->dev, "model: %x\n", gpu->identity.model);
	dev_info(gpu->dev->dev, "revision: %x\n", gpu->identity.revision);

	gpu->identity.features = gpu_read(gpu, VIVS_HI_CHIP_FEATURE);

	/* Disable fast clear on GC700. */
	if (gpu->identity.model == 0x700)
		gpu->identity.features &= ~BIT(0);

	if (((gpu->identity.model == 0x500) && (gpu->identity.revision < 2))
	||  ((gpu->identity.model == 0x300) && (gpu->identity.revision < 0x2000))) {

		/* GC500 rev 1.x and GC300 rev < 2.0 doesn't have these registers. */
		gpu->identity.minor_features  = 0;
		gpu->identity.minor_features1 = 0;
		gpu->identity.minor_features2 = 0;
		gpu->identity.minor_features3 = 0;
	} else
		gpu->identity.minor_features = gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_0);

	if (gpu->identity.minor_features & BIT(21)) {
		gpu->identity.minor_features1 = gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_1);
		gpu->identity.minor_features2 = gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_2);
		gpu->identity.minor_features3 = gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_3);
	}

	dev_info(gpu->dev->dev, "minor_features:  %x\n", gpu->identity.minor_features);
	dev_info(gpu->dev->dev, "minor_features1: %x\n", gpu->identity.minor_features1);
	dev_info(gpu->dev->dev, "minor_features2: %x\n", gpu->identity.minor_features2);
	dev_info(gpu->dev->dev, "minor_features3: %x\n", gpu->identity.minor_features3);
}

static void vivante_hw_reset(struct vivante_gpu *gpu)
{
	u32 control, idle;

	/* TODO
	 *
	 * - clock gating
	 * - puls eater
	 * - what about VG?
	 */

	while (true)
	{
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* isolate the GPU. */
		control |= VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* set soft reset. */
		control |= VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* wait for reset. */
		msleep(1);

		/* reset soft reset bit. */
		control &= ~VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* reset GPU isolation. */
		control &= ~VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* read idle register. */
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		/* try reseting again if FE it not idle */
		if ((idle & VIVS_HI_IDLE_STATE_FE) == 0) {
			dev_dbg(gpu->dev->dev, "%s: FE is not idle\n", gpu->name);
			continue;
		}

		/* read reset register. */
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* is the GPU idle? */
		if (((control & VIVS_HI_CLOCK_CONTROL_IDLE_3D) == 0)
		|| ((control & VIVS_HI_CLOCK_CONTROL_IDLE_2D) == 0)) {
			dev_dbg(gpu->dev->dev, "%s: GPU is not idle\n", gpu->name);
			continue;
		}

		break;
	}
}

int vivante_gpu_init(struct vivante_gpu *gpu)
{
	int ret, i;
	u32 words; /* 32 bit words */
	struct iommu_domain *iommu;

	vivante_hw_identify(gpu);
	vivante_hw_reset(gpu);

	/* set base addresses */
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_RA, 0x0);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_FE, 0x0);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_TX, 0x0);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_PEZ, 0x0);
	gpu_write(gpu, VIVS_MC_MEMORY_BASE_ADDR_PE, 0x0);

	/* Setup IOMMU.. eventually we will (I think) do this once per context
	 * and have separate page tables per context.  For now, to keep things
	 * simple and to get something working, just use a single address space:
	 */
	iommu = vivante_iommu_domain_alloc(gpu);
	if (!iommu) {
		ret = -ENOMEM;
		goto fail;
	}

	/* TODO: we will leak here memory - fix it! */

	gpu->mmu = vivante_iommu_new(gpu->dev, iommu);
	if (!gpu->mmu) {
		ret = -ENOMEM;
		goto fail;
	}
	vivante_register_mmu(gpu->dev, gpu->mmu);

	/* Create ringbuffer: */
	gpu->rb = vivante_ringbuffer_new(gpu, PAGE_SIZE);
	if (IS_ERR(gpu->rb)) {
		ret = PTR_ERR(gpu->rb);
		gpu->rb = NULL;
		dev_err(gpu->dev->dev, "could not create ringbuffer: %d\n", ret);
		goto fail;
	}

	/* Setup event management */
	mutex_init(&gpu->event_mutex);
	init_completion(&gpu->event_free);
	for (i = 0; i < ARRAY_SIZE(gpu->event_used); i++) {
		gpu->event_used[i] = false;
		complete(&gpu->event_free);
	}

	/* Start command processor */
	words = vivante_cmd_init(gpu);

	/* convert number of 32 bit words to number of 64 bit words */
	words = ALIGN(words, 2) / 2;

	gpu_write(gpu, VIVS_HI_INTR_ENBL, ~0U);
	gpu_write(gpu, VIVS_FE_COMMAND_ADDRESS, vivante_gem_paddr_locked(gpu->rb->bo));
	gpu_write(gpu, VIVS_FE_COMMAND_CONTROL, VIVS_FE_COMMAND_CONTROL_ENABLE | VIVS_FE_COMMAND_CONTROL_PREFETCH(words));

	return 0;

fail:
	return ret;
}

#ifdef CONFIG_DEBUG_FS
void vivante_gpu_debugfs(struct vivante_gpu *gpu, struct seq_file *m)
{
	u32 dma_address = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	u32 dma_state = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);
	u32 dma_lo = gpu_read(gpu, VIVS_FE_DMA_LOW);
	u32 dma_hi = gpu_read(gpu, VIVS_FE_DMA_HIGH);
	u32 axi = gpu_read(gpu, VIVS_HI_AXI_STATUS);
	u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

	seq_printf(m, "\taxi: 0x08%x\n", axi);
	seq_printf(m, "\tidle: 0x08%x\n", idle);
	if ((idle & VIVS_HI_IDLE_STATE_FE) == 0)
		seq_printf(m, "\t FE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_DE) == 0)
		seq_printf(m, "\t DE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PE) == 0)
		seq_printf(m, "\t PE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SH) == 0)
		seq_printf(m, "\t SH is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PA) == 0)
		seq_printf(m, "\t PA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SE) == 0)
		seq_printf(m, "\t SE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_RA) == 0)
		seq_printf(m, "\t RA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TX) == 0)
		seq_printf(m, "\t TX is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_VG) == 0)
		seq_printf(m, "\t VG is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_IM) == 0)
		seq_printf(m, "\t IM is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_FP) == 0)
		seq_printf(m, "\t FP is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TS) == 0)
		seq_printf(m, "\t TS is not idle\n");
	if (idle & VIVS_HI_IDLE_STATE_AXI_LP)
		seq_printf(m, "\t AXI low power mode\n");

	if (gpu->identity.features & chipFeatures_DEBUG_MODE) {
		u32 read0 = gpu_read(gpu, VIVS_MC_DEBUG_READ0);
		u32 read1 = gpu_read(gpu, VIVS_MC_DEBUG_READ1);
		u32 write = gpu_read(gpu, VIVS_MC_DEBUG_WRITE);

		seq_printf(m, "\tMC\n");
		seq_printf(m, "\t read0: 0x%08x\n", read0);
		seq_printf(m, "\t read1: 0x%08x\n", read1);
		seq_printf(m, "\t write: 0x%08x\n", write);
	}

	seq_printf(m, "\tDMA\n");
	seq_printf(m, "\t address: 0x%08x\n", dma_address);
	seq_printf(m, "\t state: 0x%08x\n", dma_state);
	seq_printf(m, "\t last fetch 64 bit word: 0x%08x-0x%08x\n", dma_hi, dma_lo);
}
#endif

/*
 * Power Management:
 */

static int enable_pwrrail(struct vivante_gpu *gpu)
{
#if 0
	struct drm_device *dev = gpu->dev;
	int ret = 0;

	if (gpu->gpu_reg) {
		ret = regulator_enable(gpu->gpu_reg);
		if (ret) {
			dev_err(dev->dev, "failed to enable 'gpu_reg': %d\n", ret);
			return ret;
		}
	}

	if (gpu->gpu_cx) {
		ret = regulator_enable(gpu->gpu_cx);
		if (ret) {
			dev_err(dev->dev, "failed to enable 'gpu_cx': %d\n", ret);
			return ret;
		}
	}
#endif
	return 0;
}

static int disable_pwrrail(struct vivante_gpu *gpu)
{
#if 0
	if (gpu->gpu_cx)
		regulator_disable(gpu->gpu_cx);
	if (gpu->gpu_reg)
		regulator_disable(gpu->gpu_reg);
#endif
	return 0;
}

static int enable_clk(struct vivante_gpu *gpu)
{
	if (gpu->clk_core)
		clk_prepare_enable(gpu->clk_core);
	if (gpu->clk_shader)
		clk_prepare_enable(gpu->clk_shader);

	return 0;
}

static int disable_clk(struct vivante_gpu *gpu)
{
	if (gpu->clk_core)
		clk_disable_unprepare(gpu->clk_core);
	if (gpu->clk_shader)
		clk_disable_unprepare(gpu->clk_shader);

	return 0;
}

static int enable_axi(struct vivante_gpu *gpu)
{
	if (gpu->clk_bus)
		clk_prepare_enable(gpu->clk_bus);

	return 0;
}

static int disable_axi(struct vivante_gpu *gpu)
{
	if (gpu->clk_bus)
		clk_disable_unprepare(gpu->clk_bus);

	return 0;
}

int vivante_gpu_pm_resume(struct vivante_gpu *gpu)
{
	int ret;

	DBG("%s", gpu->name);

	ret = enable_pwrrail(gpu);
	if (ret)
		return ret;

	ret = enable_clk(gpu);
	if (ret)
		return ret;

	ret = enable_axi(gpu);
	if (ret)
		return ret;

	return 0;
}

int vivante_gpu_pm_suspend(struct vivante_gpu *gpu)
{
	int ret;

	DBG("%s", gpu->name);

	ret = disable_axi(gpu);
	if (ret)
		return ret;

	ret = disable_clk(gpu);
	if (ret)
		return ret;

	ret = disable_pwrrail(gpu);
	if (ret)
		return ret;

	return 0;
}

/*
 * Hangcheck detection for locked gpu:
 */
static void recover_worker(struct work_struct *work)
{
	struct vivante_gpu *gpu = container_of(work, struct vivante_gpu, recover_work);
	struct drm_device *dev = gpu->dev;

	dev_err(dev->dev, "%s: hangcheck recover!\n", gpu->name);

	mutex_lock(&dev->struct_mutex);
	/* TODO gpu->funcs->recover(gpu); */
	mutex_unlock(&dev->struct_mutex);

	vivante_gpu_retire(gpu);
}

static void hangcheck_timer_reset(struct vivante_gpu *gpu)
{
	DBG("%s", gpu->name);
	mod_timer(&gpu->hangcheck_timer,
			round_jiffies_up(jiffies + DRM_MSM_HANGCHECK_JIFFIES));
}

static void hangcheck_handler(unsigned long data)
{
	struct vivante_gpu *gpu = (struct vivante_gpu *)data;
	struct drm_device *dev = gpu->dev;
	struct vivante_drm_private *priv = dev->dev_private;
	uint32_t fence = 0; /* TODO: gpu->funcs->last_fence(gpu); */

	if (fence != gpu->hangcheck_fence) {
		/* some progress has been made.. ya! */
		gpu->hangcheck_fence = fence;
	} else if (fence < gpu->submitted_fence) {
		/* no progress and not done.. hung! */
		gpu->hangcheck_fence = fence;
		dev_err(dev->dev, "%s: hangcheck detected gpu lockup!\n",
				gpu->name);
		dev_err(dev->dev, "%s:     completed fence: %u\n",
				gpu->name, fence);
		dev_err(dev->dev, "%s:     submitted fence: %u\n",
				gpu->name, gpu->submitted_fence);
		queue_work(priv->wq, &gpu->recover_work);
	}

	/* if still more pending work, reset the hangcheck timer: */
	if (gpu->submitted_fence > gpu->hangcheck_fence)
		hangcheck_timer_reset(gpu);

	/* workaround for missing irq: */
	queue_work(priv->wq, &gpu->retire_work);
}

/*
 * event management:
 */

static unsigned int event_alloc(struct vivante_gpu *gpu)
{
	unsigned long ret;
	unsigned int i, event = ~0U;

	ret = wait_for_completion_timeout(&gpu->event_free, msecs_to_jiffies(10 * 10000));
	if (ret != 0) {
		dev_err(gpu->dev->dev, "wait_for_completion_timeout failed");
	}

	mutex_lock(&gpu->event_mutex);

	/* find first free event */
	for (i = 0; i < ARRAY_SIZE(gpu->event_used); i++) {
		if (gpu->event_used[i] == false) {
			gpu->event_used[i] = true;
			event = i;
		}
	}

	mutex_unlock(&gpu->event_mutex);

	return event;
}

static void event_free(struct vivante_gpu *gpu, unsigned int event)
{
	mutex_lock(&gpu->event_mutex);

	if (gpu->event_used[event] == false) {
		dev_warn(gpu->dev->dev, "event %u is already marked as free", event);
		mutex_unlock(&gpu->event_mutex);
	} else {
		gpu->event_used[event] = false;
		mutex_unlock(&gpu->event_mutex);

		complete(&gpu->event_free);
	}
}

/*
 * Cmdstream submission/retirement:
 */

static void retire_worker(struct work_struct *work)
{
	struct vivante_gpu *gpu = container_of(work, struct vivante_gpu, retire_work);
	struct drm_device *dev = gpu->dev;
	uint32_t fence = gpu->retired_fence;

	vivante_update_fence(gpu->dev, fence);

	mutex_lock(&dev->struct_mutex);

	while (!list_empty(&gpu->active_list)) {
		struct vivante_gem_object *obj;

		obj = list_first_entry(&gpu->active_list,
				struct vivante_gem_object, mm_list);

		if ((obj->read_fence <= fence) &&
				(obj->write_fence <= fence)) {
			/* move to inactive: */
			vivante_gem_move_to_inactive(&obj->base);
			vivante_gem_put_iova(&obj->base);
			drm_gem_object_unreference(&obj->base);
		} else {
			break;
		}
	}

	mutex_unlock(&dev->struct_mutex);
}

/* call from irq handler to schedule work to retire bo's */
void vivante_gpu_retire(struct vivante_gpu *gpu)
{
	struct vivante_drm_private *priv = gpu->dev->dev_private;
	queue_work(priv->wq, &gpu->retire_work);
}

/* add bo's to gpu's ring, and kick gpu: */
int vivante_gpu_submit(struct vivante_gpu *gpu, struct vivante_gem_submit *submit,
		struct vivante_file_private *ctx)
{
	struct drm_device *dev = gpu->dev;
	struct vivante_drm_private *priv = dev->dev_private;
	int ret;
	unsigned int event, i;

	submit->fence = ++priv->next_fence;

	gpu->submitted_fence = submit->fence;

	/*
	 * TODO
	 *
	 * - flush
	 * - data endian
	 * - prefetch
	 *
	 */

	event = event_alloc(gpu);
	gpu->event_to_fence[event] = submit->fence;

	ret = 0;

	priv->lastctx = ctx;

	for (i = 0; i < submit->nr_bos; i++) {
		struct vivante_gem_object *vivante_obj = submit->bos[i].obj;

		/* can't happen yet.. but when we add 2d support we'll have
		 * to deal w/ cross-ring synchronization:
		 */
		WARN_ON(is_active(vivante_obj) && (vivante_obj->gpu != gpu));

		if (!is_active(vivante_obj)) {
			uint32_t iova;

			/* ring takes a reference to the bo and iova: */
			drm_gem_object_reference(&vivante_obj->base);
			vivante_gem_get_iova_locked(gpu, &vivante_obj->base, &iova);
		}

		if (submit->bos[i].flags & ETNA_SUBMIT_BO_READ)
			vivante_gem_move_to_active(&vivante_obj->base, gpu, false, submit->fence);

		if (submit->bos[i].flags & ETNA_SUBMIT_BO_WRITE)
			vivante_gem_move_to_active(&vivante_obj->base, gpu, true, submit->fence);
	}
	hangcheck_timer_reset(gpu);

	return ret;
}

/*
 * Init/Cleanup:
 */
static irqreturn_t irq_handler(int irq, void *data)
{
	struct vivante_gpu *gpu = data;
	irqreturn_t ret = IRQ_NONE;

	u32 event = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);

	if (event != 0) {
		if (event & VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR)
			dev_err(gpu->dev->dev, "AXI bus error\n");

		/* handle irq */
		dev_info(gpu->dev->dev, "event 0x%08x\n", event);

		gpu->retired_fence = gpu->event_to_fence[event];
		event_free(gpu, event);
		vivante_gpu_retire(gpu);

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int vivante_gpu_bind(struct device *dev, struct device *master,
	void *data)
{
	struct drm_device *drm = data;
	struct vivante_drm_private *priv = drm->dev_private;
	struct vivante_gpu *gpu = dev_get_drvdata(dev);
	int idx = gpu->pipe;

	dev_info(dev, "pre gpu[idx]: 0x%08x\n", (u32)priv->gpu[idx]);

	if (priv->gpu[idx] == 0) {
		dev_info(dev, "adding core @idx %d\n", idx);
		priv->gpu[idx] = gpu;
	} else {
		dev_err(dev, "failed to add core @idx %d\n", idx);
		goto fail;
	}

	dev_info(dev, "post gpu[idx]: 0x%08x\n", (u32)priv->gpu[idx]);

	gpu->dev = drm;

	INIT_LIST_HEAD(&gpu->active_list);
	INIT_WORK(&gpu->retire_work, retire_worker);
	INIT_WORK(&gpu->recover_work, recover_worker);

	setup_timer(&gpu->hangcheck_timer, hangcheck_handler,
			(unsigned long)gpu);
	return 0;
fail:
	return -1;
}

static void vivante_gpu_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct vivante_gpu *gpu = dev_get_drvdata(dev);

	DBG("%s", gpu->name);

	WARN_ON(!list_empty(&gpu->active_list));

	if (gpu->rb)
		vivante_ringbuffer_destroy(gpu->rb);

	if (gpu->mmu)
		vivante_iommu_destroy(gpu->mmu);

	drm_mm_takedown(&gpu->mm);
}

static const struct component_ops gpu_ops = {
	.bind = vivante_gpu_bind,
	.unbind = vivante_gpu_unbind,
};

static const struct of_device_id vivante_gpu_match[] = {
	{
		.compatible = "vivante,vivante-gpu-2d",
		.data = (void *)VIVANTE_PIPE_2D
	},
	{
		.compatible = "vivante,vivante-gpu-3d",
		.data = (void *)VIVANTE_PIPE_3D
	},
	{
		.compatible = "vivante,vivante-gpu-vg",
		.data = (void *)VIVANTE_PIPE_VG
	},
	{ }
};

static int vivante_gpu_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct vivante_gpu *gpu;
	int err = 0;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	match = of_match_device(vivante_gpu_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	gpu->name = pdev->name;

	/* Map registers: */
	gpu->mmio = vivante_ioremap(pdev, NULL, gpu->name);
	if (IS_ERR(gpu->mmio))
		return PTR_ERR(gpu->mmio);

	/* Get Interrupt: */
	gpu->irq = platform_get_irq(pdev, 0);
	if (gpu->irq < 0) {
		err = gpu->irq;
		dev_err(dev, "failed to get irq: %d\n", err);
		goto fail;
	}

	err = devm_request_irq(&pdev->dev, gpu->irq, irq_handler,
			IRQF_TRIGGER_HIGH, gpu->name, gpu);
	if (err) {
		dev_err(dev, "failed to request IRQ%u: %d\n", gpu->irq, err);
		goto fail;
	}

	/* Get Clocks: */
	gpu->clk_bus = devm_clk_get(&pdev->dev, "bus");
	DBG("clk_bus: %p", gpu->clk_bus);
	if (IS_ERR(gpu->clk_bus))
		gpu->clk_bus = NULL;

	gpu->clk_core = devm_clk_get(&pdev->dev, "core");
	DBG("clk_core: %p", gpu->clk_core);
	if (IS_ERR(gpu->clk_core))
		gpu->clk_core = NULL;

	gpu->clk_shader = devm_clk_get(&pdev->dev, "shader");
	DBG("clk_shader: %p", gpu->clk_shader);
	if (IS_ERR(gpu->clk_shader))
		gpu->clk_shader = NULL;

	gpu->pipe = (int)match->data;

	/* TODO: figure out max mapped size */
	drm_mm_init(&gpu->mm, 0x80000000, SZ_1G);

	dev_set_drvdata(dev, gpu);

	err = component_add(&pdev->dev, &gpu_ops);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register component: %d\n", err);
		goto fail;
	}

	return 0;

fail:
	return err;
}

static int vivante_gpu_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gpu_ops);
	return 0;
}

struct platform_driver vivante_gpu_driver = {
	.driver = {
		.name = "vivante-gpu",
		.owner = THIS_MODULE,
		.of_match_table = vivante_gpu_match,
	},
	.probe = vivante_gpu_platform_probe,
	.remove = vivante_gpu_platform_remove,
};
