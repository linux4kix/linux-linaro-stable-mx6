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

#include "vivante_gpu.h"
#include "vivante_gem.h"
#include "vivante_mmu.h"
#include "vivante_iommu.h"
#include "state_hi.xml.h"
#include "cmdstream.xml.h"

/*
 * Command Buffer helper:
 */

static inline void CMD_LOAD_STATE(struct vivante_ringbuffer *rb, u32 offset, u32 value)
{
	/* write a register via cmd stream */
	OUT_RING(rb, VIV_FE_LOAD_STATE_HEADER | VIV_FE_LOAD_STATE_HEADER_COUNT(1) | VIV_FE_LOAD_STATE_HEADER_OFFSET(offset));
	OUT_RING(rb, value);
}

static inline void CMD_END(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_END_HEADER);
}

static inline void CMD_NOP(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_NOP_HEADER);
}

static inline void CMD_WAIT(struct vivante_ringbuffer *rb)
{
	OUT_RING(rb, VIV_FE_WAIT_HEADER_OP_WAIT | 200);
}

static inline void CMD_LINK(struct vivante_ringbuffer *rb, u16 prefetch, u32 address)
{
	OUT_RING(rb, VIV_FE_LINK_HEADER | VIV_FE_LINK_HEADER_PREFETCH(prefetch));
	OUT_RING(rb, address);
}

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
		if ((idle & VIVS_HI_IDLE_STATE_FE) == 0)
		{
			dev_dbg(gpu->dev->dev, "%s: FE is not idle\n", gpu->name);
			continue;
		}

		/* read reset register. */
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* is the GPU idle? */
		if (((control & VIVS_HI_CLOCK_CONTROL_IDLE_3D) == 0)
		|| ((control & VIVS_HI_CLOCK_CONTROL_IDLE_2D) == 0))
		{
			dev_dbg(gpu->dev->dev, "%s: GPU is not idle\n", gpu->name);
			continue;
		}

		break;
	}
}

int vivante_hw_init(struct vivante_gpu *gpu)
{
	struct iommu_domain *iommu;
	int ret;

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
	if (iommu) {
		dev_info(gpu->dev->dev, "%s: using IOMMU\n", gpu->name);
		gpu->mmu = vivante_iommu_new(gpu->dev, iommu);
	} else {
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

    ret = vivante_gem_get_iova_locked(gpu->rb->bo, &gpu->rb_iova);
	if (ret) {
		gpu->rb_iova = 0;
		dev_err(gpu->dev->dev, "could not map ringbuffer: %d\n", ret);
		goto fail;
	}

	return 0;

fail:
	return ret;
}

irqreturn_t vivante_gpu_irq(struct vivante_gpu *gpu)
{
	irqreturn_t ret = IRQ_NONE;
	u32 irq = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);

	if (irq != 0)
	{
		if (irq & VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR)
			dev_err(gpu->dev->dev, "AXI bus error\n");

		/* TODO: handle irq */
		dev_info(gpu->dev->dev, "irq 0x%08x\n", irq);
		ret = IRQ_HANDLED;
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
void vivante_gpu_show(struct vivante_gpu *gpu, struct seq_file *m)
{
	/* TODO */
}
#endif

static const struct vivante_gpu_funcs funcs = {
	.hw_init = vivante_hw_init,
};

struct vivante_gpu *vivante_gpu_init(struct drm_device *dev,const char *name,
		const char *ioname, const char *irqname)
{
	int ret;
	struct vivante_gpu *gpu;

	gpu = kzalloc(sizeof(*gpu), GFP_KERNEL);
	if (!gpu) {
		return NULL;
	}

	ret = msm_gpu_init(dev, gpu, &funcs, name, ioname, irqname);
	if (ret < 0) {
		dev_err(dev->dev, "%s init failed: %d\n", __func__, ret);
		kfree(gpu);
		gpu = NULL;
	}

	return gpu;
}

/*
 * Power Management:
 */

static int enable_pwrrail(struct vivante_gpu *gpu)
{
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

	return 0;
}

static int disable_pwrrail(struct vivante_gpu *gpu)
{
	if (gpu->gpu_cx)
		regulator_disable(gpu->gpu_cx);
	if (gpu->gpu_reg)
		regulator_disable(gpu->gpu_reg);
	return 0;
}

static int enable_clk(struct vivante_gpu *gpu)
{
	struct clk *rate_clk = NULL;
	int i;

	/* NOTE: kgsl_pwrctrl_clk() ignores grp_clks[0].. */
	for (i = ARRAY_SIZE(gpu->grp_clks) - 1; i > 0; i--) {
		if (gpu->grp_clks[i]) {
			clk_prepare(gpu->grp_clks[i]);
			rate_clk = gpu->grp_clks[i];
		}
	}
#if 0
	if (rate_clk && gpu->fast_rate)
		clk_set_rate(rate_clk, gpu->fast_rate);
#endif
	for (i = ARRAY_SIZE(gpu->grp_clks) - 1; i > 0; i--)
		if (gpu->grp_clks[i])
			clk_enable(gpu->grp_clks[i]);

	return 0;
}

static int disable_clk(struct vivante_gpu *gpu)
{
	struct clk *rate_clk = NULL;
	int i;

	/* NOTE: kgsl_pwrctrl_clk() ignores grp_clks[0].. */
	for (i = ARRAY_SIZE(gpu->grp_clks) - 1; i > 0; i--) {
		if (gpu->grp_clks[i]) {
			clk_disable(gpu->grp_clks[i]);
			rate_clk = gpu->grp_clks[i];
		}
	}
#if 0
	if (rate_clk && gpu->slow_rate)
		clk_set_rate(rate_clk, gpu->slow_rate);
#endif
	for (i = ARRAY_SIZE(gpu->grp_clks) - 1; i > 0; i--)
		if (gpu->grp_clks[i])
			clk_unprepare(gpu->grp_clks[i]);

	return 0;
}

static int enable_axi(struct vivante_gpu *gpu)
{
#if 0
	if (gpu->ebi1_clk)
		clk_prepare_enable(gpu->ebi1_clk);
#endif
	return 0;
}

static int disable_axi(struct vivante_gpu *gpu)
{
#if 0
	if (gpu->ebi1_clk)
		clk_disable_unprepare(gpu->ebi1_clk);
#endif
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
	gpu->funcs->recover(gpu);
	mutex_unlock(&dev->struct_mutex);

	msm_gpu_retire(gpu);
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
	uint32_t fence = gpu->funcs->last_fence(gpu);

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
 * Cmdstream submission/retirement:
 */

static void retire_worker(struct work_struct *work)
{
	struct vivante_gpu *gpu = container_of(work, struct vivante_gpu, retire_work);
	struct drm_device *dev = gpu->dev;
	uint32_t fence = gpu->funcs->last_fence(gpu);

	msm_update_fence(gpu->dev, fence);

	mutex_lock(&dev->struct_mutex);

	while (!list_empty(&gpu->active_list)) {
		struct vivante_gem_object *obj;

		obj = list_first_entry(&gpu->active_list,
				struct vivante_gem_object, mm_list);

		if ((obj->read_fence <= fence) &&
				(obj->write_fence <= fence)) {
			/* move to inactive: */
			msm_gem_move_to_inactive(&obj->base);
            vivante_gem_put_iova(&obj->base);
			drm_gem_object_unreference(&obj->base);
		} else {
			break;
		}
	}

	mutex_unlock(&dev->struct_mutex);
}

/* call from irq handler to schedule work to retire bo's */
void msm_gpu_retire(struct vivante_gpu *gpu)
{
	struct vivante_drm_private *priv = gpu->dev->dev_private;
	queue_work(priv->wq, &gpu->retire_work);
}

/* add bo's to gpu's ring, and kick gpu: */
int msm_gpu_submit(struct vivante_gpu *gpu, struct msm_gem_submit *submit,
		struct vivante_file_private *ctx)
{
	struct drm_device *dev = gpu->dev;
	struct vivante_drm_private *priv = dev->dev_private;
	int i, ret;

	submit->fence = ++priv->next_fence;

	gpu->submitted_fence = submit->fence;

	ret = gpu->funcs->submit(gpu, submit, ctx);
	priv->lastctx = ctx;

	for (i = 0; i < submit->nr_bos; i++) {
		struct vivante_gem_object *msm_obj = submit->bos[i].obj;

		/* can't happen yet.. but when we add 2d support we'll have
		 * to deal w/ cross-ring synchronization:
		 */
		WARN_ON(is_active(msm_obj) && (msm_obj->gpu != gpu));

		if (!is_active(msm_obj)) {
			uint32_t iova;

			/* ring takes a reference to the bo and iova: */
			drm_gem_object_reference(&msm_obj->base);
            vivante_gem_get_iova_locked(&msm_obj->base, &iova);
		}

		if (submit->bos[i].flags & MSM_SUBMIT_BO_READ)
			msm_gem_move_to_active(&msm_obj->base, gpu, false, submit->fence);

		if (submit->bos[i].flags & MSM_SUBMIT_BO_WRITE)
			msm_gem_move_to_active(&msm_obj->base, gpu, true, submit->fence);
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
	return vivante_gpu_irq(gpu);
}

static const char *clk_names[] = {
		"gpu3d_core", "gpu3d_shader", "gpu3d_axi", "gpu2d_core", "gpu2d_axi", "openvg_axi",
};

int msm_gpu_init(struct drm_device *drm,struct vivante_gpu *gpu,
		const struct vivante_gpu_funcs *funcs, 	const char *name,
		const char *ioname, const char *irqname)
{
	struct platform_device *pdev = drm->platformdev;
	int i, ret;

	gpu->dev = drm;
	gpu->funcs = funcs;
	gpu->name = name;

	INIT_LIST_HEAD(&gpu->active_list);
	INIT_WORK(&gpu->retire_work, retire_worker);
	INIT_WORK(&gpu->recover_work, recover_worker);

	setup_timer(&gpu->hangcheck_timer, hangcheck_handler,
			(unsigned long)gpu);

	BUG_ON(ARRAY_SIZE(clk_names) != ARRAY_SIZE(gpu->grp_clks));

	/* Map registers: */
	gpu->mmio = vivante_ioremap(pdev, ioname, name);
	if (IS_ERR(gpu->mmio)) {
		ret = PTR_ERR(gpu->mmio);
		goto fail;
	}

	/* Get Interrupt: */
	gpu->irq = platform_get_irq_byname(pdev, irqname);
	if (gpu->irq < 0) {
		ret = gpu->irq;
		dev_err(drm->dev, "failed to get irq: %d\n", ret);
		goto fail;
	}

	ret = devm_request_irq(&pdev->dev, gpu->irq, irq_handler,
			IRQF_TRIGGER_HIGH, gpu->name, gpu);
	if (ret) {
		dev_err(drm->dev, "failed to request IRQ%u: %d\n", gpu->irq, ret);
		goto fail;
	}

	/* Acquire clocks: */
	for (i = 0; i < ARRAY_SIZE(clk_names); i++) {
		gpu->grp_clks[i] = devm_clk_get(&pdev->dev, clk_names[i]);
		DBG("grp_clks[%s]: %p", clk_names[i], gpu->grp_clks[i]);
		if (IS_ERR(gpu->grp_clks[i]))
			gpu->grp_clks[i] = NULL;
	}
#if 0
	gpu->ebi1_clk = devm_clk_get(&pdev->dev, "bus_clk");
	DBG("ebi1_clk: %p", gpu->ebi1_clk);
	if (IS_ERR(gpu->ebi1_clk))
		gpu->ebi1_clk = NULL;
#endif
	/* Acquire regulators: */
	gpu->gpu_reg = devm_regulator_get(&pdev->dev, "vdd");
	DBG("gpu_reg: %p", gpu->gpu_reg);
	if (IS_ERR(gpu->gpu_reg))
		gpu->gpu_reg = NULL;

	gpu->gpu_cx = devm_regulator_get(&pdev->dev, "vddcx");
	DBG("gpu_cx: %p", gpu->gpu_cx);
	if (IS_ERR(gpu->gpu_cx))
		gpu->gpu_cx = NULL;

	return 0;

fail:
	return ret;
}

void vivante_gpu_destroy(struct vivante_gpu *gpu)
{
	DBG("%s", gpu->name);

	WARN_ON(!list_empty(&gpu->active_list));

	if (gpu->rb) {
		if (gpu->rb_iova)
            vivante_gem_put_iova(gpu->rb->bo);
		vivante_ringbuffer_destroy(gpu->rb);
	}

	if (gpu->mmu)
		vivante_iommu_destroy(gpu->mmu);

	kfree(gpu);
	gpu = NULL;
}
