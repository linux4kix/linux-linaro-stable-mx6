ccflags-y := -Iinclude/drm -Idrivers/staging/vivante
ifeq (, $(findstring -W,$(EXTRA_CFLAGS)))
	ccflags-y += -Werror
endif

vivante-y := \
	msm_drv.o \
	msm_gem.o \
	msm_gem_prime.o \
	msm_gem_submit.o \
	msm_gpu.o \
	msm_iommu.o \
	msm_ringbuffer.o \
	vivante_3d.c

obj-$(CONFIG_DRM_VIVANTE)	+= vivante.o
