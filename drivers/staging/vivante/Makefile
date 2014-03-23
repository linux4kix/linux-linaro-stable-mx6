ccflags-y := -Iinclude/drm -Idrivers/staging/vivante
ifeq (, $(findstring -W,$(EXTRA_CFLAGS)))
	ccflags-y += -Werror
endif

vivante-y := \
	adreno/adreno_gpu.o \
	msm_drv.o \
	msm_gem.o \
	msm_gem_prime.o \
	msm_gem_submit.o \
	msm_gpu.o \
	msm_iommu.o \
	msm_ringbuffer.o

obj-$(CONFIG_DRM_VIVANTE)	+= vivante.o
