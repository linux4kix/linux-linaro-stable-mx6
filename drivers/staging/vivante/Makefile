ccflags-y := -Iinclude/drm -Idrivers/staging/vivante
ifeq (, $(findstring -W,$(EXTRA_CFLAGS)))
	ccflags-y += -Werror
endif

vivante-y := \
	vivante_drv.o \
	vivante_gem.o \
	vivante_gem_prime.o \
	vivante_gem_submit.o \
	vivante_gpu.o \
	vivante_mmu.o \
	vivante_ringbuffer.o

obj-$(CONFIG_DRM_VIVANTE)	+= vivante.o
