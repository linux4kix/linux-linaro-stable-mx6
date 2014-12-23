/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef __LINUX_MXC_HDMI_CORE_H_
#define __LINUX_MXC_HDMI_CORE_H_

#include <video/mxc_edid.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IRQ_DISABLE_SUCCEED	0
#define IRQ_DISABLE_FAIL	1

bool hdmi_check_overflow(void);

u8 hdmi_readb(unsigned int reg);
void hdmi_writeb(u8 value, unsigned int reg);
void hdmi_mask_writeb(u8 data, unsigned int addr, u8 shift, u8 mask);
unsigned int hdmi_read4(unsigned int reg);
void hdmi_write4(unsigned int value, unsigned int reg);

void hdmi_irq_init(void);
void hdmi_irq_enable(int irq);
unsigned int hdmi_irq_disable(int irq);

void hdmi_set_sample_rate(unsigned int rate);
void hdmi_set_dma_mode(unsigned int dma_running);
void hdmi_init_clk_regenerator(void);
void hdmi_clk_regenerator_update_pixel_clock(u32 pixclock);

void hdmi_set_edid_cfg(int edid_status, struct mxc_edid_cfg *cfg);
int hdmi_get_edid_cfg(struct mxc_edid_cfg *cfg);

extern int mxc_hdmi_ipu_id;
extern int mxc_hdmi_disp_id;

void hdmi_set_registered(int registered);
int hdmi_get_registered(void);
int mxc_hdmi_abort_stream(void);
int mxc_hdmi_register_audio(struct snd_pcm_substream *substream);
void mxc_hdmi_unregister_audio(struct snd_pcm_substream *substream);
void hdmi_set_dvi_mode(unsigned int state);
unsigned int hdmi_set_cable_state(unsigned int state);
unsigned int hdmi_set_blank_state(unsigned int state);
int check_hdmi_state(void);

void hdmi_cec_start_device(void);
void hdmi_cec_stop_device(void);

#endif
