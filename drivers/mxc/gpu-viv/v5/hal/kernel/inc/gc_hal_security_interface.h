/****************************************************************************
*
*    Copyright (C) 2005 - 2014 by Vivante Corp.
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


#ifndef _GC_HAL_SECURITY_INTERFACE_H_
#define _GC_HAL_SECURITY_INTERFACE_H_
/*!
 @brief Command codes between kernel module and TrustZone
 @discussion
 Critical services must be done in TrustZone to avoid sensitive content leak. Most of kernel module is kept in non-Secure os to minimize
 code in TrustZone.
 */
typedef enum kernel_packet_command {
    KERNEL_START_COMMAND,
    KERNEL_SUBMIT,
    KERNEL_MAP_MEMORY,                    /* */
    KERNEL_UNMAP_MEMORY,
    KERNEL_ALLOCATE_SECRUE_MEMORY,        /*! Security memory management. */
    KERNEL_FREE_SECURE_MEMORY,
    KERNEL_EXECUTE,                       /* Execute a command buffer. */
} kernel_packet_command_t;

/*!
 @brief gckCOMMAND Object requests TrustZone to start FE.
 @discussion
 DMA enabled register can only be written in TrustZone to avoid GPU from jumping to a hacked code.
 Kernel module need use these command to ask TrustZone start command parser.
 */
struct kernel_start_command {
    kernel_packet_command_t command;      /*! The command (always needs to be the first entry in a structure). */
    gctUINT8       gpu;                    /*! Which GPU. */
};

/*!
 @brief gckCOMMAND Object requests TrustZone to submit command buffer.
 @discussion
 Code in trustzone will check content of command buffer after copying command buffer to TrustZone.
 */
struct kernel_submit {
    kernel_packet_command_t command;      /*! The command (always needs to be the first entry in a structure). */
    gctUINT8       gpu;                    /*! Which GPU. */
    gctUINT8       kernel_command;         /*! Whether it is a kernel command. */
    gctUINT32      command_buffer_handle;  /*! Handle to command buffer. */
    gctUINT32      offset;                  /* Offset in command buffer. */
    gctUINT32 *    command_buffer;         /*! Content of command buffer need to be submit. */
    gctUINT32      command_buffer_length;  /*! Length of command buffer. */
};


/*!
 @brief gckVIDMEM Object requests TrustZone to allocate security memory.
 @discussion
 Allocate a buffer from security GPU memory.
 */
struct kernel_allocate_security_memory {
    kernel_packet_command_t command;      /*! The command (always needs to be the first entry in a structure). */
    gctUINT32      bytes;                  /*! Requested bytes. */
    gctUINT32      memory_handle;          /*! Handle of allocated memory. */
};

/*!
 @brief gckVIDMEM Object requests TrustZone to allocate security memory.
 @discussion
 Free a video memory buffer from security GPU memory.
 */
struct kernel_free_security_memory {
    kernel_packet_command_t command;      /*! The command (always needs to be the first entry in a structure). */
    gctUINT32      memory_handle;          /*! Handle of allocated memory. */
};

struct kernel_execute {
    kernel_packet_command_t command;      /*! The command (always needs to be the first entry in a structure). */
    gctUINT8       gpu;                    /*! Which GPU. */
    gctUINT8       kernel_command;         /*! Whether it is a kernel command. */
    gctUINT32 *    command_buffer;         /*! Content of command buffer need to be submit. */
    gctUINT32      command_buffer_length;  /*! Length of command buffer. */
};

typedef struct kernel_map_scatter_gather {
    gctUINT32      bytes;
    gctUINT32      physical;
    struct kernel_map_scatter_gather *next;
}
kernel_map_scatter_gather_t;

struct kernel_map_memory {
    kernel_packet_command_t command;
    kernel_map_scatter_gather_t *scatter;
    gctUINT32       *physicals;
    gctUINT32       pageCount;
    gctUINT32       gpuAddress;
};

struct kernel_unmap_memory {
    gctUINT32       gpuAddress;
    gctUINT32       pageCount;
};

typedef struct _gcsTA_INTERFACE {
    kernel_packet_command_t command;
    union {
        struct kernel_submit                   Submit;
        struct kernel_start_command            StartCommand;
        struct kernel_allocate_security_memory AllocateSecurityMemory;
        struct kernel_execute                  Execute;
        struct kernel_map_memory               MapMemory;
        struct kernel_unmap_memory             UnmapMemory;
    } u;
    gceSTATUS result;
} gcsTA_INTERFACE;

enum {
    gcvTA_COMMAND_INIT,
    gcvTA_COMMAND_DISPATCH,

    gcvTA_CALLBACK_ALLOC_SECURE_MEM,
    gcvTA_CALLBACK_FREE_SECURE_MEM,
};

#endif
