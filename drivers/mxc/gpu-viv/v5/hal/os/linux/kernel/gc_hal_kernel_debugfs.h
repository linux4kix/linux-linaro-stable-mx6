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


#include <stdarg.h>

#ifndef __gc_hal_kernel_debugfs_h_
#define __gc_hal_kernel_debugfs_h_

 #define MAX_LINE_SIZE 768           /* Max bytes for a line of debug info */


 typedef struct _gcsDEBUGFS_Node gcsDEBUGFS_Node;

typedef struct _gcsDEBUGFS_DIR *gckDEBUGFS_DIR;
typedef struct _gcsDEBUGFS_DIR
{
    struct dentry *     root;
    struct list_head    nodeList;
}
gcsDEBUGFS_DIR;

typedef struct _gcsINFO
{
    const char *        name;
    int                 (*show)(struct seq_file*, void*);
}
gcsINFO;

typedef struct _gcsINFO_NODE
{
    gcsINFO *          info;
    gctPOINTER         device;
    struct dentry *    entry;
    struct list_head   head;
}
gcsINFO_NODE;

gceSTATUS
gckDEBUGFS_DIR_Init(
    IN gckDEBUGFS_DIR Dir,
    IN struct dentry *root,
    IN gctCONST_STRING Name
    );

gceSTATUS
gckDEBUGFS_DIR_CreateFiles(
    IN gckDEBUGFS_DIR Dir,
    IN gcsINFO * List,
    IN int count,
    IN gctPOINTER Data
    );

gceSTATUS
gckDEBUGFS_DIR_RemoveFiles(
    IN gckDEBUGFS_DIR Dir,
    IN gcsINFO * List,
    IN int count
    );

void
gckDEBUGFS_DIR_Deinit(
    IN gckDEBUGFS_DIR Dir
    );

/*******************************************************************************
 **
 **                             System Related
 **
 *******************************************************************************/

gctINT gckDEBUGFS_IsEnabled(void);

gctINT gckDEBUGFS_Initialize(void);

gctINT gckDEBUGFS_Terminate(void);


/*******************************************************************************
 **
 **                             Node Related
 **
 *******************************************************************************/

gctINT
gckDEBUGFS_CreateNode(
    IN gctPOINTER Device,
    IN gctINT SizeInKB,
    IN struct dentry * Root,
    IN gctCONST_STRING NodeName,
    OUT gcsDEBUGFS_Node **Node
    );

void gckDEBUGFS_FreeNode(
            IN gcsDEBUGFS_Node  * Node
            );



void gckDEBUGFS_SetCurrentNode(
            IN gcsDEBUGFS_Node  * Node
            );



void gckDEBUGFS_GetCurrentNode(
            OUT gcsDEBUGFS_Node  ** Node
            );


ssize_t gckDEBUGFS_Print(
                IN gctCONST_STRING  Message,
                ...
                );

#endif


