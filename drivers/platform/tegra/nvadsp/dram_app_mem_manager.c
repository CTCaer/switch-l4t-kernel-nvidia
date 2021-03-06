/*
 * dram_app_mem_manager.c
 *
 * dram app memory manager for allocating memory for text,bss and data
 *
 * Copyright (C) 2014-2018, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s : %d, " fmt, __func__, __LINE__

#include <linux/debugfs.h>
#include <linux/kernel.h>

#include "dram_app_mem_manager.h"

#define  ALIGN_TO_ADSP_PAGE(x)	ALIGN(x, 4096)

static void *dram_app_mem_handle;

static LIST_HEAD(dram_app_mem_alloc_list);
static LIST_HEAD(dram_app_mem_free_list);

void dram_app_mem_print(void)
{
	mem_print(dram_app_mem_handle);
}

void *dram_app_mem_request(const char *name, size_t size)
{
	return mem_request(dram_app_mem_handle, name, ALIGN_TO_ADSP_PAGE(size));
}

bool dram_app_mem_release(void *handle)
{
	return mem_release(dram_app_mem_handle, handle);
}

unsigned long dram_app_mem_get_address(void *handle)
{
	return mem_get_address(handle);
}

int dram_app_mem_init(unsigned long start, unsigned long size)
{
	dram_app_mem_handle =
		create_mem_manager("DRAM_APP_MANAGER", start, size);
	if (IS_ERR(dram_app_mem_handle)) {
		pr_err("ERROR: failed to create aram memory_manager");
		return PTR_ERR(dram_app_mem_handle);
	}

	return 0;
}

void dram_app_mem_exit(void)
{
	destroy_mem_manager(dram_app_mem_handle);
}

