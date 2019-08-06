/**
 * @file drivers/media/platform/tegra/camera/fusa-capture/capture-common.c
 * @brief VI/ISP channel common operations for T186/T194
 *
 * Copyright (c) 2017-2019 NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/nospec.h>
#include <linux/nvhost.h>
#include <linux/slab.h>
#include <linux/hashtable.h>
#include <linux/atomic.h>
#include <media/fusa-capture/capture-common.h>
#include <media/mc_common.h>

#include "capture-common-priv.h"

struct capture_buffer_table *create_buffer_table(
	struct device *dev)
{
	struct capture_buffer_table *tab;

	tab = kmalloc(sizeof(*tab), GFP_KERNEL);

	if (likely(tab != NULL)) {
		tab->cache = KMEM_CACHE(capture_mapping, 0U);

		if (likely(tab->cache != NULL)) {
			tab->dev = dev;
			hash_init(tab->hhead);
			rwlock_init(&tab->hlock);
		} else {
			kfree(tab);
			tab = NULL;
		}
	}

	return tab;
}

void destroy_buffer_table(
	struct capture_buffer_table *tab)
{
	size_t bkt;
	struct hlist_node *next;
	struct capture_mapping *pin;

	write_lock(&tab->hlock);

	hash_for_each_safe(tab->hhead, bkt, next, pin, hnode) {
		hash_del(&pin->hnode);
		dma_buf_unmap_attachment(
			pin->atch, pin->sgt, flag_dma_direction(pin->flag));
		dma_buf_detach(pin->buf, pin->atch);
		dma_buf_put(pin->buf);
		kmem_cache_free(tab->cache, pin);
	}

	write_unlock(&tab->hlock);

	kmem_cache_destroy(tab->cache);
	kfree(tab);
}

inline bool flag_compatible(
	unsigned int self,
	unsigned int other)
{
	return (self & other) == other;
}

inline unsigned int flag_access_mode(
	unsigned int flag)
{
	return flag & BUFFER_RDWR;
}

inline enum dma_data_direction flag_dma_direction(
	unsigned int flag)
{
	static const enum dma_data_direction dir[4U] = {
		[0U] = DMA_BIDIRECTIONAL,
		[BUFFER_READ] = DMA_TO_DEVICE,
		[BUFFER_WRITE] = DMA_FROM_DEVICE,
		[BUFFER_RDWR] = DMA_BIDIRECTIONAL,
	};

	return dir[flag_access_mode(flag)];
}

inline dma_addr_t mapping_iova(
	const struct capture_mapping *pin)
{
	dma_addr_t addr = sg_dma_address(pin->sgt->sgl);

	return (addr != 0) ? addr : sg_phys(pin->sgt->sgl);
}

inline struct dma_buf *mapping_buf(
	const struct capture_mapping *pin)
{
	return pin->buf;
}

inline bool mapping_preserved(
	const struct capture_mapping *pin)
{
	return (bool)(pin->flag & BUFFER_ADD);
}

inline void set_mapping_preservation(
	struct capture_mapping *pin,
	bool val)
{
	if (val) {
		pin->flag |= BUFFER_ADD;
		atomic_inc(&pin->refcnt);
	} else {
		pin->flag &= (~BUFFER_ADD);
		atomic_dec(&pin->refcnt);
	}
}

struct capture_mapping *find_mapping(
	struct capture_buffer_table *tab,
	struct dma_buf *buf,
	unsigned int flag)
{
	struct capture_mapping *pin;
	bool success;

	read_lock(&tab->hlock);

	hash_for_each_possible(tab->hhead, pin, hnode, (unsigned long)buf) {
		if (
			(pin->buf == buf) &&
			flag_compatible(pin->flag, flag)
		) {
			success =  atomic_inc_not_zero(&pin->refcnt);
			if (success) {
				read_unlock(&tab->hlock);
				return pin;
			}
		}
	}

	read_unlock(&tab->hlock);

	return NULL;
}

struct capture_mapping *get_mapping(
	struct capture_buffer_table *tab,
	uint32_t fd,
	unsigned int flag)
{
	struct capture_mapping *pin;
	struct dma_buf *buf;
	void *err;

	buf = dma_buf_get((int)fd);
	if (IS_ERR(buf)) {
		dev_err(tab->dev, fmt("invalid memfd %u; errno %ld"),
			fd, PTR_ERR(buf));
		return ERR_CAST(buf);
	}

	pin = find_mapping(tab, buf, flag);
	if (pin != NULL) {
		dma_buf_put(buf);
		return pin;
	}

	pin = kmem_cache_alloc(tab->cache, GFP_KERNEL);
	if (unlikely(pin == NULL)) {
		err = ERR_PTR(-ENOMEM);
		goto err0;
	}

	pin->atch = dma_buf_attach(buf, tab->dev);
	if (unlikely(IS_ERR(pin->atch))) {
		err = pin->atch;
		goto err1;
	}

	pin->sgt = dma_buf_map_attachment(pin->atch, flag_dma_direction(flag));
	if (unlikely(IS_ERR(pin->sgt))) {
		err = pin->sgt;
		goto err2;
	}

	pin->flag = flag;
	pin->buf = buf;
	atomic_set(&pin->refcnt, 1U);
	INIT_HLIST_NODE(&pin->hnode);

	write_lock(&tab->hlock);
	hash_add(tab->hhead, &pin->hnode, (unsigned long)pin->buf);
	write_unlock(&tab->hlock);

	return pin;
err2:
	dma_buf_detach(buf, pin->atch);
err1:
	kmem_cache_free(tab->cache, pin);
err0:
	dma_buf_put(buf);
	dev_err(tab->dev, fmt("memfd %u, flag %u; errno %ld"),
		fd, flag, PTR_ERR(buf));
	return err;
}

void put_mapping(
	struct capture_buffer_table *t,
	struct capture_mapping *pin)
{
	bool zero;

	zero = atomic_dec_and_test(&pin->refcnt);
	if (zero) {
		if (unlikely(mapping_preserved(pin))) {
			dev_err(t->dev,
				fmt("unexpected put for a preserved mapping"));
			atomic_inc(&pin->refcnt);
			return;
		}

		write_lock(&t->hlock);
		hash_del(&pin->hnode);
		write_unlock(&t->hlock);

		dma_buf_unmap_attachment(
			pin->atch, pin->sgt, flag_dma_direction(pin->flag));
		dma_buf_detach(pin->buf, pin->atch);
		dma_buf_put(pin->buf);
		kmem_cache_free(t->cache, pin);
	}
}

static DEFINE_MUTEX(req_lock);

int capture_buffer_request(
	struct capture_buffer_table *tab,
	uint32_t memfd,
	uint32_t flag)
{
	struct capture_mapping *pin;
	struct dma_buf *buf;
	bool add = (bool)(flag & BUFFER_ADD);
	int err = 0;

	mutex_lock(&req_lock);

	if (add) {
		pin = get_mapping(tab, memfd, flag_access_mode(flag));
		if (IS_ERR(pin)) {
			err = PTR_ERR_OR_ZERO(pin);
			dev_err(tab->dev, fmt("memfd %u, flag %u; errno %d"),
				memfd, flag, err);
			goto end;
		}

		if (mapping_preserved(pin)) {
			err = -EEXIST;
			dev_err(tab->dev, fmt("memfd %u exists; errno %d"),
				memfd, err);
			put_mapping(tab, pin);
			goto end;
		}
	} else {
		buf = dma_buf_get((int)memfd);
		if (IS_ERR(buf)) {
			err = PTR_ERR_OR_ZERO(buf);
			dev_err(tab->dev, fmt("invalid memfd %u; errno %d"),
				memfd, err);
			goto end;
		}

		pin = find_mapping(tab, buf, BUFFER_ADD);
		if (pin == NULL) {
			err = -ENOENT;
			dev_err(tab->dev, fmt("memfd %u not exists; errno %d"),
				memfd, err);
			dma_buf_put(buf);
			goto end;
		}
		dma_buf_put(buf);
	}

	set_mapping_preservation(pin, add);
	put_mapping(tab, pin);

end:
	mutex_unlock(&req_lock);
	return err;
}

int capture_common_setup_progress_status_notifier(
	struct capture_common_status_notifier *status_notifier,
	uint32_t mem,
	uint32_t buffer_size,
	uint32_t mem_offset)
{
	struct dma_buf *dmabuf;
	void *va;

	/* take reference for the userctx */
	dmabuf = dma_buf_get(mem);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if ((buffer_size + mem_offset) > dmabuf->size) {
		dma_buf_put(dmabuf);
		pr_err("%s: invalid offset\n", __func__);
		return -EINVAL;
	}

	/* map handle and clear error notifier struct */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("%s: Cannot map notifier handle\n", __func__);
		return -ENOMEM;
	}

	memset(va, 0, buffer_size);

	status_notifier->buf = dmabuf;
	status_notifier->va = va;
	status_notifier->offset = mem_offset;
	return 0;
}

int capture_common_set_progress_status(
	struct capture_common_status_notifier *progress_status_notifier,
	uint32_t buffer_slot,
	uint32_t buffer_depth,
	uint8_t new_val)
{
	uint32_t *status_notifier = (uint32_t *) (progress_status_notifier->va +
			progress_status_notifier->offset);

	if (buffer_slot >= buffer_depth) {
		pr_err("%s: Invalid offset!", __func__);
		return -EINVAL;
	}
	buffer_slot = array_index_nospec(buffer_slot, buffer_depth);

	/*
	 * Since UMD and KMD can both write to the shared progress status
	 * notifier buffer, insert memory barrier here to ensure that any
	 * other store operations to the buffer would be done before the
	 * write below.
	 */
	wmb();

	status_notifier[buffer_slot] = new_val;

	return 0;
}

int capture_common_release_progress_status_notifier(
	struct capture_common_status_notifier *progress_status_notifier)
{
	struct dma_buf *dmabuf = progress_status_notifier->buf;
	void *va = progress_status_notifier->va;

	if (dmabuf != NULL) {
		if (va != NULL)
			dma_buf_vunmap(dmabuf, va);

		dma_buf_put(dmabuf);
	}

	progress_status_notifier->buf = NULL;
	progress_status_notifier->va = NULL;
	progress_status_notifier->offset = 0;

	return 0;
}

int capture_common_pin_memory(
	struct device *dev,
	uint32_t mem,
	struct capture_common_buf *unpin_data)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;

	buf = dma_buf_get(mem);
	if (IS_ERR(buf)) {
		err = PTR_ERR(buf);
		goto fail;
	}

	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		err = PTR_ERR(attach);
		goto fail;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = PTR_ERR(sgt);
		goto fail;
	}

	if (sg_dma_address(sgt->sgl) == 0)
		sg_dma_address(sgt->sgl) = sg_phys(sgt->sgl);

	unpin_data->iova = sg_dma_address(sgt->sgl);
	unpin_data->buf = buf;
	unpin_data->attach = attach;
	unpin_data->sgt = sgt;

	return 0;

fail:
	capture_common_unpin_memory(unpin_data);
	return err;
}

void capture_common_unpin_memory(
	struct capture_common_buf *unpin_data)
{
	if (unpin_data->sgt != NULL)
		dma_buf_unmap_attachment(unpin_data->attach, unpin_data->sgt,
				DMA_BIDIRECTIONAL);
	if (unpin_data->attach != NULL)
		dma_buf_detach(unpin_data->buf, unpin_data->attach);
	if (unpin_data->buf != NULL)
		dma_buf_put(unpin_data->buf);

	unpin_data->sgt = NULL;
	unpin_data->attach = NULL;
	unpin_data->buf = NULL;
	unpin_data->iova = 0;
}

int capture_common_request_pin_and_reloc(
	struct capture_common_pin_req *req)
{
	uint32_t *reloc_relatives;
	void *reloc_page_addr = NULL;
	int last_page = -1;
	int i;
	int err = 0;

	if (!req) {
		pr_err("%s: NULL pin request", __func__);
		return -EINVAL;
	}

	if (req->unpins) {
		dev_err(req->dev, "%s: request unpins already exist", __func__);
		return -EEXIST;
	}

	req->unpins = kzalloc(sizeof(struct capture_common_unpins) +
		(sizeof(req->unpins->data[0]) * req->num_relocs),
		GFP_KERNEL);
	if (unlikely(req->unpins == NULL)) {
		dev_err(req->dev, "failed to allocate request unpins\n");
		return -ENOMEM;
	}

	reloc_relatives = kcalloc(req->num_relocs, sizeof(uint32_t),
		GFP_KERNEL);
	if (unlikely(reloc_relatives == NULL)) {
		dev_err(req->dev, "failed to allocate request reloc array\n");
		err = -ENOMEM;
		goto reloc_fail;
	}

	err = copy_from_user(reloc_relatives, req->reloc_user,
			req->num_relocs * sizeof(uint32_t)) ? -EFAULT : 0;
	if (err < 0) {
		dev_err(req->dev, "failed to copy request user relocs\n");
		goto reloc_fail;
	}

	dev_dbg(req->dev, "%s: relocating %u addresses", __func__,
			req->num_relocs);

	for (i = 0; i < req->num_relocs; i++) {
		uint32_t reloc_relative = reloc_relatives[i];
		uint32_t reloc_offset = req->request_offset + reloc_relative;

		union capture_surface surf;
		struct capture_mapping *pin;
		dma_addr_t target_phys_addr = 0;

		dev_dbg(req->dev,
			"%s: idx:%i reloc:%u reloc_offset:%u", __func__,
			i, reloc_relative, reloc_offset);

		/* locate page of request in capture desc reloc is on */
		if (last_page != reloc_offset >> PAGE_SHIFT) {
			if (reloc_page_addr != NULL)
				dma_buf_kunmap(req->requests->buf,
					last_page, reloc_page_addr);

			reloc_page_addr = dma_buf_kmap(req->requests->buf,
						reloc_offset >> PAGE_SHIFT);
			last_page = reloc_offset >> PAGE_SHIFT;

			if (unlikely(reloc_page_addr == NULL)) {
				dev_err(req->dev,
					"%s: couldn't map request\n", __func__);
				err = -ENOMEM;
				goto pin_fail;
			}
		}

		/* read surf offset and mem handle from request descr */
		surf.raw = __raw_readq(
			(void __iomem *)(reloc_page_addr +
			(reloc_offset & ~PAGE_MASK)));

		dev_dbg(req->dev, "%s: hmem:0x%x offset:0x%x\n", __func__,
				surf.hmem, surf.offset);

		pin = get_mapping(req->table, surf.hmem, BUFFER_RDWR);
		if (IS_ERR(pin)) {
			err = PTR_ERR_OR_ZERO(pin);
			dev_info(req->dev,
				"%s: pin memory failed pin count %d\n",
				__func__, req->unpins->num_unpins);
			goto pin_fail;
		}

		target_phys_addr = (mapping_buf(pin) == req->requests_mem) ?
			mapping_iova(pin) + req->request_offset + surf.offset :
			mapping_iova(pin) + surf.offset;

		req->unpins->data[req->unpins->num_unpins++] = pin;

		if (!target_phys_addr) {
			dev_err(req->dev,
				"%s: target addr is NULL for mem 0x%x\n",
				__func__, surf.hmem);
			err = -EINVAL;
			goto pin_fail;
		}

		dev_dbg(req->dev,
			"%s: target addr 0x%llx at desc loc 0x%llx\n",
			__func__, (uint64_t)target_phys_addr,
			(uint64_t)reloc_page_addr +
			(reloc_offset & ~PAGE_MASK));

		/* write relocated physical address to request descr */
		__raw_writeq(
			target_phys_addr,
			(void __iomem *)(reloc_page_addr +
				(reloc_offset & ~PAGE_MASK)));

		dma_sync_single_range_for_device(req->rtcpu_dev,
				req->requests->iova, req->request_offset,
				req->request_size, DMA_TO_DEVICE);
	}
	speculation_barrier();

pin_fail:
	if (err) {
		for (i = 0; i < req->unpins->num_unpins; i++)
			put_mapping(req->table, req->unpins->data[i]);
	}

reloc_fail:
	if (err)
		kfree(req->unpins);

	if (reloc_page_addr != NULL)
		dma_buf_kunmap(req->requests->buf, last_page,
			reloc_page_addr);

	kfree(reloc_relatives);

	return err;
}