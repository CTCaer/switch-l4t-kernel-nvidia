/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of_address.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/tegra-soc.h>
#include "bpmp.h"
#include "mail_t186.h"

static struct ivc ivc_channels[NR_CHANNELS];
static void __iomem *cpu_ma_page;
static void __iomem *cpu_sl_page;

static int native_init_prepare(void)
{
	return tegra_hsp_init();
}

static void native_inbox_irq(int master, void *data)
{
	bpmp_handle_irq(BPMP_TO_CPU_CH);
}

static int native_init_irq(void)
{
	tegra_hsp_db_add_handler(HSP_MASTER_BPMP, native_inbox_irq, NULL);
	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);

	return 0;
}

static int native_iomem_init(void)
{
	struct device_node *of_node;

	/* FIXME: do not assume DT path */
	of_node = of_find_node_by_path("/bpmp");
	if (WARN_ON(!of_device_is_available(of_node)))
		return -ENODEV;

	cpu_ma_page = of_iomap(of_node, 1);
	if (!cpu_ma_page)
		return -ENODEV;

	cpu_sl_page = of_iomap(of_node, 2);
	if (!cpu_sl_page)
		return -ENODEV;

	return 0;
}

static int native_handshake(void)
{
	if (tegra_platform_is_linsim())
		return -ENODEV;

	pr_info("bpmp: waiting for handshake\n");
	while (!tegra_hsp_db_can_ring(HSP_DB_BPMP))
		;

	pr_info("bpmp: handshake completed\n");
	return 0;
}

static void native_resume(void)
{
	size_t sz;
	int i;

	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);

	pr_info("bpmp: waiting for handshake\n");
	while (!tegra_hsp_db_can_ring(HSP_DB_BPMP))
		;

	pr_info("bpmp: resuming channels\n");
	sz = tegra_ivc_total_queue_size(0);

	for (i = 0; i < NR_CHANNELS; i++)
		memset_io((void *)ivc_channels[i].tx_channel, 0, sz);
}

static void native_notify(struct ivc *ivc)
{
}

static int native_channel_init(int ch)
{
	struct ivc *ivc;
	uintptr_t rx_base;
	uintptr_t tx_base;
	size_t msg_sz;
	size_t que_sz;
	size_t hdr_sz;
	int r;

	msg_sz = tegra_ivc_align(MSG_SZ);
	hdr_sz = tegra_ivc_total_queue_size(0);
	que_sz = tegra_ivc_total_queue_size(msg_sz);

	rx_base = (uintptr_t)((uint8_t *)cpu_sl_page + ch * que_sz);
	tx_base = (uintptr_t)((uint8_t *)cpu_ma_page + ch * que_sz);

	/* init the channel frame */
	memset_io((void *)tx_base, 0, hdr_sz);

	ivc = ivc_channels + ch;
	r = tegra_ivc_init(ivc, rx_base, tx_base,
			1, msg_sz, device, native_notify);
	if (r) {
		pr_err("tegra_ivc_init() ch %d returned %d\n", ch, r);
		WARN_ON(1);
		return r;
	}

	return 0;
}

static void native_ring_doorbell(void)
{
	tegra_hsp_db_ring(HSP_DB_BPMP);
}

static struct ivc *native_ivc_obj(int ch)
{
	return ivc_channels + ch;
}

static struct mail_ops mail_ops = {
	.init_prepare = native_init_prepare,
	.ring_doorbell = native_ring_doorbell,
	.init_irq = native_init_irq,
	.iomem_init = native_iomem_init,
	.handshake = native_handshake,
	.channel_init = native_channel_init,
	.ivc_obj = native_ivc_obj,
	.resume = native_resume
};

struct mail_ops *native_mail_ops(void)
{
	/* FIXME: consider using an attr */
	const char *ofm_native = "nvidia,tegra186-bpmp";
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, ofm_native);
	if (!np)
		return NULL;

	of_node_put(np);
	return &mail_ops;
}
