/*
 * drivers/char/hw_random/tegra-trng.c
 *
 * hwrng dev node for NVIDIA tegra prng hardware
 *
 * Copyright (c) 2021, NVIDIA Corporation. All Rights Reserved.
 * Copyright (c) 2023, CTCaer.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <crypto/rng.h>

#define MODULE_NAME "tegra-trng"
#define RETRY_TIMEOUT_SECS	2

static int tegra_trng_read(struct hwrng *h, void *data, size_t max, bool wait)
{
	int ret;
	struct crypto_rng *trng;
	unsigned long t;

	if (!wait)
		return 0;

	trng = crypto_alloc_rng("trng_elp-tegra",
			CRYPTO_ALG_TYPE_RNG, 0);
	if (IS_ERR(trng)) {
		pr_err("crypto_alloc_rng(trng_elp-tegra) failed: %ld\n",
				PTR_ERR(trng));
		return PTR_ERR(trng);
	}
	t = get_seconds();
	do {
		ret = crypto_rng_get_bytes(trng, data, max);
		if (ret != -EAGAIN)
			break;
		msleep_interruptible(20);
	} while ((get_seconds() - t) <= RETRY_TIMEOUT_SECS);

	/* crypto_rng_get_bytes returns 0 upon success */
	if (ret == 0)
		ret = max;

	crypto_free_rng(trng);
	return ret;
}

static struct hwrng tegra_trng = {
	.name = MODULE_NAME,
	.read = tegra_trng_read,
};

static int __init tegra_trng_init(void)
{
	return hwrng_register(&tegra_trng);
}
device_initcall_sync(tegra_trng_init);

static void __exit tegra_trng_exit(void)
{
	hwrng_unregister(&tegra_trng);
}
module_exit(tegra_trng_exit);

MODULE_DESCRIPTION("TRNG driver for Tegra devices");
MODULE_AUTHOR("CTCaer <ctcaer@gmail.com>");
MODULE_LICENSE("GPL v2");
