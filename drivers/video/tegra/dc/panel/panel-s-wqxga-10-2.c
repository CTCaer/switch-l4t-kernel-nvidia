/*
 * panel-s-wqxga-10-2.c: Panel driver for s-wqxga-10-2 panel.
 *
 * Copyright (c) 2012-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include "../dc.h"
#include "board.h"
#include "board-panel.h"
#include "gpio-names.h"

#define DSI_PANEL_RESET		0

static bool reg_requested;
static struct regulator *ddi;
static struct regulator *power;
static u16 en_panel_en;
static u16 en_panel_rst;
static u16 ts_reset_gpio;

static int dalmore_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;
	power = regulator_get(dev, "power");
	if (IS_ERR(power)) {
		pr_err("power regulator get failed\n");
		err = PTR_ERR(power);
		power = NULL;
		goto fail;
	}

	ddi = regulator_get(dev, "ddi");
	if (IS_ERR(ddi)) {
		pr_err("ddi regulator get failed\n");
		err = PTR_ERR(ddi);
		ddi = NULL;
		goto fail;
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_s_wqxga_10_2_enable(struct device *dev)
{
	pr_info("dsi_s_wqxga_10_2_enable\n");
	return 0;
}


static int dsi_s_wqxga_10_2_postpoweron(struct device *dev)
{
	int err = 0;

	pr_info("dsi_s_wqxga_10_2_postpoweron\n");

	err = dalmore_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = tegra_panel_gpio_get_dt("s,wqxga-10-2", &panel_of);
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	/* If panel rst gpio is specified in device tree,
	 * use that.
	 */
	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];


	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN]))
		en_panel_en = panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN];

	if (power) {
		err = regulator_enable(power);
		if (err < 0) {
			pr_err("power regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(2000, 4000);

	if (ddi) {
		err = regulator_enable(ddi);
		if (err < 0) {
			pr_err("ddi regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(1000, 3000);

		/* use platform data */
	gpio_direction_output(en_panel_en, 1);
	gpio_direction_output(en_panel_rst, 1);
	gpio_set_value(en_panel_en, 1);
	usleep_range(10000, 15000);
	gpio_set_value(en_panel_rst, 1);
	usleep_range(3000, 5000);

	ts_reset_gpio = TEGRA_GPIO_PV6;
	gpio_set_value(ts_reset_gpio, 0);
	usleep_range(9000, 10000);
	gpio_set_value(ts_reset_gpio, 1);
	return 0;
fail:
	return err;
}

static int dsi_s_wqxga_10_2_disable(struct device *dev)
{
	pr_info("dsi_s_wqxga_10_2_disable\n");
	gpio_set_value(en_panel_rst, 0);

	usleep_range(1000, 3000);

	gpio_set_value(en_panel_en, 0);

	usleep_range(2000, 4000);

//	if (ddi)
	//	regulator_disable(ddi);

	usleep_range(5000, 6000);

//	if (power)
//		regulator_disable(power);

	return 0;
}

static int dsi_s_wqxga_10_2_postsuspend(void)
{
	pr_info("dsi_s_wqxga_10_2_postsuspend\n");
	return 0;
}

struct tegra_panel_ops dsi_s_wqxga_10_2_ops = {
	.enable = dsi_s_wqxga_10_2_enable,
	.disable = dsi_s_wqxga_10_2_disable,
	.postpoweron = dsi_s_wqxga_10_2_postpoweron,
	.postsuspend = dsi_s_wqxga_10_2_postsuspend,
};
