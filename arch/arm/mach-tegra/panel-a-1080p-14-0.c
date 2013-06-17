/*
 * arch/arm/mach-tegra/panel-a-1080p-14-0.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <mach/dc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/max8831_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include "board.h"
#include "board-panel.h"
#include "board-ardbeg.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define TEGRA_DSI_GANGED_MODE	0
#define DC_CTRL_MODE    TEGRA_DC_OUT_CONTINUOUS_MODE
#define DSI_PANEL_RESET		1

static bool reg_requested;
static bool gpio_requested;
static struct platform_device *disp_device;
static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_1v2_en;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *dvdd_lcd_1v8;
static struct regulator *vdd_ds_1v8;

static tegra_dc_bl_output dsi_a_1080p_14_0_bl_output_measured = {
	0, 0, 1, 2, 3, 4, 5, 6,
	7, 8, 9, 9, 10, 11, 12, 13,
	13, 14, 15, 16, 17, 17, 18, 19,
	20, 21, 22, 22, 23, 24, 25, 26,
	27, 27, 28, 29, 30, 31, 32, 32,
	33, 34, 35, 36, 37, 37, 38, 39,
	40, 41, 42, 42, 43, 44, 45, 46,
	47, 48, 48, 49, 50, 51, 52, 53,
	54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 68,
	69, 70, 71, 71, 72, 73, 74, 75,
	76, 77, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 87, 88, 89, 90, 91,
	92, 93, 94, 95, 96, 97, 98, 99,
	100, 101, 102, 103, 104, 105, 106, 107,
	108, 109, 110, 111, 112, 113, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124,
	125, 126, 127, 128, 129, 130, 131, 132,
	133, 134, 135, 136, 137, 138, 139, 141,
	142, 143, 144, 146, 147, 148, 149, 151,
	152, 153, 154, 155, 156, 157, 158, 158,
	159, 160, 161, 162, 163, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 176,
	177, 178, 179, 180, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 194,
	195, 196, 197, 198, 199, 200, 201, 202,
	203, 204, 205, 206, 207, 208, 209, 210,
	211, 212, 213, 214, 215, 216, 217, 219,
	220, 221, 222, 224, 225, 226, 227, 229,
	230, 231, 232, 233, 234, 235, 236, 238,
	239, 240, 241, 242, 243, 244, 245, 246,
	247, 248, 249, 250, 251, 252, 253, 255
};

static struct tegra_dsi_cmd dsi_a_1080p_14_0_init_cmd[] = {
	/* no init command required */
};

static struct tegra_dsi_out dsi_a_1080p_14_0_pdata = {
	.controller_vs = DSI_VS_1,
	.dsi2edp_bridge_enable = true,
	.n_data_lanes = 4,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,

	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 61,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = DSI_INSTANCE_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.dsi_init_cmd = dsi_a_1080p_14_0_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_a_1080p_14_0_init_cmd),
	.phy_timing = {
		.t_hsdexit_ns = 123,
		.t_hstrail_ns = 85,
		.t_datzero_ns = 170,
		.t_hsprepare_ns = 57,
	},
	.chip_id = 0x162,
	.chip_rev = 1,
};

static int laguna_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	dvdd_lcd_1v8 = regulator_get(dev, "dvdd_lcd");
		if (IS_ERR_OR_NULL(dvdd_lcd_1v8)) {
			pr_err("dvdd_lcd regulator get failed\n");
			err = PTR_ERR(dvdd_lcd_1v8);
			dvdd_lcd_1v8 = NULL;
			goto fail;
	}

	vdd_ds_1v8 = regulator_get(dev, "vdd_ds_1v8");
	if (IS_ERR_OR_NULL(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}

	avdd_lcd_3v3 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v3)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v3);
		avdd_lcd_3v3 = NULL;
		goto fail;
	}

	vdd_1v2_en = regulator_get(dev, "vdd_1v2_en");
	if (IS_ERR_OR_NULL(vdd_1v2_en)) {
		pr_err("vdd_1v2_en regulator get failed\n");
		err = PTR_ERR(vdd_1v2_en);
		vdd_1v2_en = NULL;
		goto fail;
	}


	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int laguna_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	err = gpio_request(DSI_PANEL_RST_GPIO, "panel rst");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}
	err = gpio_request(LCD_LR, "lcd_lr");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}

	err = gpio_request(LCD_RST_L, "lcd_rst_l");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}


	/* free pwm GPIO */
	err = gpio_request(LCD_TE, "lcd_te");
	if (err < 0) {
		pr_err("panel lcd te request failed\n");
		goto fail;
	}

	err = gpio_request(en_vdd_bl, "edp bridge 1v2 enable");
	if (err < 0) {
		pr_err("edp bridge 1v2 enable gpio request failed\n");
		goto fail;
	}

	err = gpio_request(lvds_en, "edp bridge 1v8 enable");
	if (err < 0) {
		pr_err("edp bridge 1v8 enable gpio request failed\n");
		goto fail;
	}

	err = gpio_request(refclk_en, "edp bridge refclk enable");
	if (err < 0) {
		pr_err("edp bridge refclk enable gpio request failed\n");
		goto fail;
	}

	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_a_1080p_14_0_enable(struct device *dev)
{
	int err = 0;
	err = laguna_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = laguna_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}
	if (avdd_lcd_3v3) {
		err = regulator_enable(avdd_lcd_3v3);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	/* enable 1.8v */
	if (vdd_ds_1v8) {
		err = regulator_enable(vdd_ds_1v8);
		if (err < 0) {
			pr_err("vdd_ds_1v8 regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_1v2_en) {
		err = regulator_enable(vdd_1v2_en);
		if (err < 0) {
			pr_err("vdd_1v2_en regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}
	gpio_direction_output(en_vdd_bl, 1);
	gpio_direction_output(DSI_PANEL_RST_GPIO, 0);
	usleep_range(10000, 12000);
	gpio_direction_output(lvds_en, 1);
	usleep_range(8000, 10000);
	gpio_direction_output(refclk_en, 1);
	usleep_range(5000, 8000);
	gpio_direction_output(DSI_PANEL_RST_GPIO, 1);
	return 0;
fail:
	return err;

}

static int dsi_a_1080p_14_0_disable(void)
{
	gpio_set_value(lvds_en, 0);
	gpio_set_value(refclk_en, 0);
	gpio_set_value(en_vdd_bl, 0);

	if (vdd_1v2_en)
		regulator_disable(vdd_1v2_en);

	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);

	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);
	return 0;
}

static int dsi_a_1080p_14_0_postsuspend(void)
{
	return 0;
}

static struct tegra_dc_mode dsi_a_1080p_14_0_modes[] = {
	{
		.pclk = 137986200,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 14,
		.h_back_porch = 152,
		.v_back_porch = 19,
		.h_active = 1920,
		.v_active = 1080,
		.h_front_porch = 16,
		.v_front_porch = 3,
	},
};

static int dsi_a_1080p_14_0_bl_notify(struct device *unused, int brightness)
{
	atomic_t __maybe_unused sd_brightness = ATOMIC_INIT(255);
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_a_1080p_14_0_bl_output_measured[brightness];

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	return brightness;
}

static int dsi_a_1080p_14_0_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static struct platform_pwm_backlight_data dsi_a_1080p_14_0_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_a_1080p_14_0_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_a_1080p_14_0_check_fb,
};

static struct platform_device __maybe_unused
		dsi_a_1080p_14_0_bl_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_a_1080p_14_0_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_a_1080p_14_0_bl_devices[] __initdata = {
	&tegra_pwfm_device,
	&dsi_a_1080p_14_0_bl_device,
};

static int  __init dsi_a_1080p_14_0_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(dsi_a_1080p_14_0_bl_devices,
				ARRAY_SIZE(dsi_a_1080p_14_0_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_a_1080p_14_0_set_disp_device(
	struct platform_device *laguna_display_device)
{
	disp_device = laguna_display_device;
}

static void dsi_a_1080p_14_0_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_a_1080p_14_0_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_a_1080p_14_0_modes;
	dc->n_modes = ARRAY_SIZE(dsi_a_1080p_14_0_modes);
	dc->enable = dsi_a_1080p_14_0_enable;
	dc->disable = dsi_a_1080p_14_0_disable;
	dc->postsuspend	= dsi_a_1080p_14_0_postsuspend,
	dc->width = 256;
	dc->height = 144;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_a_1080p_14_0_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_a_1080p_14_0_modes[0].h_active;
	fb->yres = dsi_a_1080p_14_0_modes[0].v_active;
}

static void
dsi_a_1080p_14_0_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}

static struct i2c_board_info laguna_sn65dsi86_dsi2edp_board_info __initdata = {
		I2C_BOARD_INFO("sn65dsi86_dsi2edp", 0x2d),
};

static int __init dsi_a_1080p_14_0_i2c_bridge_register(void)
{
	int err = 0;
	pr_info("dsi_a_1080p_14_0_i2c_bridge_register\n");
	err = i2c_register_board_info(0,
			&laguna_sn65dsi86_dsi2edp_board_info, 1);
	return err;
}
struct tegra_panel __initdata dsi_a_1080p_14_0 = {
	.init_sd_settings = dsi_a_1080p_14_0_sd_settings_init,
	.init_dc_out = dsi_a_1080p_14_0_dc_out_init,
	.init_fb_data = dsi_a_1080p_14_0_fb_data_init,
	.register_bl_dev = dsi_a_1080p_14_0_register_bl_dev,
	.register_i2c_bridge = dsi_a_1080p_14_0_i2c_bridge_register,
	.set_disp_device = dsi_a_1080p_14_0_set_disp_device,
};
EXPORT_SYMBOL(dsi_a_1080p_14_0);

