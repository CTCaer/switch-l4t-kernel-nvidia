/*
 * panel-s-720p-7-0.c: Panel driver for Samsung AMS699VC01 panel.
 *
 * Copyright (c) 2022, CTCaer.
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
#include <linux/interrupt.h>

#include "../dc.h"
#include "../dsi.h"
#include "board.h"
#include "board-panel.h"
#include "gpio-names.h"

/* DCS commands */
#define DCS_SET_BRIGHTNESS		0x51
#define DCS_PRIV_SM_SET_COLOR_MODE	0xA0

/* DCS command values */
#define DCS_SM_COLOR_MODE_BASIC		0x03
#define DCS_SM_COLOR_MODE_WASHED	0x45
#define DCS_SM_COLOR_MODE_NATURAL	0x23
#define DCS_SM_COLOR_MODE_VIVID		0x65
#define DCS_SM_COLOR_MODE_NIGHT0	0x43
#define DCS_SM_COLOR_MODE_NIGHT1	0x15
#define DCS_SM_COLOR_MODE_NIGHT2	0x35
#define DCS_SM_COLOR_MODE_NIGHT3	0x75
#define DCS_SM_COLOR_MODE_DEFAULT	DCS_SM_COLOR_MODE_NATURAL

/* Values used for the panel registers */
#define DCS_DEV_BRIGHTNESS_MIN		0
#define DCS_DEV_BRIGHTNESS_MAX		2047
#define DCS_DEV_BRIGHTNESS_CAL_MIN	1
#define DCS_DEV_BRIGHTNESS_CAL_MAX	2040

/* Values used for the driver. Use 0-255 for user space compatibility */
#define DCS_DEV_DUTY_DEFAULT		150
#define DCS_DEV_DUTY_MAX		255

#define DCS_DEV_BACKLIGHT_NAME		"backlight"

#define DSI_VIDEO_TYPE_VIDEO_MODE	0x1

static u8   color_mode = DCS_SM_COLOR_MODE_DEFAULT;
static bool color_mode_set = false;
static u16  gpio_panel_rst = -1;

static u8 brightness_command[] = { DCS_SET_BRIGHTNESS, 0x00, 0x00 };

static struct tegra_dsi_cmd brightness_cmd =
	DSI_CMD_LONG(DSI_DCS_LONG_WRITE, brightness_command);

static struct tegra_dsi_cmd color_mode_cmd =
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, DCS_PRIV_SM_SET_COLOR_MODE, 0x00);

static int dcs_send_cmd(struct tegra_dc *dc, struct tegra_dc_dsi_data *dsi,
			struct tegra_dsi_cmd *cmd)
{
	int err;

	if (dsi->status.vtype == DSI_VIDEO_TYPE_VIDEO_MODE) {
		/* Send command during vblank to avoid visible artifacts */
		err = tegra_dsi_start_host_cmd_v_blank_dcs(dsi, cmd);
		tegra_dsi_stop_host_cmd_v_blank_dcs(dsi);
	} else {
		/* Send command directly */
		err = tegra_dsi_write_data(dc, dsi, cmd, 0);
	}

	return err;
}

static int dcs_set_color_mode(struct tegra_dc *dc,
			      struct tegra_dc_dsi_data *dsi, u8 cmode)
{
	int err;

	color_mode_cmd.sp_len_dly.sp.data1 = cmode;

	err = dcs_send_cmd(dc, dsi, &color_mode_cmd);
	if (err < 0) {
		dev_err(&dc->ndev->dev,
			"dsi: error sending dsi color mode cmd\n");
		return err;
	}

	color_mode = cmode;

	return 0;
}

static int dcs_bl_update_status(struct backlight_device *bd)
{
	struct tegra_dc_dsi_data *dsi = bl_get_data(bd);
	struct tegra_dc *dc = dsi->dc;
	int reg_val;
	int err;

	if (!dsi->enabled) {
		dev_warn(&dc->ndev->dev, "dsi: can't set bl to %d, "
			"dsi interface is down.\n", bd->props.brightness);

		/* Report OK to userspace */
		return 0;
	}

	/* If brightness exceeds max, clamp it */
	if (bd->props.brightness > DCS_DEV_DUTY_MAX)
		bd->props.brightness = DCS_DEV_DUTY_MAX;

	/* Encode duty to panel brightness */
	reg_val = bd->props.brightness *
		  (DCS_DEV_BRIGHTNESS_MAX / DCS_DEV_DUTY_MAX);

	/* Clamp min/max */
	if (reg_val && reg_val < DCS_DEV_BRIGHTNESS_CAL_MIN)
		reg_val = DCS_DEV_BRIGHTNESS_MIN;
	if (reg_val >= DCS_DEV_BRIGHTNESS_CAL_MAX)
		reg_val = DCS_DEV_BRIGHTNESS_MAX;

	/* Set and send command */
	brightness_command[1] = reg_val >> 8;
	brightness_command[2] = reg_val & 0xFF;

	err = dcs_send_cmd(dc, dsi, &brightness_cmd);
	if (err < 0) {
		dev_err(&dc->ndev->dev,
			"dsi: error sending dsi backlight cmd\n");
		return err;
	}

	/* Restore color mode in case of panel init */
	if (!color_mode_set) {
		if (color_mode == DCS_SM_COLOR_MODE_DEFAULT) {
			color_mode_set = true;
			return 0;
		}

		err = dcs_set_color_mode(dc, dsi, color_mode);
		if (!err)
			color_mode_set = true;
	}

	return 0;
}

static int dcs_bl_check_fb(struct backlight_device *bd,
				      struct fb_info *info)
{
	struct platform_device *pdev;
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static const struct backlight_ops dcs_bl_ops = {
	.update_status = dcs_bl_update_status,
	.check_fb = dcs_bl_check_fb,
};

static struct backlight_device *create_dcs_backlight(
			struct tegra_dc_dsi_data *dsi, struct device *dev)
{
	struct backlight_properties props;

	/* Set backlight props  */
	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = DCS_DEV_DUTY_DEFAULT;
	props.max_brightness = DCS_DEV_DUTY_MAX;

	return backlight_device_register(DCS_DEV_BACKLIGHT_NAME, dev, dsi,
					 &dcs_bl_ops, &props);
}

static ssize_t dcs_cm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%02X\n", color_mode);
}

static ssize_t dcs_cm_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct tegra_dc *dc = dev_get_drvdata(dev);
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);

	unsigned long val;
	int err;

	if (!dsi->enabled) {
		dev_err(&dc->ndev->dev,
			"dsi: error setting color mode. interface is down.\n");
		return -EPERM;
	}

	err = kstrtoul(buf, 16, &val);
	if (err)
		return err;

	err = dcs_set_color_mode(dc, dsi, val);
	if (err)
		return err;

	return count;
}

static DEVICE_ATTR(panel_color_mode, 0664, dcs_cm_show, dcs_cm_store);

static struct attribute *dcs_cm_attributes[] = {
	&dev_attr_panel_color_mode.attr,
	NULL
};

static const struct attribute_group dcs_cm_attr_group = {
	.attrs = dcs_cm_attributes,
};

static irqreturn_t dsi_s_720p_7_0_irq_handler(int irq, void *dev_id)
{
	pr_debug("tegradc.0: dsi: Panel irq\n");

	return IRQ_HANDLED;
}


static int dsi_s_720p_7_0_postpoweron(struct device *dev)
{
	struct tegra_dc *dc = dev_get_drvdata(dev);
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);
	int err, irq;

	if (!dsi->info.bl) {
		dsi->info.bl = create_dcs_backlight(dsi, dev);
		if (IS_ERR(dsi->info.bl)) {
			err = PTR_ERR(dsi->info.bl);
			dsi->info.bl = NULL;
			dev_err(dev, "failed to register backlight %d\n", err);
			return err;
		}

		/* Inform dsi driver that backlight is controlled via dcs */
		dsi->info.dcs_controlled_bl = true;

		err = sysfs_create_group(&dev->kobj, &dcs_cm_attr_group);
		if (err) {
			dev_err(dev, "failed to register sysfs %d\n", err);
			return err;
		}

		/* Register panel irq handler */
		if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_IRQ])) {
			irq = gpio_to_irq(panel_of.panel_gpio[TEGRA_GPIO_PANEL_IRQ]);
			if (request_irq(irq, dsi_s_720p_7_0_irq_handler,
				IRQF_TRIGGER_RISING, "dsi-panel-irq", NULL)) {
				dev_err(dev, "Could not get panel irq\n");
			}
		}

	}

	return 0;
}

static int dsi_s_720p_7_0_enable(struct device *dev)
{
	int err = 0;

	err = tegra_panel_gpio_get_dt("s,720-1280-7-0", &panel_of);
	if (err < 0) {
		dev_err(dev, "dsi gpio request failed\n");
		return err;
	}

	/* If panel rst gpio is specified in device tree, use that. */
	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		gpio_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];
	else
		dev_warn(dev, "rst gpio is not defined in DT\n");

	/* Enable panel */
	if (!tegra_dc_bl_initialized(dev) && gpio_is_valid(gpio_panel_rst))
		gpio_direction_output(gpio_panel_rst, 1);

	return 0;
}

static int dsi_s_720p_7_0_disable(struct device *dev)
{
	/* Enable setting color mode on next enable. */
	color_mode_set = false;

	/* Disable panel */
	if (gpio_is_valid(gpio_panel_rst))
		gpio_set_value(gpio_panel_rst, 0);
	msleep(40);

	return 0;
}

struct tegra_panel_ops dsi_s_720p_7_0_ops = {
	.enable = dsi_s_720p_7_0_enable,
	.disable = dsi_s_720p_7_0_disable,
	.postpoweron = dsi_s_720p_7_0_postpoweron,
};
