/*
 * dp_branch.h: DP Branch device communication definitions.
 *
 * Copyright (c) 2021, CTCaer.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DP_BRANCH_H__
#define __DRIVERS_VIDEO_TEGRA_DC_DP_BRANCH_H__

#include <linux/types.h>

#include <media/cec.h>

#include "dc_priv.h"

#define STDP_CEC_NAME "stdp2550_cec"

struct tegra_dp_branch_data {
	struct tegra_dc_dp_data *dp;
	bool branch_registered;

	struct cec_adapter *adap;
	struct work_struct cec_rx_work;
	struct work_struct cec_tx_work;
	u8 cec_rx_buf[CEC_MAX_MSG_SIZE];
	u8 cec_tx_buf[CEC_MAX_MSG_SIZE];
	u8 cec_tx_attempts;
	int cec_tx_len;
};

int tegra_dp_branch_init(struct tegra_dp_branch_data *branch_data,
			 struct tegra_dc_dp_data *dp);
void tegra_dp_branch_unregister(struct tegra_dc_dp_data *dp);
int tegra_dp_branch_notify_event(struct tegra_dc_dp_data *dp);
void tegra_dp_branch_notify_edid_ready(struct tegra_dc_dp_data *dp);


#endif
