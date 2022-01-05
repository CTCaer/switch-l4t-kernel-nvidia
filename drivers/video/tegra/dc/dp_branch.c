/*
 * dp_branch.c: DP Branch device communication functions.
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

/* This is an initial work to support a single branch device (STDP2550) */

#include <dp.h>

#ifdef CONFIG_TEGRA_DP_BRANCH_STDP2550
static int dpb_stm_cec_read(struct tegra_dc_dp_data *dp, u8 *msg)
{
	u32 i;
	int ret = 0;
	u8 msg_len = 0;
	u8 dpcd_data = 0;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_STDP_CEC_RX_INFO,
				    &dpcd_data);
	if (ret) {
		dev_err(&dp->dc->ndev->dev,
			"dp: cec: rx packet info read error!\n");
		return ret;
	}

	/* Check if a rx error occured. Normally we should exit here
	 * and clear both HAS_DATA and PACKET_ERROR flags.
	 */
	if (dpcd_data & NV_DPCD_BRANCH_STDP_CEC_RX_INFO_PACKET_ERROR) {
		dpcd_data &= ~NV_DPCD_BRANCH_STDP_CEC_RX_INFO_PACKET_ERROR;
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_RX_INFO,
				       dpcd_data);
		dev_dbg(&dp->dc->ndev->dev, "dp: cec: rx packet error!\n");
	}

	/* Check if data is available */
	if (!(dpcd_data & NV_DPCD_BRANCH_STDP_CEC_RX_INFO_HAS_DATA))
		return 0;

	/* Copy over data */
	msg_len = (dpcd_data & NV_DPCD_BRANCH_STDP_CEC_RX_INFO_MSG_LENGTH_MASK);
	msg_len++;
	for (i = 0; i < msg_len; i++) {
		ret = tegra_dc_dp_dpcd_read(dp,
			NV_DPCD_BRANCH_STDP_CEC_RX_PACKET_BUFFER_BASE + i,
			&msg[i]);
		if (ret) {
			dev_err(&dp->dc->ndev->dev,
				"dp: cec: rx buffer copy failed!\n");
			return ret;
		}
	}

	/* Clear available data flag */
	dpcd_data &= ~NV_DPCD_BRANCH_STDP_CEC_RX_INFO_HAS_DATA;
	tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_RX_INFO,
			       dpcd_data);

	return msg_len;
}

static bool dpb_stm_cec_is_tx_done(struct tegra_dc_dp_data *dp)
{
	u8 dpcd_data = 0;

	if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_STDP_CEC_TX_CTRL,
			          &dpcd_data)) {
		return true;
	}

	if (!(dpcd_data & NV_DPCD_BRANCH_STDP_CEC_TX_CTRL_TRANSMIT))
		return true;
	else
		return false;
}

static int dpb_stm_cec_start_tx_and_wait(struct tegra_dc_dp_data *dp,
					      int msg_len)
{
	u32 tries = 50;
	u8 dpcd_data = 0;

	/* Set tx config */
	dpcd_data = (msg_len - 1);
	dpcd_data |= NV_DPCD_BRANCH_STDP_CEC_TX_CTRL_TRANSMIT;
	tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_TX_CTRL,
			       dpcd_data);

	/* Wait for transfer to complete */
	if (24 * msg_len > 50)
		msleep(24 * msg_len);
	else
		msleep(50);
	msleep(10);
	while (!dpb_stm_cec_is_tx_done(dp))
	{
		msleep(20);
		tries--;
		if (!tries)
			return -ETIME;
	}

	return 0;
}

static int dpb_stm_cec_write(struct tegra_dc_dp_data *dp,
				         u8 *msg, int msg_len)
{
	u32 i;
	int ret = 0;
	u8 dpcd_data = 0;
	u32 tx_base = NV_DPCD_BRANCH_STDP_CEC_TX_PACKET_BUFFER_BASE +
		      NV_DPCD_BRANCH_STDP_CEC_TX_PACKET_BUFFER_SIZE -
		      msg_len;

	if (msg_len > CEC_MAX_MSG_SIZE)
		return -EINVAL;

	/* Wait for previous transfer to finish */
	if (!dpb_stm_cec_is_tx_done(dp))
		return -EBUSY;

	/* Write data to buffer if any */
	for (i = 0; i < msg_len; i++)
	{
		ret = tegra_dc_dp_dpcd_write(dp, tx_base + i, msg[i]);
		if (ret) {
			dev_err(&dp->dc->ndev->dev,
				"dp: cec: tx buffer write error!\n");
			return -EIO;
		}
	}

	/* Start transfer and wait for completion */
	ret = dpb_stm_cec_start_tx_and_wait(dp, msg_len);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "dp: cec: tx timeout!\n");
		return ret;
	}

	/* Check if a tx error occured */
	tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_STDP_CEC_TX_CTRL,
			      &dpcd_data);
	if (dpcd_data & NV_DPCD_BRANCH_STDP_CEC_TX_CTRL_TRANSMIT_ERROR) {
		/* Clear error flag */
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_TX_CTRL,
			dpcd_data & ~NV_DPCD_BRANCH_STDP_CEC_TX_CTRL_TRANSMIT_ERROR);
		dev_dbg(&dp->dc->ndev->dev, "dp: cec: tx failed!\n");
		return -EIO;
	}

	return 0;
}

static int dpb_stm_cec_set_logical_address(struct tegra_dc_dp_data *dp,
					        u8 address)
{
	int ret = 0;
	u8 dpcd_data = 0;

	/* Exit if disconnected but return success to framework */
	if (!dp->dc->connected)
		return 0;

	if (address == CEC_LOG_ADDR_INVALID)
		address = 0;

	if (address > 15)
		return -EINVAL;

	/* Set logical address */
	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_STDP_CEC_CTRL,
				    &dpcd_data);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "dp: cec: failed to read config!\n");
		return ret;
	}
	dpcd_data &= ~NV_DPCD_BRANCH_STDP_CEC_CTRL_LOGICAL_ADDR_MASK;
	dpcd_data |= address;
	ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_CTRL,
				     dpcd_data);
	if (ret)
		dev_err(&dp->dc->ndev->dev, "dp: cec: failed to set la!\n");
	return ret;
}

static int dpb_stm_cec_enable(struct tegra_dc_dp_data *dp,
					  bool enable)
{
	u32 i;
	int ret = 0;
	u8 dpcd_data = 0;
	u8 branch_oui[3] = {0};

	/* Exit if disconnected but return success to framework */
	if (!dp->dc->connected)
		return 0;

	/* Check if supported device */
	for (i = 0; i < sizeof(branch_oui); i++) {
		ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_IEEE_OUI + i,
					    &branch_oui[i]);
		if (ret) {
			dev_err(&dp->dc->ndev->dev,
				"dp: cec: failed to read OUI!\n");
			return ret;
		}
	}

	/* Check if it's STMicroelectronics based */
	if (!(branch_oui[0] == 0x00 &&
	      branch_oui[1] == 0x80 &&
	      branch_oui[2] == 0xe1))
	{
		dev_warn(&dp->dc->ndev->dev,
			"dp: cec: not compatible oui (%02X-%02X-%02X)!\n",
			branch_oui[0], branch_oui[1], branch_oui[2]);
		return -ENODEV;
	}

	/* Clear registers */
	tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_RX_INFO, 0);
	tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_TX_CTRL, 0);
	for (i = 0; i < NV_DPCD_BRANCH_STDP_CEC_TX_PACKET_BUFFER_SIZE; i++) {
		tegra_dc_dp_dpcd_write(dp,
			NV_DPCD_BRANCH_STDP_CEC_TX_PACKET_BUFFER_BASE + i,
			0);
	}

	/* Enable/Disable STDP2550 CEC */
	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_BRANCH_STDP_CEC_CTRL,
				    &dpcd_data);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "dp: cec: failed to read config!\n");
		return ret;
	}
	if (enable)
		dpcd_data |= NV_DPCD_BRANCH_STDP_CEC_CTRL_ENABLE;
	else
		dpcd_data &= ~NV_DPCD_BRANCH_STDP_CEC_CTRL_ENABLE;
	ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_BRANCH_STDP_CEC_CTRL,
				     dpcd_data);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "dp: cec: failed to configure!\n");
	}

	return ret;
}

static int dpb_stm_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct tegra_dp_branch_data *data = adap->priv;
	int ret;

	ret = dpb_stm_cec_enable(data->dp, enable);

	/* Check if HPD is disconnected and return success to framework */
	if (ret && !READ_ONCE(data->dp->hpd_plug.done))
		ret = 0;

	return ret;
}

static int dpb_stm_cec_adap_log_addr(struct cec_adapter *adap,
				      u8 logical_addr)
{
	struct tegra_dp_branch_data *data = adap->priv;
	int ret;

	ret = dpb_stm_cec_set_logical_address(data->dp, logical_addr);

	/* Check if HPD is disconnected and return success to framework */
	if (ret && !READ_ONCE(data->dp->hpd_plug.done))
		ret = 0;

	return ret;
}

static int dpb_stm_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				  u32 signal_free_time, struct cec_msg *msg)
{
	struct tegra_dp_branch_data *data = adap->priv;

	if (!data->dp->dc->connected)
		return -ENODEV;

	data->cec_tx_attempts = attempts;
	data->cec_tx_len = msg->len;
	memcpy(data->cec_tx_buf, msg->msg, msg->len);

	schedule_work(&data->cec_tx_work);

	return 0;
}

static const struct cec_adap_ops dpb_stm_cec_ops = {
	.adap_enable = dpb_stm_cec_adap_enable,
	.adap_log_addr = dpb_stm_cec_adap_log_addr,
	.adap_transmit = dpb_stm_cec_adap_transmit,
};

static void dpb_stm_cec_tx_worker(struct work_struct *work)
{
	struct tegra_dp_branch_data *data = container_of(work,
						struct tegra_dp_branch_data,
						cec_tx_work);
	int ret = 0;
	int i;

	/* Try to transmit */
	for (i = 0; i < data->cec_tx_attempts; i++) {
		ret = dpb_stm_cec_write(data->dp,
					data->cec_tx_buf, data->cec_tx_len);
		if (!ret)
			break;
	}
	/* Notify framework of the result */
	if (ret) {
		cec_transmit_done(data->adap, CEC_TX_STATUS_NACK,
				  0, data->cec_tx_attempts, 0, 0);
	} else {
		cec_transmit_done(data->adap, CEC_TX_STATUS_OK,
				  0, 0, 0, 0);
	}
}

static void dpb_stm_cec_rx_worker(struct work_struct *work)
{
	struct tegra_dp_branch_data *data = container_of(work,
						struct tegra_dp_branch_data,
						cec_rx_work);
	struct cec_msg msg = {};
	int len;

	/* Read data */
	memset(data->cec_rx_buf, 0, CEC_MAX_MSG_SIZE);
	len = dpb_stm_cec_read(data->dp, data->cec_rx_buf);
	if (len <= 0)
		return;

	/* Send message to framework */
	msg.len = len;
	memcpy(msg.msg, data->cec_rx_buf, len);
	cec_received_msg(data->adap, &msg);
}
#endif

int tegra_dp_branch_notify_event(struct tegra_dc_dp_data *dp)
{
#ifdef CONFIG_TEGRA_DP_BRANCH_STDP2550
	struct tegra_dp_branch_data *data = &dp->branch_data;

	if (!data->branch_registered)
		return -ENODEV;

	schedule_work(&data->cec_rx_work);
#endif

	return 0;
}

void tegra_dp_branch_notify_edid_ready(struct tegra_dc_dp_data *dp)
{
#ifdef CONFIG_TEGRA_DP_BRANCH_STDP2550
	struct tegra_dp_branch_data *data = &dp->branch_data;
	u8 pa[2] = {0};

	if (!data->branch_registered)
		return;

	cec_s_phys_addr(data->adap, CEC_PHYS_ADDR_INVALID, false);
	if (!tegra_dc_get_source_physical_address(pa))
		cec_s_phys_addr(data->adap, pa[0] << 8 | pa[1], false);
#endif
}

int tegra_dp_branch_init(struct tegra_dp_branch_data *branch_data,
			 struct tegra_dc_dp_data *dp)
{
	int ret = 0;

	BUG_ON(!dp || !branch_data || !dp->dc);

	branch_data->dp = dp;

#ifdef CONFIG_TEGRA_DP_BRANCH_STDP2550
	branch_data->adap = cec_allocate_adapter(&dpb_stm_cec_ops,
				branch_data, STDP_CEC_NAME,
				CEC_CAP_LOG_ADDRS | CEC_CAP_TRANSMIT |
				CEC_CAP_PASSTHROUGH | CEC_CAP_RC,
				1, &dp->dc->ndev->dev);
	if (IS_ERR(branch_data->adap)) {
		dev_err(&dp->dc->ndev->dev, "Couldn't create cec adapter\n");
		return -ENOMEM;
	}

	ret = cec_register_adapter(branch_data->adap);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "Couldn't register device\n");
		cec_delete_adapter(branch_data->adap);
		return ret;
	}

	INIT_WORK(&branch_data->cec_rx_work, dpb_stm_cec_rx_worker);
	INIT_WORK(&branch_data->cec_tx_work, dpb_stm_cec_tx_worker);

	dev_info(&dp->dc->ndev->dev, "Branch device registered\n");
	branch_data->branch_registered = true;
#endif

	return ret;
}

void tegra_dp_branch_unregister(struct tegra_dc_dp_data *dp)
{
	struct tegra_dp_branch_data *data = &dp->branch_data;

	if (data->branch_registered) {
		cec_unregister_adapter(data->adap);
		cec_delete_adapter(data->adap);
	}

	data->branch_registered = false;
}
