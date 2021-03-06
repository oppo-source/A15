/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#define pr_fmt(fmt) "[sar_modem] " fmt

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/pm_wakeup.h>

#include <hwmsensor.h>
#include <sensors_io.h>
#include "sar_modem.h"
#include "situation.h"

#include <hwmsen_helper.h>
#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"

static struct wakeup_source sar_modem_wake_lock;
static struct situation_init_info sar_modem_init_info;

static int sar_modem_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_SAR_MODEM, &data);
	if (err < 0) {
		pr_err_ratelimited("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp = data.time_stamp;
	*probability = data.sar_modem_event.state;
	pr_debug("recv ipi: timestamp: %lld, probability: %d!\n", time_stamp, *probability);
	return 0;
}

static int sar_modem_open_report_data(int open)
{
	int ret = 0;

	pr_debug("%s : enable=%d\n", __func__, open);
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_SAR_MODEM, 120);
#endif

	ret = sensor_enable_to_hub(ID_SAR_MODEM, open);
	return ret;
}

static int sar_modem_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	pr_debug("%s : flag=%d\n", __func__, flag);

	return sensor_batch_to_hub(ID_SAR_MODEM, flag, samplingPeriodNs, maxBatchReportLatencyNs);
}

static int sar_modem_flush(void)
{
	return sensor_flush_to_hub(ID_SAR_MODEM);
}

static int sar_modem_recv_data(struct data_unit_t *event, void *reserved)
{
	if (event->flush_action == FLUSH_ACTION)
		situation_flush_report(ID_SAR_MODEM);
	else if (event->flush_action == DATA_ACTION) {
		__pm_wakeup_event(&sar_modem_wake_lock, msecs_to_jiffies(100));
		situation_notify(ID_SAR_MODEM); // situation_data_report
		//situation_data_report(ID_SAR_MODEM, event->sar_modem_event.state);
	}
	return 0;
}

static int sar_modem_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	ctl.open_report_data = sar_modem_open_report_data;
	ctl.batch = sar_modem_batch;
	ctl.flush = sar_modem_flush;
	ctl.is_support_wake_lock = true;
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_SAR_MODEM);
	if (err) {
		pr_err_ratelimited("register sar_modem control path err\n");
		goto exit;
	}

	data.get_data = sar_modem_get_data;
	err = situation_register_data_path(&data, ID_SAR_MODEM);
	if (err) {
		pr_err_ratelimited("register sar_modem data path err\n");
		goto exit;
	}
	err = scp_sensorHub_data_registration(ID_SAR_MODEM, sar_modem_recv_data);
	if (err) {
		pr_err_ratelimited("SCP_sensorHub_data_registration fail!!\n");
		goto exit;
	}
	wakeup_source_init(&sar_modem_wake_lock, "sar_modem_wake_lock");

	return 0;
exit:
	return -1;
}
static int sar_modem_local_uninit(void)
{
	return 0;
}

static struct situation_init_info sar_modem_init_info = {
	.name = "sar_modem_hub",
	.init = sar_modem_local_init,
	.uninit = sar_modem_local_uninit,
};

static int __init sar_modem_init(void)
{
	situation_driver_add(&sar_modem_init_info, ID_SAR_MODEM);
	return 0;
}

static void __exit sar_modem_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(sar_modem_init);
module_exit(sar_modem_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SAR_MODEM driver");
MODULE_AUTHOR("benjamin.chao@mediatek.com");
