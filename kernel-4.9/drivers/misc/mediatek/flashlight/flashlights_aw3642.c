/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"


/* device tree should be defined in flashlight-dt.h */
#ifndef AW3642_DTNAME
#define AW3642_DTNAME "mediatek,flashlights_aw3642"
#endif
#ifndef AW3642_DTNAME_I2C
#define AW3642_DTNAME_I2C "mediatek,strobe_main"
#endif

#define AW3642_NAME "flashlights_aw3642"
/* define registers */
#define AW3642_REG_ENABLE           (0x01)
#define AW3642_MASK_ENABLE_LED1     (0x01)
#define AW3642_DISABLE              (0x00)
#define AW3642_ENABLE_LED1          (0x03)
#define AW3642_ENABLE_LED1_TORCH    (0x0B)
#define AW3642_ENABLE_LED1_FLASH    (0x0F)

#define AW3642_REG_FLASH_LEVEL_LED1 (0x03)
#define AW3642_REG_TORCH_LEVEL_LED1 (0x05)


#define AW3642_REG_TIMING_CONF      (0x08)
#define AW3642_TORCH_RAMP_TIME      (0x10)
#define AW3642_FLASH_TIMEOUT        (0x0A)

/* define channel, level */
#define AW3642_CHANNEL_NUM          1
#define AW3642_CHANNEL_CH1          0

#define AW3642_LEVEL_NUM            26
#define AW3642_LEVEL_TORCH          7

/* define pinctrl */
#define AW3642_PINCTRL_PIN_HWEN 0
#define AW3642_PINCTRL_PINSTATE_LOW 0
#define AW3642_PINCTRL_PINSTATE_HIGH 1
#define AW3642_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define AW3642_PINCTRL_STATE_HWEN_LOW  "hwen_low"
static struct pinctrl *aw3642_pinctrl;
static struct pinctrl_state *aw3642_hwen_high;
static struct pinctrl_state *aw3642_hwen_low;

/* define mutex and work queue */
static DEFINE_MUTEX(aw3642_mutex);
static struct work_struct aw3642_work_ch1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw3642_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw3642_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(reg, 0660, aw3642_get_reg,  aw3642_set_reg);
/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *aw3642_i2c_client;

/* platform data */
struct aw3642_platform_data {
	u8 torch_pin_enable;         /* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;      /* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable; /* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;       /* 1: STROBE Input disabled */
	u8 vout_mode_enable;         /* 1: Voltage Out Mode enable */
};

/* aw3642 chip data */
struct aw3642_chip_data {
	struct i2c_client *client;
	struct aw3642_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};


/******************************************************************************
 * aw3642 operations
 *****************************************************************************/
static const unsigned char aw3642_torch_level[AW3642_LEVEL_NUM] = {
	0x83, 0x87, 0x8b, 0x8f, 0x93, 0x95, 0x97, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw3642_flash_level[AW3642_LEVEL_NUM] = {
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x88, 0x89, 0x8b,0x8d,
	0x8f, 0x91, 0x93, 0x94, 0x96, 0x99, 0x9a, 0x9c, 0x9e, 0xa0, 0xa2,
	0xa4, 0xa6, 0xa8, 0xaa,};

static volatile unsigned char aw3642_reg_enable;
static volatile int aw3642_level_ch1 = -1;

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int aw3642_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

    printk("aw3642_pinctrl_init start\n");
	/* get pinctrl */
	aw3642_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw3642_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(aw3642_pinctrl);
	}
	printk("devm_pinctrl_get finish %p\n", aw3642_pinctrl);

	/* Flashlight HWEN pin initialization */
	aw3642_hwen_high = pinctrl_lookup_state(aw3642_pinctrl, AW3642_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(aw3642_hwen_high)) {
		pr_err("Failed to init (%s)\n", AW3642_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(aw3642_hwen_high);
	}
	printk("pinctrl_lookup_state aw3642_hwen_high finish %p\n", aw3642_hwen_high);
	aw3642_hwen_low = pinctrl_lookup_state(aw3642_pinctrl, AW3642_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(aw3642_hwen_low)) {
		pr_err("Failed to init (%s)\n", AW3642_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(aw3642_hwen_low);
	}
	printk("pinctrl_lookup_state aw3642_hwen_low finish\n");

	return ret;
}

static int aw3642_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(aw3642_pinctrl)) {
		printk("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW3642_PINCTRL_PIN_HWEN:
		if (state == AW3642_PINCTRL_PINSTATE_LOW && !IS_ERR(aw3642_hwen_low))
			pinctrl_select_state(aw3642_pinctrl, aw3642_hwen_low);
		else if (state == AW3642_PINCTRL_PINSTATE_HIGH && !IS_ERR(aw3642_hwen_high))
			pinctrl_select_state(aw3642_pinctrl, aw3642_hwen_high);
		else
			printk("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		printk("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	printk("pin(%d) state(%d)\n", pin, state);

	return ret;
}

static int aw3642_is_torch(int level)
{
	if (level >= AW3642_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw3642_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW3642_LEVEL_NUM)
		level = AW3642_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int aw3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("AW3642_FLASH""failed writing at 0x%02x\n", reg);

	return ret;
}

static int aw3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		printk("AW3642_FLASH""failed read at 0x%02x\n", reg);

	return val;
}

/* flashlight enable function */
static int aw3642_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW3642_REG_ENABLE;
	if (!aw3642_is_torch(aw3642_level_ch1)) {
		/* torch mode */
		aw3642_reg_enable |= AW3642_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		aw3642_reg_enable |= AW3642_ENABLE_LED1_FLASH;
	}

	val = aw3642_reg_enable;

	return aw3642_write_reg(aw3642_i2c_client, reg, val);
}



/* flashlight disable function */
static int aw3642_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW3642_REG_ENABLE;
	aw3642_reg_enable &= (~AW3642_ENABLE_LED1_FLASH);
	val = aw3642_reg_enable;

	return aw3642_write_reg(aw3642_i2c_client, reg, val);
}

static int aw3642_enable(int channel)
{
	if (channel == AW3642_CHANNEL_CH1)
		aw3642_enable_ch1();
	else {
		printk("AW3642_FLASH""Error channel\n");
		return -1;
	}

	return 0;
}

static int aw3642_disable(int channel)
{
	if (channel == AW3642_CHANNEL_CH1)
		aw3642_disable_ch1();
	else {
		printk("AW3642_FLASH""Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int aw3642_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw3642_verify_level(level);

	/* set torch brightness level */
	reg = AW3642_REG_TORCH_LEVEL_LED1;
	val = aw3642_torch_level[level];
	ret = aw3642_write_reg(aw3642_i2c_client, reg, val);

	aw3642_level_ch1 = level;

	/* set flash brightness level */
	reg = AW3642_REG_FLASH_LEVEL_LED1;
	val = aw3642_flash_level[level];
	ret = aw3642_write_reg(aw3642_i2c_client, reg, val);

	return ret;
}



static int aw3642_set_level(int channel, int level)
{
	if (channel == AW3642_CHANNEL_CH1)
		aw3642_set_level_ch1(level);
	else {
		printk("AW3642_FLASH""Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int aw3642_init(void)
{
	int ret;
	unsigned char reg, val;
	aw3642_pinctrl_set(AW3642_PINCTRL_PIN_HWEN, AW3642_PINCTRL_PINSTATE_HIGH);

    msleep(2);

	/* clear enable register */
	reg = AW3642_REG_ENABLE;
	val = AW3642_DISABLE;
	ret = aw3642_write_reg(aw3642_i2c_client, reg, val);

	aw3642_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW3642_REG_TIMING_CONF;
	val = AW3642_TORCH_RAMP_TIME | AW3642_FLASH_TIMEOUT;
	ret = aw3642_write_reg(aw3642_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw3642_uninit(void)
{
	aw3642_disable(AW3642_CHANNEL_CH1);
	aw3642_pinctrl_set(AW3642_PINCTRL_PIN_HWEN, AW3642_PINCTRL_PINSTATE_LOW);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw3642_timer_ch1;
static unsigned int aw3642_timeout_ms[AW3642_CHANNEL_NUM];

static void aw3642_work_disable_ch1(struct work_struct *data)
{
	printk("AW3642_FLASH""ht work queue callback\n");
	aw3642_disable_ch1();
}

static enum hrtimer_restart aw3642_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw3642_work_ch1);
	return HRTIMER_NORESTART;
}


int aw3642_timer_start(int channel, ktime_t ktime)
{
	if (channel == AW3642_CHANNEL_CH1)
		hrtimer_start(&aw3642_timer_ch1, ktime, HRTIMER_MODE_REL);
	else {
		printk("AW3642_FLASH""Error channel\n");
		return -1;
	}

	return 0;
}

int aw3642_timer_cancel(int channel)
{
	if (channel == AW3642_CHANNEL_CH1)
		hrtimer_cancel(&aw3642_timer_ch1);
	else {
		printk("AW3642_FLASH""Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw3642_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= AW3642_CHANNEL_NUM) {
		printk("AW3642_FLASH""Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("AW3642_FLASH""FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3642_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("AW3642_FLASH""FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw3642_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("AW3642_FLASH""FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw3642_timeout_ms[channel]) {
				ktime = ktime_set(aw3642_timeout_ms[channel] / 1000,
						(aw3642_timeout_ms[channel] % 1000) * 1000000);
				aw3642_timer_start(channel, ktime);
			}
			aw3642_pinctrl_set(AW3642_PINCTRL_PIN_HWEN, AW3642_PINCTRL_PINSTATE_HIGH);
			aw3642_enable(channel);
		} else {
			aw3642_disable(channel);
			aw3642_pinctrl_set(AW3642_PINCTRL_PIN_HWEN, AW3642_PINCTRL_PINSTATE_LOW);
			aw3642_timer_cancel(channel);
		}
		break;

	default:
		printk("AW3642_FLASH""No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw3642_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int aw3642_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw3642_mutex);
	use_count--;
	if (!use_count)
		aw3642_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw3642_mutex);

	printk("AW3642_FLASH""Release: %d\n", use_count);

	return 0;
}

static int aw3642_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&aw3642_mutex);
	if (!use_count)
		aw3642_init();
	use_count++;
	mutex_unlock(&aw3642_mutex);

	printk("AW3642_FLASH""Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw3642_strobe_store(struct flashlight_arg arg)
{
	aw3642_set_driver(1);
	aw3642_set_level(arg.ct, arg.level);
	aw3642_enable(arg.ct);
	msleep(arg.dur);
	aw3642_disable(arg.ct);
	aw3642_set_driver(0);

	return 0;
}

static struct flashlight_operations aw3642_ops = {
	aw3642_open,
	aw3642_release,
	aw3642_ioctl,
	aw3642_strobe_store,
	aw3642_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw3642_chip_init(struct aw3642_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw3642_init();
	 */

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW3642 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw3642_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<0x0E;i++)
	{
		if(! (i==0x09 ||i==0x0A ))
		{
			reg_val = aw3642_read_reg(aw3642_i2c_client,i);
			len += snprintf(buf+len, PAGE_SIZE-len, "reg0x%2X = 0x%2X \n", i,reg_val);
		}
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw3642_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		//i2c_write_reg(databuf[0],databuf[1]);
		aw3642_write_reg(aw3642_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}



static int aw3642_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);

	return err;
}

static int aw3642_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw3642_chip_data *chip;
	struct aw3642_platform_data *pdata = client->dev.platform_data;
	int err;

	printk("AW3642_FLASH""Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("AW3642_FLASH""Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw3642_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		printk("AW3642_FLASH""Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct aw3642_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	aw3642_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw3642_work_ch1, aw3642_work_disable_ch1);

	/* init timer */
	hrtimer_init(&aw3642_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw3642_timer_ch1.function = aw3642_timer_func_ch1;
	aw3642_timeout_ms[AW3642_CHANNEL_CH1] = 100;

	/* init chip hw */
	aw3642_chip_init(chip);

	/* register flashlight operations */
	if (flashlight_dev_register(AW3642_NAME, &aw3642_ops)) {
		printk("AW3642_FLASH""Failed to register flashlight device.\n");
		err = -EFAULT;
		goto err_free;
	}

	/* clear usage count */
	use_count = 0;

	aw3642_create_sysfs(client);

	printk("AW3642_FLASH""Probe done.\n");

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int aw3642_i2c_remove(struct i2c_client *client)
{
	struct aw3642_chip_data *chip = i2c_get_clientdata(client);

	printk("AW3642_FLASH""Remove start.\n");

	/* flush work queue */
	flush_work(&aw3642_work_ch1);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW3642_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	printk("AW3642_FLASH""Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw3642_i2c_id[] = {
	{AW3642_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw3642_i2c_of_match[] = {
	{.compatible = AW3642_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw3642_i2c_driver = {
	.driver = {
		   .name = AW3642_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw3642_i2c_of_match,
#endif
		   },
	.probe = aw3642_i2c_probe,
	.remove = aw3642_i2c_remove,
	.id_table = aw3642_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw3642_probe(struct platform_device *dev)
{
	printk("AW3642_FLASH""Probe start.\n");

	if (aw3642_pinctrl_init(dev)) {
		printk("Failed to init pinctrl.\n");
		return -1;
	}
	if (i2c_add_driver(&aw3642_i2c_driver)) {
		printk("AW3642_FLASH""Failed to add i2c driver.\n");
		return -1;
	}

	printk("AW3642_FLASH""Probe done.\n");

	return 0;
}

static int aw3642_remove(struct platform_device *dev)
{
	printk("AW3642_FLASH""Remove start.\n");

	i2c_del_driver(&aw3642_i2c_driver);

	printk("AW3642_FLASH""Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3642_of_match[] = {
	{.compatible = AW3642_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw3642_of_match);
#else
static struct platform_device aw3642_platform_device[] = {
	{
		.name = AW3642_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3642_platform_device);
#endif

static struct platform_driver aw3642_platform_driver = {
	.probe = aw3642_probe,
	.remove = aw3642_remove,
	.driver = {
		.name = AW3642_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3642_of_match,
#endif
	},
};

static int __init flashlight_aw3642_init(void)
{
	int ret;

	printk("AW3642_FLASH""flashlight_aw3642-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3642_platform_device);
	if (ret) {
		printk("AW3642_FLASH""Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw3642_platform_driver);
	if (ret) {
		printk("AW3642_FLASH""Failed to register platform driver\n");
		return ret;
	}

	printk("AW3642_FLASH""flashlight_aw3642 Init done.\n");

	return 0;
}

static void __exit flashlight_aw3642_exit(void)
{
	printk("AW3642_FLASH""flashlight_aw3642-Exit start.\n");

	platform_driver_unregister(&aw3642_platform_driver);

	printk("AW3642_FLASH""flashlight_aw3642 Exit done.\n");
}

module_init(flashlight_aw3642_init);
module_exit(flashlight_aw3642_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight AW3642 Driver");

