/*
 * drivers/misc/gpio_hpdet.c
 *
 * Copyright (C) 2012-2016 Ubiqconn Co.,Ltd.
 * Author: Timmy Huang <timmy@ubiqconn.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>

#define MAXSIZE 10

struct gpio_hpdet {
	struct device *dev;
	bool current_status;

	u32 on_array[MAXSIZE];
	u32 off_array[MAXSIZE];
	u32 play_on_array[MAXSIZE];
	struct gpio_descs	*gpios_array;
	struct gpio_desc	*intgpio;
	struct delayed_work	work;
	int irq;
	bool invert;
	struct mutex lock;
	bool onplay;
};

struct gpio_hpdet *gt_cur_global = NULL;

static void gpios_array_work(struct work_struct *work)
{
	struct gpio_hpdet *gt_cur =
		container_of(work, struct gpio_hpdet, work.work);
	struct gpio_descs	*ga = gt_cur->gpios_array;
	int on = gt_cur->current_status;
	int i;
	
	mutex_lock(&gt_cur->lock);
	if (on){
		for (i=0; i<ga->ndescs; i++){
			mdelay(gt_cur->on_array[i]);
			if (gt_cur->play_on_array[i]){
				if (gt_cur->onplay)
					gpiod_set_value_cansleep(ga->desc[i], 1);
				else
					gpiod_set_value_cansleep(ga->desc[i], 0);
				
			}
			else{	
				gpiod_set_value_cansleep(ga->desc[i], 1);
			}
		}
	}	
	else{	
		for (i=0; i<ga->ndescs; i++){
			mdelay(gt_cur->off_array[i]);
			gpiod_set_value_cansleep(ga->desc[ga->ndescs - i - 1], 0);
		}	
	}
	mutex_unlock(&gt_cur->lock);
}

static irqreturn_t dummy_interrupt(int irq, void *data)
{
	struct gpio_hpdet *gt_cur = data;
	bool value;

	value = gpiod_get_value(gt_cur->intgpio)? true : false; 
//	if (gt_cur->invert)
//		value = !value;
	if (value !=  gt_cur->current_status){
		gt_cur->current_status = value;
		schedule_delayed_work(&gt_cur->work, msecs_to_jiffies(10));
	}	
	return IRQ_NONE;
}

static int gpio_hpdet_probe(struct platform_device *pdev)
{
	struct gpio_hpdet *gt_cur;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	int i;
	bool value;

	gt_cur = devm_kzalloc(&pdev->dev, sizeof(*gt_cur), GFP_KERNEL);
	if (!gt_cur)
		return -ENOMEM;

	gt_cur->dev = &pdev->dev;
	
	ret = of_property_read_u32_array(np, "on_timing", gt_cur->on_array,
                                                 ARRAY_SIZE(gt_cur->on_array));
	if (ret){
		for (i=0; i<ARRAY_SIZE(gt_cur->on_array); i++)
			gt_cur->on_array[i]=0;
	}	

	ret = of_property_read_u32_array(np, "off_timing", gt_cur->off_array,
                                                 2);
	if (ret){
		for (i=0; i<ARRAY_SIZE(gt_cur->off_array); i++){
			gt_cur->off_array[i]=0;
		}	
	}	

	ret = of_property_read_u32_array(np, "play_on", gt_cur->play_on_array,
                                                 2);
	if (ret){
		for (i=0; i<ARRAY_SIZE(gt_cur->play_on_array); i++){
			gt_cur->play_on_array[i]=0;
		}	
	}	

	gt_cur->gpios_array = devm_gpiod_get_array(gt_cur->dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(gt_cur->gpios_array) &&
	    PTR_ERR(gt_cur->gpios_array) != -ENOENT &&
	    PTR_ERR(gt_cur->gpios_array) != -ENOSYS) {
		ret = PTR_ERR(gt_cur->gpios_array);
		goto err_end;
	}
	
	gt_cur->invert = of_property_read_bool(np, "invert-interrupt");
	gt_cur_global = gt_cur;

	mutex_init(&gt_cur->lock);

	gt_cur->intgpio = devm_gpiod_get_optional(gt_cur->dev, "irqpin", GPIOD_IN);
	INIT_DELAYED_WORK(&gt_cur->work, gpios_array_work);
	gt_cur->irq = gpiod_to_irq(gt_cur->intgpio);

	ret = devm_request_irq(gt_cur->dev, gt_cur->irq, dummy_interrupt, 
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "gpio_hpdet_irq", gt_cur);
	if (ret) {
		dev_err(&pdev->dev,
			"request_irq failed with err %d\n",
			ret);
		goto err_end;
	}

	value = gpiod_get_value(gt_cur->intgpio)? true : false; 
	if (gt_cur->invert)
		value = !value;
	gt_cur->current_status = value;

	schedule_delayed_work(&gt_cur->work, msecs_to_jiffies(10));
	dev_info(gt_cur->dev, "gpio_hpdet_probe success\n");

err_end:
	return 0;
}

void digital_mute_notify(bool mute)
{
	struct gpio_hpdet *gt_cur = gt_cur_global;
	if (gt_cur){
		gt_cur->onplay = !mute;
		schedule_delayed_work(&gt_cur->work, msecs_to_jiffies(10));
	}
}	
EXPORT_SYMBOL(digital_mute_notify);

static const struct of_device_id gpio_hpdet_match[] = {
	{ .compatible = "gpio-hpdet" },
	{ /* Sentinel */ }
};

static int gpio_hpdet_remove(struct platform_device *pdev)
{
	struct gpio_hpdet *gt_cur = platform_get_drvdata(pdev);

	free_irq(gt_cur->irq, pdev);
	return 0;
}

static struct platform_driver gpio_hpdet_driver = {
	.probe = gpio_hpdet_probe,
	.remove = gpio_hpdet_remove,
	.driver = {
		.name = "gpio_hpdet",
		.owner = THIS_MODULE,
		.of_match_table	= gpio_hpdet_match,
	},
};

module_platform_driver(gpio_hpdet_driver);

MODULE_ALIAS("platform:gpio_hpdet");
MODULE_AUTHOR("Timmy Huang <timmy@ubiqconn.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("gpio trigger gpios driver");
