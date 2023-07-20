/*
 * @file ledtrig-ir.c
 * @author Joe
 * @brief
 * @version 0.1
 * @date 2023-07-17
 *
 * @copyright Copyright (c) 2023
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/extcon.h>
#include <linux/leds.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/leds.h>
#include <linux/string.h>

#define LED_TIME_SLEEP 100

enum WORK_DEFUALT_STATE{
	OFF = 0,
	ON,
	KEEP,
}DEFUALT_STATE;

struct notifier_block nb;
struct led_classdev *led_dev;
struct timer_list led_timer;

static int get_led_default_state(struct led_classdev *led_cdev){
	const char *default_state_value;
	int ret = -1;
	if (of_property_read_string(led_cdev->dev->of_node, "default-state", &default_state_value) == 0){
		if(!strcmp(default_state_value,"off"))
			DEFUALT_STATE = OFF;
		else if(!strcmp(default_state_value,"on"))
			DEFUALT_STATE = ON;
		else if(!strcmp(default_state_value,"keep"))
			DEFUALT_STATE = KEEP;
		return 0;
	}
	else {
		dev_info(led_cdev->dev,"Failed to read default-state property.\n");
		return ret;
	}
}

static void ir_blink_task(struct timer_list *t){
	(DEFUALT_STATE==OFF)?(led_set_brightness(led_dev, LED_OFF)) : led_set_brightness(led_dev, LED_ON);
}

static int ir_led_call_back(struct notifier_block *self, unsigned long event, void *ptr){
	if (event && DEFUALT_STATE != KEEP) {
        if (!timer_pending(&led_timer)) {
			(DEFUALT_STATE==OFF)?(led_set_brightness(led_dev, LED_ON)) : led_set_brightness(led_dev, LED_OFF);
            mod_timer(&led_timer, jiffies + msecs_to_jiffies(LED_TIME_SLEEP));
        }
	}
	return NOTIFY_DONE;
}

static int ir_trig_activate(struct led_classdev *led_cdev){
	static int ret;
	struct extcon_dev *edev;
	led_dev = led_cdev;

	if (!led_cdev->dev->of_node){
		dev_err(led_cdev->dev, "nedo not find\n");
		return -1;
	}
    if (of_property_read_bool(led_cdev->dev->of_node, "extcon")){
        edev = extcon_get_edev_by_phandle(led_cdev->dev, 0);
        if (IS_ERR(edev)) {
            dev_err(led_cdev->dev, "Invalid or missing extcon\n");
        }
		if (!IS_ERR(edev)){
			nb.notifier_call = ir_led_call_back;
			ret  = devm_extcon_register_notifier(led_cdev->dev, edev, EXTCON_IRINPUT, &nb);
			if(ret < 0 ){
				dev_err(led_cdev->dev, "register notifier fail\n");
				return ret;
			}
		}
	}
	timer_setup(&led_timer, ir_blink_task, 0);
	ret = get_led_default_state(led_cdev);
	if(ret < 0){
		dev_info(led_cdev->dev, "read led defualt state fail !!\n");
	}
	dev_info(led_cdev->dev, "defualt state : %d !!\n",DEFUALT_STATE);
	if(DEFUALT_STATE != KEEP )
		(DEFUALT_STATE==OFF)?(led_set_brightness(led_dev, LED_OFF)) : led_set_brightness(led_dev, LED_ON);
	return 0;
}

static void ir_trig_deactivate(struct led_classdev *led_cdev) {
	struct extcon_dev *edev = extcon_get_edev_by_phandle(led_cdev->dev, 0);
	devm_extcon_unregister_notifier(led_cdev->dev, edev, EXTCON_IRINPUT, &nb);
	del_timer_sync(&led_timer);
	led_set_brightness(led_cdev, LED_OFF);
}

static struct led_trigger IR_led_trigger = {
	.name = "ir0",
	.activate = ir_trig_activate,
	.deactivate = ir_trig_deactivate,
};

MODULE_AUTHOR("Joe <mengzhixuan@skysi.com.cn>");
module_led_trigger(IR_led_trigger);
MODULE_DESCRIPTION("IR LED trigger");
MODULE_LICENSE("GPL v2");
