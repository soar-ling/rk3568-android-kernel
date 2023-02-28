/*
 * Linux driver for custom gpio control
 *
 * Copyright (C) 2022.9, Iflytek
 *
 * Author: <@iflytek.com>
 *
 */
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

//#include <linux/gpio.h>
//#include <linux/of.h>
//#include <linux/of_irq.h>
//#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/rk_keys.h>

#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

static struct gpio_desc *motor_en;        	

static struct pinctrl *motor_pin;
static struct pinctrl_state  *motor_pin_df;



/*----------------------------------------------------------------------------*/
static int  pwm_motor_control_probe(struct platform_device *pdev);
/*----------------------------------------------------------------------------*/
static const struct of_device_id pwm_motor_of_match[] = {
	{.compatible = "iflytek, pwm-motor-control",},
	{},
};
/*----------------------------------------------------------------------------*/
static struct platform_driver  pwm_motor_control_driver = {
	.probe	  =  pwm_motor_control_probe,
	.driver = {
		.name  = "pwm-motor-control",
		.owner = THIS_MODULE,
		.of_match_table = pwm_motor_of_match,
	}
};
/*----------------------------------------------------------------------------*/
static void set_gpio_status(struct gpio_desc *desc,int enable)
{
        if(!enable)
                gpiod_direction_output(desc, 0);
        else
                gpiod_direction_output(desc, 1);
}
/*----------------------------------------------------------------------------*/
static ssize_t  motor_en_show(struct device_driver *ddri, char *buf)
{

	int val;
			
	val = gpiod_get_value(motor_en);
	
	printk("motor_en val %d\n",val);
		  
	
        return sprintf(buf,"%d\n",val);
}
/*----------------------------------------------------------------------------*/
static ssize_t  motor_en_store(struct device_driver *ddri, const char *buf, size_t count)
{
        int enable;

        if(sscanf(buf, "%d", &enable) == 1)
                set_gpio_status(motor_en,enable);

        return count;
}



static DRIVER_ATTR_RW(motor_en);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *pwm_motor_control_attr_list[] = {
        &driver_attr_motor_en,
};


/*----------------------------------------------------------------------------*/
static int pwm_motor_control_create_attr(struct device_driver *driver) 
{
        int idx, err = 0;
        int num = (int)(sizeof(pwm_motor_control_attr_list)/sizeof(pwm_motor_control_attr_list[0]));

        if (driver == NULL)
        {
                printk("device driver is NULL\n");
                return -EINVAL;
        }

        for(idx = 0; idx < num; idx++)
        {
                err = driver_create_file(driver, pwm_motor_control_attr_list[idx]);
                if(err)
                {
                        printk("driver_create_file (%s) = %d\n", pwm_motor_control_attr_list[idx]->attr.name, err);
                        break;
                }
        }
        return err;
}
/*----------------------------------------------------------------------------*/
static int gpio_parse_dt(struct device *dev)
{
        int ret;
        
        motor_en = devm_gpiod_get_optional(dev, "motor-en", 0);
        if (IS_ERR(motor_en)) {
                ret = PTR_ERR(motor_en);
                goto exit;
        }

       gpiod_direction_input(motor_en);
        
        
        
        
        return 0;

exit:
        printk("%s error : %d\n",__func__,ret);
        return ret;
}




/*----------------------------------------------------------------------------*/
static int pwm_motor_control_probe(struct platform_device *pdev)
{
        int ret;    
  
        ret = gpio_parse_dt(&pdev->dev);
        if(ret) {
                printk("%s:parse gpio dt failed\n",__func__);
                return ret;
        }

         
         


        //motor velocity irq  
        motor_pin = devm_pinctrl_get(&pdev->dev);
        if (!IS_ERR(motor_pin)){
                motor_pin_df = pinctrl_lookup_state(motor_pin, "default");
                if (IS_ERR_OR_NULL(motor_pin_df)) {
			printk("no motor_pin_df pinctrl state\n");
			motor_pin_df = NULL;
		}
                else{
                        pinctrl_select_state(motor_pin, motor_pin_df);
			printk("motor_pin_df\n");
                }
        }


        //driver attr
        ret = pwm_motor_control_create_attr(&pwm_motor_control_driver.driver);
        if(ret) {
                printk("%s:create attr failed\n",__func__);
                return ret;
        }

	

        printk("%s success\n",__func__);
        return 0;
}
/*----------------------------------------------------------------------------*/

static int __init pwm_motor_control_init(void)
{
	return platform_driver_register(&pwm_motor_control_driver);
}

static void __exit pwm_motor_control_exit(void)
{
	platform_driver_unregister(&pwm_motor_control_driver);
}

/*----------------------------------------------------------------------------*/
module_init( pwm_motor_control_init);
module_exit( pwm_motor_control_exit);
/*----------------------------------------------------------------------------*/




