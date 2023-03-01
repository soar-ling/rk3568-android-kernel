
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

#include <linux/of_platform.h>
#include <linux/rk_keys.h>

#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#define LENGHT_GPIO  72
static struct gpio_desc *tsgpio_en[LENGHT_GPIO];        	

static struct pinctrl *motor_pin;
static struct pinctrl_state  *motor_pin_df;


static int  tsgpio_control_probe(struct platform_device *pdev);

static const struct of_device_id tsgpio_of_match[] = {
	{.compatible = "skysi, ts-gpio-control",},
	{},
};

static struct platform_driver  tsgpio_control_driver = {
	.probe	  =  tsgpio_control_probe,
	.driver = {
		.name  = "ts-gpio-control",
		.owner = THIS_MODULE,
		.of_match_table = tsgpio_of_match,
	}
};
/*
static void set_gpio_status(struct gpio_desc *desc,int enable)
{
        if(!enable)
                gpiod_direction_output(desc, 0);
        else
                gpiod_direction_output(desc, 1);
}
*/
static ssize_t  tsgpio_show(struct device_driver *ddri, char *buf)
{

	int val[LENGHT_GPIO];
        int value;
        int i;
	for(i=0;i<LENGHT_GPIO;i++){
          val[i] = gpiod_get_value(tsgpio_en[i]);
        }
        

	value = val[0] && val[1] && val[2] && val[3] && val[4] && val[5] && val[6] && val[7] && val[8] && val[9]
         && val[10] && val[11] && val[12] && val[13] && val[14] && val[15] && val[16] && val[17] && val[18]
        && val[19] && val[20]&& val[21] && val[22] && val[23] && val[24]
        && val[25] && val[26] && val[27] && val[28] && val[29] && val[30] && val[31] && val[32] && val[33] && val[34]
        && val[35] && val[36] && val[37] && val[38] && val[39] && val[40] && val[41] && val[42]
        && val[43] && val[44] && val[45] && val[46] && val[47] && val[48] 
        && val[49] && val[50] && val[51] && val[52] && val[53] && val[54] && val[55]
        && val[56] && val[57] && val[58] && val[59] && val[60] && val[61] && val[62] && val[63] && val[64]
        && val[65] && val[66] && val[67] && val[68] && val[69] && val[70] && val[71];
        
         
        
	printk("motor_en val %d\n",value);
		  
	
        return sprintf(buf,"%d\n",value);
}
/*----------------------------------------------------------------------------*/
static ssize_t  tsgpio_store(struct device_driver *ddri, const char *buf, size_t count)
{
      //  int enable;

      //  if(sscanf(buf, "%d", &enable) == 1)
       //         set_gpio_status(motor_en,enable);

        return count;
}



static DRIVER_ATTR_RW(tsgpio);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *tsgpio_control_attr_list[] = {
        &driver_attr_tsgpio,
};


/*----------------------------------------------------------------------------*/
static int tsgpio_control_create_attr(struct device_driver *driver) 
{
        int idx, err = 0;
        int num = (int)(sizeof(tsgpio_control_attr_list)/sizeof(tsgpio_control_attr_list[0]));

        if (driver == NULL)
        {
                printk("device driver is NULL\n");
                return -EINVAL;
        }

        for(idx = 0; idx < num; idx++)
        {
                err = driver_create_file(driver, tsgpio_control_attr_list[idx]);
                if(err)
                {
                        printk("driver_create_file (%s) = %d\n", tsgpio_control_attr_list[idx]->attr.name, err);
                        break;
                }
        }
        return err;
}
/*----------------------------------------------------------------------------*/
static int gpio_parse_dt(struct device *dev)
{
       

        int i;
        tsgpio_en[0] = devm_gpiod_get_optional(dev, "tsgpio1-en", 0);
        tsgpio_en[1] = devm_gpiod_get_optional(dev, "tsgpio2-en", 0);
        tsgpio_en[2] = devm_gpiod_get_optional(dev, "tsgpio3-en", 0);
        tsgpio_en[3] = devm_gpiod_get_optional(dev, "tsgpio4-en", 0);
        tsgpio_en[4] = devm_gpiod_get_optional(dev, "tsgpio5-en", 0);
        tsgpio_en[5] = devm_gpiod_get_optional(dev, "tsgpio6-en", 0);
        tsgpio_en[6] = devm_gpiod_get_optional(dev, "tsgpio7-en", 0);
        tsgpio_en[7] = devm_gpiod_get_optional(dev, "tsgpio8-en", 0);
        tsgpio_en[8] = devm_gpiod_get_optional(dev, "tsgpio9-en", 0);
        tsgpio_en[9] = devm_gpiod_get_optional(dev, "tsgpio10-en", 0);
        tsgpio_en[10] = devm_gpiod_get_optional(dev, "tsgpio11-en", 0);
        tsgpio_en[11] = devm_gpiod_get_optional(dev, "tsgpio12-en", 0);
        tsgpio_en[12] = devm_gpiod_get_optional(dev, "tsgpio13-en", 0);
        tsgpio_en[13] = devm_gpiod_get_optional(dev, "tsgpio14-en", 0);
        tsgpio_en[14] = devm_gpiod_get_optional(dev, "tsgpio15-en", 0);
        tsgpio_en[15] = devm_gpiod_get_optional(dev, "tsgpio16-en", 0);
        tsgpio_en[16] = devm_gpiod_get_optional(dev, "tsgpio17-en", 0);
        tsgpio_en[17] = devm_gpiod_get_optional(dev, "tsgpio18-en", 0);
        tsgpio_en[18] = devm_gpiod_get_optional(dev, "tsgpio19-en", 0);
        tsgpio_en[19] = devm_gpiod_get_optional(dev, "tsgpio20-en", 0);
        tsgpio_en[20] = devm_gpiod_get_optional(dev, "tsgpio21-en", 0);
        tsgpio_en[21] = devm_gpiod_get_optional(dev, "tsgpio22-en", 0);
        tsgpio_en[22] = devm_gpiod_get_optional(dev, "tsgpio23-en", 0);
        tsgpio_en[23] = devm_gpiod_get_optional(dev, "tsgpio24-en", 0);
        tsgpio_en[24] = devm_gpiod_get_optional(dev, "tsgpio25-en", 0);
        tsgpio_en[25] = devm_gpiod_get_optional(dev, "tsgpio26-en", 0);
        tsgpio_en[26] = devm_gpiod_get_optional(dev, "tsgpio27-en", 0);
        tsgpio_en[27] = devm_gpiod_get_optional(dev, "tsgpio28-en", 0);
        tsgpio_en[28] = devm_gpiod_get_optional(dev, "tsgpio29-en", 0);
        tsgpio_en[29] = devm_gpiod_get_optional(dev, "tsgpio30-en", 0);
        tsgpio_en[30] = devm_gpiod_get_optional(dev, "tsgpio31-en", 0);
        tsgpio_en[31] = devm_gpiod_get_optional(dev, "tsgpio32-en", 0);
        tsgpio_en[32] = devm_gpiod_get_optional(dev, "tsgpio33-en", 0);
        tsgpio_en[33] = devm_gpiod_get_optional(dev, "tsgpio34-en", 0);
        tsgpio_en[34] = devm_gpiod_get_optional(dev, "tsgpio35-en", 0);
        tsgpio_en[35] = devm_gpiod_get_optional(dev, "tsgpio36-en", 0);
        tsgpio_en[36] = devm_gpiod_get_optional(dev, "tsgpio37-en", 0);
        tsgpio_en[37] = devm_gpiod_get_optional(dev, "tsgpio38-en", 0);
        tsgpio_en[38] = devm_gpiod_get_optional(dev, "tsgpio39-en", 0);
        tsgpio_en[39] = devm_gpiod_get_optional(dev, "tsgpio40-en", 0);
        tsgpio_en[40] = devm_gpiod_get_optional(dev, "tsgpio41-en", 0);
        tsgpio_en[41] = devm_gpiod_get_optional(dev, "tsgpio42-en", 0);
        tsgpio_en[42] = devm_gpiod_get_optional(dev, "tsgpio43-en", 0);
        tsgpio_en[43] = devm_gpiod_get_optional(dev, "tsgpio44-en", 0);
        tsgpio_en[44] = devm_gpiod_get_optional(dev, "tsgpio45-en", 0);
        tsgpio_en[45] = devm_gpiod_get_optional(dev, "tsgpio46-en", 0);
        tsgpio_en[46] = devm_gpiod_get_optional(dev, "tsgpio47-en", 0);
        tsgpio_en[47] = devm_gpiod_get_optional(dev, "tsgpio48-en", 0);
        tsgpio_en[48] = devm_gpiod_get_optional(dev, "tsgpio49-en", 0);
        tsgpio_en[49] = devm_gpiod_get_optional(dev, "tsgpio50-en", 0);
        tsgpio_en[50] = devm_gpiod_get_optional(dev, "tsgpio51-en", 0);
        tsgpio_en[51] = devm_gpiod_get_optional(dev, "tsgpio52-en", 0);
        tsgpio_en[52] = devm_gpiod_get_optional(dev, "tsgpio53-en", 0);
        tsgpio_en[53] = devm_gpiod_get_optional(dev, "tsgpio54-en", 0);
        tsgpio_en[54] = devm_gpiod_get_optional(dev, "tsgpio55-en", 0);
        tsgpio_en[55] = devm_gpiod_get_optional(dev, "tsgpio56-en", 0);
        tsgpio_en[56] = devm_gpiod_get_optional(dev, "tsgpio57-en", 0);
        tsgpio_en[57] = devm_gpiod_get_optional(dev, "tsgpio58-en", 0);
        tsgpio_en[58] = devm_gpiod_get_optional(dev, "tsgpio59-en", 0);
        tsgpio_en[59] = devm_gpiod_get_optional(dev, "tsgpio60-en", 0);
        tsgpio_en[60] = devm_gpiod_get_optional(dev, "tsgpio61-en", 0);
        tsgpio_en[61] = devm_gpiod_get_optional(dev, "tsgpio62-en", 0);
        tsgpio_en[62] = devm_gpiod_get_optional(dev, "tsgpio63-en", 0);
        tsgpio_en[63] = devm_gpiod_get_optional(dev, "tsgpio64-en", 0);
        tsgpio_en[64] = devm_gpiod_get_optional(dev, "tsgpio65-en", 0);
        tsgpio_en[65] = devm_gpiod_get_optional(dev, "tsgpio66-en", 0);
        tsgpio_en[66] = devm_gpiod_get_optional(dev, "tsgpio67-en", 0);
        tsgpio_en[67] = devm_gpiod_get_optional(dev, "tsgpio68-en", 0);
        tsgpio_en[68] = devm_gpiod_get_optional(dev, "tsgpio69-en", 0);
        tsgpio_en[69] = devm_gpiod_get_optional(dev, "tsgpio70-en", 0);
        tsgpio_en[70] = devm_gpiod_get_optional(dev, "tsgpio71-en", 0);
        tsgpio_en[71] = devm_gpiod_get_optional(dev, "tsgpio72-en", 0);
       
       
        
       
        for(i=0;i<LENGHT_GPIO;i++){
          gpiod_direction_input(tsgpio_en[i]);
        }


        return 0;
}




/*----------------------------------------------------------------------------*/
static int tsgpio_control_probe(struct platform_device *pdev)
{
        int ret;    
  
        ret = gpio_parse_dt(&pdev->dev);
        if(ret) {
                printk("%s:parse gpio dt failed\n",__func__);
                return ret;
        }

      
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


        ret = tsgpio_control_create_attr(&tsgpio_control_driver.driver);
        if(ret) {
                printk("%s:create attr failed\n",__func__);
                return ret;
        }

	

        printk("%s success\n",__func__);
        return 0;
}

static int __init tsgpio_control_init(void)
{
	return platform_driver_register(&tsgpio_control_driver);
}

static void __exit tsgpio_control_exit(void)
{
	platform_driver_unregister(&tsgpio_control_driver);
}

module_init(tsgpio_control_init);
module_exit(tsgpio_control_exit);




