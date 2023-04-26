#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/gpio/driver.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/hrtimer.h>
#include <linux/ioport.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include <linux/string.h>





#define TS_DEVICE_NAME	"skykey"


#define DATA_LENGTH 32 /* 定义数据大小 */


struct skykey_priv {
	char orientation[DATA_LENGTH]; 
	const char	*dpi;
	const char	*firstscreen;
	const char	*secondscreen;
	const char	*firstbufsize;
	const char	*secondbufsize;
	
	struct platform_device *pdev;
};

 
	

static struct skykey_priv  *sky_data;

static struct kobject *kobj = NULL;


int skykey_open(struct inode *inode,struct file *filp)
{
    printk(KERN_INFO "ts open gpio==\n");
	
	return 0;
}

int skykey_release(struct inode *inode,struct file *filp){
	
    return 0;
}

	
static long skykey_ioctl(struct file *file, unsigned int cmd, unsigned long  arg){
   printk("skykey Default %d\n",cmd);
   return 0;

}


static struct file_operations skykey_ops = {	
	.owner 	= THIS_MODULE,	
	.open 	= skykey_open,
	.release= skykey_release,
	.unlocked_ioctl = skykey_ioctl,
};


static struct miscdevice skykey_dev = {
	.minor	= DMAPI_MINOR,		
	.name	= TS_DEVICE_NAME,
	.fops	= &skykey_ops,
};


static int read_key(struct skykey_priv *sData, int id)
{
	char str[DATA_LENGTH] = {0};
	int ret = -1;
 
	memset(str, 0, DATA_LENGTH);
	ret = rk_vendor_read(id, str, (DATA_LENGTH-1));
 
	if (ret > 0) {
		strcpy(sky_data->orientation, str);
		if(strlen(sky_data->orientation) <= 0 ){
			strcpy(sky_data->orientation, "null");
		}
		printk("%s: Read key data: %s\n",__func__,sky_data->orientation);
	}else {
		goto set_null;
	}
 
	return ret;
 
set_null:
	strcpy(sky_data->orientation, "null");
	
	return ret;
}
 

static int write_key(struct skykey_priv *sData, int id, const char *buff)
{
	char str[DATA_LENGTH];
	int ret = -1;
 
	memset(str, 0, DATA_LENGTH);
	strcpy(str, buff);
	ret = rk_vendor_write(id, str, (DATA_LENGTH-1));
	
	if (ret == 0) {
		printk("%s: Data key succeeded: %s\n",__func__,str);
	} else {
		printk("%s: Data key failure: %s\n",__func__,str);
	}
	
	return ret;
}
 
static ssize_t sky_key_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
    
	int ret = -1;
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
	
   
	
	if(sky_data==NULL){
		return -1;
	}
	
	ret = write_key(sky_data, PLAYREADY_ROOT_KEY_1_ID, buf);
	
	if(ret!=0){
		return ret;
	}
	
         
    return count;
}

static ssize_t sky_key_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  	if(read_key(sky_data, PLAYREADY_ROOT_KEY_1_ID) > 0){
		return sprintf(buf, "%s\n", sky_data->orientation);
	} else {
		return sprintf(buf, "%s\n","null");
	} 
}
static DEVICE_ATTR(key,0664,sky_key_show, sky_key_store);




static ssize_t sky_dpi_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
    return count;
}

static ssize_t sky_dpi_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", sky_data->dpi);
}

static DEVICE_ATTR(dpi,0664,sky_dpi_show, sky_dpi_store);

static ssize_t sky_firstscreen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
    return count;
}

static ssize_t sky_firstscreen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", sky_data->firstscreen);
}

static DEVICE_ATTR(firstscreen,0664,sky_firstscreen_show, sky_firstscreen_store);

static ssize_t sky_secondscreen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
    return count;
}

static ssize_t sky_secondscreen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", sky_data->secondscreen);
}

static DEVICE_ATTR(secondscreen,0664,sky_secondscreen_show, sky_secondscreen_store);

static ssize_t sky_firstbufsize_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
    return count;
}

static ssize_t sky_firstbufsize_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", sky_data->firstbufsize);
}

static DEVICE_ATTR(firstbufsize,0664,sky_firstbufsize_show, sky_firstbufsize_store);

	
static ssize_t sky_secondbufsize_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{ 
	unsigned long enable;	
	enable = simple_strtoul(buf, NULL, 10);
	printk("ts:%s",buf);
    return count;
}

static ssize_t sky_secondbufsize_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", sky_data->secondbufsize);
}

static DEVICE_ATTR(secondbufsize,0664,sky_secondbufsize_show, sky_secondbufsize_store);



static struct attribute *skykey_sysfs_entries[] = {    
    &dev_attr_key.attr,     
	&dev_attr_dpi.attr,   
	&dev_attr_firstscreen.attr,
	&dev_attr_secondscreen.attr,
	&dev_attr_firstbufsize.attr,
	&dev_attr_secondbufsize.attr,
    NULL,
};

static struct attribute_group  skykey_attr_group = 
{ 
  .attrs =(struct attribute **) skykey_sysfs_entries,
};

static int skykey_probe(struct platform_device *pdev)
{
	int ret = -1;

	struct skykey_priv *ts;
	
	struct device_node *np = pdev->dev.of_node;
	ts = devm_kzalloc(&pdev->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		ret = -ENOMEM;
		goto err0;
	}

	ts->pdev = pdev;
	platform_set_drvdata(pdev, ts);
	sky_data=ts;
	

    kobj = kobject_create_and_add("skykey",&platform_bus.kobj);
    if (!kobj) 
        {        
        printk(KERN_ERR "Fail, ts kobject for key ...\n");    
        return 0;
         }    
    ret = sysfs_create_group(kobj, &skykey_attr_group);
    if (ret<0) 
        {
        printk(KERN_ERR "Fail, create sysfs group for  skykey...\n");
        kobject_del(kobj);        
        return 0;    
        }
    
	ret = of_property_read_string(np, "sky,screendpi", &sky_data->dpi);
	if (ret) {
		return -1;
	}
    
    ret = of_property_read_string(np, "sky,firstscreen", &sky_data->firstscreen);
	if (ret) {
		return -1;
	}
	ret = of_property_read_string(np, "sky,secondscreen", &sky_data->secondscreen);
	if (ret) {
		return -1;
	}
	
	ret = of_property_read_string(np, "sky,firstbufsize", &sky_data->firstbufsize);
	if (ret) {
		return -1;
	}

	ret = of_property_read_string(np, "sky,secondbufsize", &sky_data->secondbufsize);
	if (ret) {
		return -1;
	}

	return 0;
	
err0:
	return ret;
	
}

static int skykey_remove(struct platform_device *pdev)
{
	
	sysfs_remove_group(kobj, &skykey_attr_group);
	return 0;
}

static const struct of_device_id skykey_dt_ids[] = {
	{ .compatible = "sky,key", },
	{},
};
MODULE_DEVICE_TABLE(of, skykey_dt_ids);

static struct platform_driver skykey_device_driver = {
	.probe		= skykey_probe,
	.remove		= skykey_remove,
	.driver		= {
		.name	= "skykey",
		.of_match_table = skykey_dt_ids,
	}
};

static int __init skykey_init(void)
{

  int ret=0;
  platform_driver_register(&skykey_device_driver);
  
  ret = misc_register(&skykey_dev);//注册混杂设备
  if(ret<0)
  {
	printk(KERN_ERR "misc register error!!!\n");  
  }
  
  

  return  ret; 
}

module_init(skykey_init);   //注册优先级为2

static void __exit skykey_exit(void)
{

    misc_deregister(&skykey_dev);//注销设备
	platform_driver_unregister(&skykey_device_driver);
}
module_exit(skykey_exit);

MODULE_AUTHOR("TS chengzheng");
MODULE_DESCRIPTION("TS Mobile KEY Driver");
MODULE_LICENSE("GPL v2");


