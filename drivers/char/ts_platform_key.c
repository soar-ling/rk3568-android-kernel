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
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include <linux/string.h>

static int dbg_enable;
module_param_named(dbg_level, dbg_enable, int, 0644);

#define DBG(args...) \
    do { \
        if (dbg_enable) { \
            pr_info(args); \
        } \
    } while (0)

#define DRIVER_NAME	"fdt_key"
#define DEVICE_NAME	"key"
#define DATA_LENGTH	32 /* vendor page size */
static struct platform_device *pdev;

struct fdt_info_priv {
    char orientation[DATA_LENGTH];
    const char	*dpi;
    const char	*default_orientation;
    const char	*firstscreen;
    const char	*secondscreen;
    const char	*firstbufsize;
    const char	*secondbufsize;
};

static int vendor_key_read(struct fdt_info_priv *priv, int id)
{
    char str[DATA_LENGTH] = {0};
    int ret = -1;

    memset(str, 0, DATA_LENGTH);

    ret = rk_vendor_read(id, str, (DATA_LENGTH-1));
    if (ret && strlen(str)) {
        strcpy(priv->orientation, str);
    } else if (!strlen(str)) {
	strcpy(priv->orientation, priv->default_orientation);
    }
    DBG("%s: %s\n", __func__, priv->orientation);

    return ret;
}

static int vendor_key_write(struct fdt_info_priv *priv, int id, const char *buff)
{
    char str[DATA_LENGTH];
    int ret = -1;

    memset(str, 0, DATA_LENGTH);

    strcpy(str, buff);
    ret = rk_vendor_write(id, str, (DATA_LENGTH-1));
    if (ret == 0) {
        DBG("%s: %s\n",__func__,str);
    } else {
        DBG("%s: failed %s\n",__func__,str);
    }

    return ret;
}

static ssize_t fdt_info_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    if(vendor_key_read(priv, COMMON_DRM_KEY) > 0) {
        return sprintf(buf, "%s\n", priv->orientation);
    } if(strlen(priv->default_orientation)) {
		return sprintf(buf, "%s\n", priv->default_orientation);
	} else {
        return sprintf(buf, "%s\n","null");
    }
}

static ssize_t fdt_info_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t count)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);
    int ret = -1;
    unsigned long enable;

    enable = simple_strtoul(buf, NULL, 10);

    ret = vendor_key_write(priv, COMMON_DRM_KEY, buf);
    if(ret) {
        DBG("%s vendor key write failed.\n", __func__);
    }

    return count;
}
static DEVICE_ATTR(key, 0664, fdt_info_show, fdt_info_store);

static ssize_t orientation_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->default_orientation);
}
static DEVICE_ATTR(orientation, 0444, orientation_show, NULL);

static ssize_t dpi_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->dpi);
}
static DEVICE_ATTR(dpi, 0444, dpi_show, NULL);

static ssize_t firstscreen_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->firstscreen);
}
static DEVICE_ATTR(firstscreen, 0444, firstscreen_show, NULL);

static ssize_t secondscreen_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->secondscreen);
}
static DEVICE_ATTR(secondscreen, 0444, secondscreen_show, NULL);

static ssize_t firstbufsize_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->firstbufsize);
}
static DEVICE_ATTR(firstbufsize, 0444, firstbufsize_show, NULL);

static ssize_t secondbufsize_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct fdt_info_priv * priv = dev_get_drvdata(dev);

    return sprintf(buf, "%s\n", priv->secondbufsize);
}
static DEVICE_ATTR(secondbufsize, 0444, secondbufsize_show, NULL);

static struct attribute *fdt_info_sysfs_entries[] = {
    &dev_attr_key.attr,
    &dev_attr_orientation.attr,
    &dev_attr_dpi.attr,
    &dev_attr_firstscreen.attr,
    &dev_attr_secondscreen.attr,
    &dev_attr_firstbufsize.attr,
    &dev_attr_secondbufsize.attr,
    NULL,
};

static struct attribute_group  fdt_info_attr_group = {
    .attrs =(struct attribute **) fdt_info_sysfs_entries,
};

static int fdt_info_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    struct fdt_info_priv *priv;
    int ret = -1;

    priv = devm_kzalloc(&pdev->dev, sizeof(struct fdt_info_priv), GFP_KERNEL);
    if (!priv) {
        ret = -ENOMEM;
        goto err;
    }

    platform_set_drvdata(pdev, priv);

    if (of_property_read_string(np, "dpi", &priv->dpi)) {
        DBG("Invalid or missing dpi!\n");
    }

    if (of_property_read_string(np, "orientation", &priv->default_orientation)) {
        DBG("Invalid or missing orientation!\n");
    }

    if (of_property_read_string(np, "vendor.hwc.device.primary", &priv->firstscreen)) {
        DBG("Invalid or missing vendor.hwc.device.primary!\n");
    }

    if (of_property_read_string(np, "vendor.hwc.device.extend", &priv->secondscreen)) {
        DBG("Invalid or missing vendor.hwc.device.extend!\n");
    }

    if (of_property_read_string(np, "persist.vendor.framebuffer.main", &priv->firstbufsize)) {
        DBG("Invalid or missing persist.vendor.framebuffer.main!\n");
    }

    if (of_property_read_string(np, "persist.vendor.framebuffer.aux", &priv->secondbufsize)) {
        DBG("Invalid or missing persist.vendor.framebuffer.aux!\n");
    }

    ret = sysfs_create_group(&pdev->dev.kobj, &fdt_info_attr_group);
    if (ret) {
        dev_err(&pdev->dev, "failed to create sysfs group\n");
        return ret;
    }

    return 0;

err:
    return ret;
}

static int fdt_info_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &fdt_info_attr_group);

    return 0;
}

static const struct of_device_id fdt_info_dt_ids[] = {
    { .compatible = "info,lcd", },
    {},
};
MODULE_DEVICE_TABLE(of, fdt_info_dt_ids);

static struct platform_driver fdt_info_device_driver = {
    .probe		= fdt_info_probe,
    .remove		= fdt_info_remove,
    .driver		= {
        .name		= DRIVER_NAME,
        .of_match_table	= fdt_info_dt_ids,
    }
};

static int __init fdt_info_init(void)
{
    int ret;

    /*pdev = platform_device_alloc(DEVICE_NAME, -1);
    if (!pdev) {
        pr_err("failed to allocate platform device\n");
        return -ENOMEM;
    }

    ret = platform_device_add(pdev);
    if (ret) {
        pr_err("failed to add platform device\n");
        platform_device_put(pdev);
        return ret;
    }*/

    ret = platform_driver_register(&fdt_info_device_driver);
    if (ret) {
        pr_err("failed to register platform driver\n");
        platform_device_unregister(pdev);
        return ret;
    }

    return 0;
}
module_init(fdt_info_init);

static void __exit fdt_info_exit(void)
{
    platform_driver_unregister(&fdt_info_device_driver);
}
module_exit(fdt_info_exit);

MODULE_AUTHOR("skysi@skysi.com");
MODULE_DESCRIPTION("DTS transfer LCD infomation to system");
MODULE_LICENSE("GPL v2");

