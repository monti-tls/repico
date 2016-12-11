#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/errno.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandre Monti");
MODULE_DESCRIPTION("");
MODULE_VERSION("");

// For reference :
//   https://github.com/beagleboard/linux/blob/4.4/drivers/spi/spidev.c

/***************************/
/*** Internal structures ***/
/***************************/

struct rap_device
{
    struct spi_device* spi_device;
    struct list_head list;
};

/****************************/
/*** Internal module data ***/
/****************************/

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/******************/
/*** Prototypes ***/
/******************/

static int rap_driver_probe(struct spi_device* spi);
static int rap_driver_remove(struct spi_device* spi);

/**************************/
/*** Device match table ***/
/**************************/

static const struct of_device_id rap_dt_ids[] =
{
    { .compatible = "rohm,dh2228fv" },
    { .compatible = "lineartechnology,ltc2488" },
    {},
};

MODULE_DEVICE_TABLE(of, rap_dt_ids);

/******************/
/*** SPI driver ***/
/******************/

static struct spi_driver rap_driver =
{
    .driver =
    {
        .name = "rap",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rap_dt_ids)
    },
    .probe = &rap_driver_probe,
    .remove = &rap_driver_remove
};

static int rap_driver_probe(struct spi_device* spi)
{
    struct rap_device* rap;

    printk(KERN_INFO "rap_driver_probe()\n");

    // Allocate memory for the device object
    rap = kzalloc(sizeof(struct rap_device), GFP_KERNEL);
    if (!rap)
        return -ENOMEM;

    // Add it in our device list
    INIT_LIST_HEAD(&rap->list);
    list_add(&rap->list, &device_list);

    // Create sysfs kobj
    // Create sysfs attr group

    // Initialize the SPI device
    spi->max_speed_hz = 500000;
    spi_setup(spi);
    rap->spi_device = spi;

    // Add driver-specific data to the SPI device
    //   so we can find ourselves in driver_remove for example
    spi_set_drvdata(spi, rap);

    return 0;
}

static int rap_driver_remove(struct spi_device* spi)
{
    struct rap_device* rap;
    
    printk(KERN_INFO "rap_driver_remove()\n");

    // Get corresponding RAP device
    rap = spi_get_drvdata(spi);

    // Remove the RAP device from list and free allocated memory
    mutex_lock(&device_list_lock);
    list_del(&rap->list);
    kfree(rap);
    mutex_unlock(&device_list_lock);

    return 0;
}

/*********************************/
/*** Kernel module init / exit ***/
/*********************************/

static int __init rap_init(void)
{
    int err;

    printk(KERN_INFO "rap_init()\n");

    // Register device driver
    err = spi_register_driver(&rap_driver);
    if (err < 0)
    {
        printk(KERN_ERR "spi_register_driver() failed with code %d\n", err);
        return err;
    }

    return 0;
}

static void __exit rap_exit(void)
{
    spi_unregister_driver(&rap_driver);

    printk(KERN_INFO "rap_exit()\n");
}

module_init(rap_init);
module_exit(rap_exit);
