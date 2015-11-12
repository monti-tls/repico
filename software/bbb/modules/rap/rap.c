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
#include <linux/delay.h>

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
    struct kobject* kobj;
    struct attribute_group kobj_attr_group;
    uint8_t regval;

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

static uint8_t spi_write_reg(struct rap_device* rap, uint8_t reg, uint8_t val);
static ssize_t debug_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
static ssize_t debug_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size);
static struct rap_device* kobj_rap_device(struct kobject* kobj);
static int rap_driver_probe(struct spi_device* spi);
static int rap_driver_remove(struct spi_device* spi);

/**************************/
/*** SPI RAP primitives ***/
/**************************/

static int spi_rap_sync_write(struct rap_device* rap, uint16_t sync)
{
    struct spi_transfer t[1];
    struct spi_message m;

    // Setup SPI transfers and message
    memset(t, 0, sizeof(t));
    spi_message_init(&m);

    // First, send the sync sequence
    t[0].tx_buf = &sync;
    t[0].len = sizeof(uint16_t);
    spi_message_add_tail(&t[0], &m);

    // spin_lock_irqsave(&spilock, spilockflags);
    spi_sync(rap->spi_device, &m);
    // spin_unlock_irqrestore(&spilock, spilockflags);

    return 0;
}

static int spi_rap_sync_read(struct rap_device* rap, uint16_t sync)
{
    struct spi_transfer t[1];
    struct spi_message m;
    uint16_t rxbuf;

    // Setup SPI transfers and message
    memset(t, 0, sizeof(t));
    spi_message_init(&m);

    // Get back the sync sequence
    t[0].rx_buf = &rxbuf;
    t[0].len = sizeof(uint16_t);
    spi_message_add_tail(&t[0], &m);

    // spin_lock_irqsave(&spilock, spilockflags);
    spi_sync(rap->spi_device, &m);
    // spin_unlock_irqrestore(&spilock, spilockflags);

    printk(KERN_INFO "sync = %04X\n", rxbuf);

    return rxbuf == sync ? 0 : -1;
}

static int spi_rap_reg_write(struct rap_device* rap, uint16_t cmd, uint16_t reg)
{
    struct spi_transfer t[2];
    struct spi_message m;

    // Setup SPI transfers and message
    memset(t, 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = &cmd;
    t[0].len = 2;
    spi_message_add_tail(&t[0], &m);

    t[1].tx_buf = &reg;
    t[1].len = 2;
    spi_message_add_tail(&t[1], &m);

    // spin_lock_irqsave(&spilock, spilockflags);
    spi_sync(rap->spi_device, &m);
    // spin_unlock_irqrestore(&spilock, spilockflags);

    return 0;
}

static int spi_rap_reg_write_data(struct rap_device* rap, uint16_t len, uint16_t* data)
{
    struct spi_transfer t[2];
    struct spi_message m;

    // Setup SPI transfers and message
    memset(t, 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = &len;
    t[0].len = 2;
    spi_message_add_tail(&t[0], &m);

    t[1].tx_buf = &data;
    t[1].len = 2 * len;
    spi_message_add_tail(&t[1], &m);

    // spin_lock_irqsave(&spilock, spilockflags);
    spi_sync(rap->spi_device, &m);
    // spin_unlock_irqrestore(&spilock, spilockflags);

    return 0;
}

/************************/
/*** kobject bindings ***/
/************************/

static ssize_t debug_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
    struct rap_device* rap;

    rap = kobj_rap_device(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?
    
    return sprintf(buf, "0x%02X\n", rap->regval);
}

static ssize_t debug_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size)
{
    struct rap_device* rap;

    rap = kobj_rap_device(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?

    /*spi_rap_sync_write(rap);
    mdelay(100);
    spi_rap_sync_read(rap);*/

    spi_rap_sync_write(rap, 0x5A5A);
    spi_rap_sync_read(rap, 0x5A5A);
    spi_rap_sync_write(rap, 0x0001);
    spi_rap_sync_write(rap, 0x1234);
    spi_rap_sync_write(rap, 0x0002);
    spi_rap_sync_write(rap, 0xDEAD);
    spi_rap_sync_write(rap, 0xBEEF);

    return size;
}

static struct kobj_attribute debug_attr = __ATTR(debug, 0666, debug_show, debug_store);

static struct attribute* rap_attrs[] =
{
    &debug_attr.attr,
    0
};

static struct rap_device* kobj_rap_device(struct kobject* kobj)
{
    struct rap_device* rap;
    list_for_each_entry(rap, &device_list, list)
    {
        if (rap->kobj == kobj)
            return rap;
    }

    return 0;
}

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
    int err;
    struct rap_device* rap;

    printk(KERN_INFO "rap_driver_probe()\n");

    // Allocate memory for the device object
    rap = kzalloc(sizeof(struct rap_device), GFP_KERNEL);
    if (!rap)
        return -ENOMEM;

    // Create kobject entry
    rap->kobj = kobject_create_and_add("repico_rap", kernel_kobj);
    if (!rap->kobj)
    {
        printk(KERN_ALERT "repico_rap: failed to create kobject\n");
        kfree(rap);
        return -ENOMEM;
    }

    // Setup an unnamed attribute group
    rap->kobj_attr_group.name = 0;
    rap->kobj_attr_group.attrs = rap_attrs;

    // And associated SysFS entries
    err = sysfs_create_group(rap->kobj, &rap->kobj_attr_group);
    if(err)
    {
        printk(KERN_ALERT "repico_rap: failed to create sysfs group\n");
        kobject_put(rap->kobj);
        kfree(rap);
        return err;
    }

    // Initialize the SPI device
    // spi->max_speed_hz = 50000;
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 16;
    spi_setup(spi);
    rap->spi_device = spi;

    // Add driver-specific data to the SPI device
    //   so we can find ourselves in driver_remove for example
    spi_set_drvdata(spi, rap);

    // Add the RAP device in our device list
    INIT_LIST_HEAD(&rap->list);
    list_add(&rap->list, &device_list);

    return 0;
}

static int rap_driver_remove(struct spi_device* spi)
{
    struct rap_device* rap;
    
    printk(KERN_INFO "rap_driver_remove()\n");

    // Get corresponding RAP device
    rap = spi_get_drvdata(spi);

    // Release SysFS entries
    kobject_put(rap->kobj);

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
