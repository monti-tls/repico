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

static ssize_t debug_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
static ssize_t debug_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size);
static struct rap_device* kobj_rap_device(struct kobject* kobj);
static int rap_driver_probe(struct spi_device* spi);
static int rap_driver_remove(struct spi_device* spi);

/**************************/
/*** SPI RAP primitives ***/
/**************************/

enum
{
    RAP_SYNC_BYTE = 0xA5,
    RAP_CMD_WRITE = 0x01,
    RAP_CMD_READ = 0x02
};

void rap_spi_write(struct spi_device* spi, uint8_t* data, int len)
{
    struct spi_transfer t[1];
    struct spi_message m;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = data;
    t[0].len = len;
    spi_message_add_tail(&t[0], &m);

    spi_sync(spi, &m);
}

void rap_spi_write2(struct spi_device* spi, uint8_t* data1, int len1, uint8_t* data2, int len2)
{
    struct spi_transfer t[2];
    struct spi_message m;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = data1;
    t[0].len = len1;
    spi_message_add_tail(&t[0], &m);

    t[1].tx_buf = data2;
    t[1].len = len2;
    spi_message_add_tail(&t[1], &m);

    spi_sync(spi, &m);
}

void rap_spi_read(struct spi_device* spi, uint8_t* data, int len)
{
    struct spi_transfer t[1];
    struct spi_message m;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].rx_buf = data;
    t[0].len = len;
    spi_message_add_tail(&t[0], &m);

    spi_sync(spi, &m);
}

void rap_write(struct rap_device* rap, uint8_t reg, void* data, uint8_t len)
{
    uint8_t cmd[] =
    {
        RAP_SYNC_BYTE,
        RAP_CMD_WRITE,
        reg,
        len
    };
    uint8_t rx[4];

    rap_spi_write2(rap->spi_device, &cmd[0], sizeof(cmd), data, len);
    rap_spi_read(rap->spi_device, &rx[0], sizeof(rx));

    printk(KERN_INFO "Sync bytes %02X %02X %02X\n", rx[1], rx[2], rx[3]);
}

int rap_read(struct rap_device* rap, uint8_t reg, void* data)
{
    uint8_t cmd[] =
    {
        RAP_SYNC_BYTE,
        RAP_CMD_READ,
        reg
    };
    uint8_t rx[4];

    rap_spi_write(rap->spi_device, &cmd[0], sizeof(cmd));
    rap_spi_read(rap->spi_device, &rx[0], sizeof(rx));

    rap_spi_read(rap->spi_device, data, rx[3]);
    return rx[3];
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
    uint8_t data[] = {
        0xDE, 0xAD, 0xBE, 0xEF
    };
    uint8_t rx[256];
    int i;
    int value = 123456;

    rap = kobj_rap_device(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?

    rap_write(rap, 0xAB, &data[0], sizeof(data));
    int len = rap_read(rap, 0xAB, &rx[0]);

    printk(KERN_INFO "Register contents :");
    for (i = 0; i < len; ++i)
        printk(KERN_INFO "%02X ", rx[i]);
    printk(KERN_INFO "\n");

    rap_write(rap, 0xAC, &value, sizeof(value));

    value = 0;
    len = rap_read(rap, 0xAC, &value);
    printk(KERN_INFO "regAC = %d\n", value);

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
    spi->mode = SPI_MODE_1;
    spi->bits_per_word = 8;
    spi->max_speed_hz = 50000;
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
