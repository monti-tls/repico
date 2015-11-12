/*
 * repico/software/bbb/repico_rap
 * Copyright (C) 2016 Alexandre Monti
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
#include <linux/spinlock.h>
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

    spinlock_t lock;
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

#define DEF(a) a,
enum
{
    #include "defs/rap.def"
    #include "defs/rap_errno.def"
};
#undef DEF

void rap_spi_write(struct rap_device* rap, uint8_t* data, int len)
{
    struct spi_transfer t[1];
    struct spi_message m;
    unsigned long flags;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = data;
    t[0].len = len;
    spi_message_add_tail(&t[0], &m);

    spin_lock_irqsave(&rap->lock, flags);
    spi_sync(rap->spi_device, &m);
    spin_unlock_irqrestore(&rap->lock, flags);
}

void rap_spi_write2(struct rap_device* rap, uint8_t* data1, int len1, uint8_t* data2, int len2)
{
    struct spi_transfer t[2];
    struct spi_message m;
    unsigned long flags;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].tx_buf = data1;
    t[0].len = len1;
    spi_message_add_tail(&t[0], &m);

    t[1].tx_buf = data2;
    t[1].len = len2;
    spi_message_add_tail(&t[1], &m);

    spin_lock_irqsave(&rap->lock, flags);
    spi_sync(rap->spi_device, &m);
    spin_unlock_irqrestore(&rap->lock, flags);
}

void rap_spi_read(struct rap_device* rap, uint8_t* data, int len)
{
    struct spi_transfer t[1];
    struct spi_message m;
    unsigned long flags;

    memset(&t[0], 0, sizeof(t));
    spi_message_init(&m);

    t[0].rx_buf = data;
    t[0].len = len;
    spi_message_add_tail(&t[0], &m);

    spin_lock_irqsave(&rap->lock, flags);
    spi_sync(rap->spi_device, &m);
    spin_unlock_irqrestore(&rap->lock, flags);
}

int rap_write(struct rap_device* rap, uint8_t reg, void* data, uint8_t len)
{
    uint8_t cmd[] =
    {
        RAP_SYNC_BYTE,
        RAP_CMD_WRITE,
        reg,
        len
    };
    uint8_t rx[4];

    rap_spi_write2(rap, &cmd[0], sizeof(cmd), data, len);
    rap_spi_read(rap, &rx[0], sizeof(rx));

    if (rx[1] != RAP_SYNC_BYTE)
        return -RAP_ESYNC;
    else if (rx[2] == RAP_STATUS_INVALID_CMD)
        return -RAP_ECMD;
    else if (rx[2] == RAP_STATUS_INVALID_REG)
        return -RAP_EREG;
    else if (rx[2] == RAP_STATUS_INVALID_ACC)
        return -RAP_EACC;

    return len;
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

    rap_spi_write(rap, &cmd[0], sizeof(cmd));
    rap_spi_read(rap, &rx[0], sizeof(rx));

    if (rx[1] != RAP_SYNC_BYTE)
        return -RAP_ESYNC;
    else if (rx[2] == RAP_STATUS_INVALID_CMD)
        return -RAP_ECMD;
    else if (rx[2] == RAP_STATUS_INVALID_REG)
        return -RAP_EREG;
    else if (rx[2] == RAP_STATUS_INVALID_ACC)
        return -RAP_EACC;

    rap_spi_read(rap, data, rx[3]);

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
    int err;
    struct rap_device* rap;
    uint8_t data[] = {
        0xDE, 0xAD, 0xBE, 0xEF
    };

    rap = kobj_rap_device(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?

    err = rap_write(rap, 0xAB, &data[0], sizeof(data));
    printk(KERN_INFO "%d\n", err);

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
        .name = "repico_rap",
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

    spin_lock_init(&rap->lock);

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
}

module_init(rap_init);
module_exit(rap_exit);
