/*
 * repico/software/bbb/rapdev
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
#include <net/genetlink.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandre Monti");
MODULE_DESCRIPTION("");
MODULE_VERSION("");

// For reference :
//   https://github.com/beagleboard/linux/blob/4.4/drivers/spi/spidev.c

/***************************/
/*** Internal structures ***/
/***************************/

struct rapdev
{
    uint32_t dev_id;

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

static uint32_t rapdev_next_devid;
static LIST_HEAD(rapdev_devlist);
static DEFINE_MUTEX(rapdev_lock);

/******************/
/*** Prototypes ***/
/******************/

static void rapdev_spi_write(struct rapdev* rap, uint8_t* data, int len);
static void rapdev_spi_write2(struct rapdev* rap, uint8_t* data1, int len1, uint8_t* data2, int len2);
static void rapdev_spi_read(struct rapdev* rap, uint8_t* data, int len);
static int rapdev_write_reg(struct rapdev* rap, uint8_t reg, void* data, uint8_t len);
static int rapdev_read_reg(struct rapdev* rap, uint8_t reg, void* data);
static int rapdev_genl_list_devices(struct sk_buff* skb, struct genl_info* info);
static int rapdev_genl_write_reg(struct sk_buff* skb, struct genl_info* info);
static int rapdev_genl_read_reg(struct sk_buff* skb, struct genl_info* info);
static ssize_t debug_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf);
static ssize_t debug_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size);
static struct rapdev* rapdev_get_kobj(struct kobject* kobj);
static int rapdev_driver_probe(struct spi_device* spi);
static int rapdev_driver_remove(struct spi_device* spi);

/**************************/
/*** SPI RAP primitives ***/
/**************************/

#define DEF(a) a,
enum
{
    #include "defs/rap_spi.def"
    #include "defs/rap_errno.def"
};
#undef DEF

static void rapdev_spi_write(struct rapdev* rap, uint8_t* data, int len)
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

static void rapdev_spi_write2(struct rapdev* rap, uint8_t* data1, int len1, uint8_t* data2, int len2)
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

static void rapdev_spi_read(struct rapdev* rap, uint8_t* data, int len)
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

static int rapdev_write_reg(struct rapdev* rap, uint8_t reg, void* data, uint8_t len)
{
    uint8_t cmd[] =
    {
        RAP_SYNC_BYTE,
        RAP_CMD_WRITE,
        reg,
        len
    };
    uint8_t rx[4];

    rapdev_spi_write2(rap, &cmd[0], sizeof(cmd), data, len);
    rapdev_spi_read(rap, &rx[0], sizeof(rx));

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

static int rapdev_read_reg(struct rapdev* rap, uint8_t reg, void* data)
{
    uint8_t cmd[] =
    {
        RAP_SYNC_BYTE,
        RAP_CMD_READ,
        reg
    };
    uint8_t rx[4];

    rapdev_spi_write(rap, &cmd[0], sizeof(cmd));
    rapdev_spi_read(rap, &rx[0], sizeof(rx));

    if (rx[1] != RAP_SYNC_BYTE)
        return -RAP_ESYNC;
    else if (rx[2] == RAP_STATUS_INVALID_CMD)
        return -RAP_ECMD;
    else if (rx[2] == RAP_STATUS_INVALID_REG)
        return -RAP_EREG;
    else if (rx[2] == RAP_STATUS_INVALID_ACC)
        return -RAP_EACC;

    rapdev_spi_read(rap, data, rx[3]);

    return rx[3];
}

/***********************/
/*** Generic Netlink ***/
/***********************/

/*** Attribute IDs ***/

#define DEF(name, nla_type) name,
enum
{
    RAP_GENL_ATTR_INVAL = 0,
    #include "defs/rap_genl_attrs.def"
    RAP_GENL_ATTR_COUNT
};
#undef DEF

/*** Attribute types ***/

#define DEF(name, nla_type) [name] = { .type = nla_type },
static struct nla_policy rapdev_genl_policy[RAP_GENL_ATTR_COUNT] =
{
    #include "defs/rap_genl_attrs.def"
};
#undef DEF

/*** Generic Netlink family ***/

static struct genl_family rapdev_genl_family =
{
    .id = GENL_ID_GENERATE,
    .hdrsize = 0,
    .name = "rapdev",
    .version = 0,
    .maxattr = RAP_GENL_ATTR_COUNT - 1
};

/*** Command IDs ***/

#define DEF(name) name,
enum
{
    #include "defs/rap_genl_cmds.def"
};
#undef DEF

static int rapdev_genl_list_devices(struct sk_buff* skb, struct genl_info* info)
{
    struct sk_buff* n_skb;
    void* n_msg_header;
    int err;
    struct rapdev* rap;
    int rap_dev_count;
    int i;
    uint32_t* rap_dev_ids;

    /*** Get RAP device listing ***/

    mutex_lock(&rapdev_lock);
    
    rap_dev_count = 0;
    list_for_each_entry(rap, &rapdev_devlist, list)
        ++rap_dev_count;
    
    rap_dev_ids = kzalloc(rap_dev_count * sizeof(uint32_t), GFP_KERNEL);
    if (rap_dev_ids)
    {
        i = 0;
        list_for_each_entry(rap, &rapdev_devlist, list)
        {
            rap_dev_ids[i++] = rap->dev_id;
        }
    }
    
    mutex_unlock(&rapdev_lock);

    if (!rap_dev_ids)
        goto error;

    /*** Craft GENL response packet ***/

    if (!info)
        goto error;

    n_skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
    if (!n_skb)
        goto error;

    n_msg_header = genlmsg_put(n_skb, 0, info->snd_seq + 1, &rapdev_genl_family, 0, RAP_GENL_LIST_DEVICES);
    if (!n_msg_header)
        goto error;

    /*** Put actual GENL response contents ***/

    // No error
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_ERRNO, 0);
    if (err != 0)
        goto error;

    // Put this command's identifier for userspace side
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_CMD, RAP_GENL_LIST_DEVICES);
    if (err != 0)
        goto error;

    // Device count
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_DEV_COUNT, rap_dev_count);
    if (err != 0)
        goto error;

    // Device IDs
    err = nla_put(n_skb, RAP_GENL_ATTR_DEV_IDS, rap_dev_count * sizeof(uint32_t), rap_dev_ids);
    if (err != 0)
        goto error;

    /*** Send answer back to userspace ***/

    genlmsg_end(n_skb, n_msg_header);

    err = genlmsg_unicast(genl_info_net(info), n_skb, info->snd_portid);
    if (err != 0)
        goto error;

    /*** Clean-up and error handling ***/

    kfree(rap_dev_ids);
    return 0;

    error:
    printk(KERN_ERR "rapdev_genl_list_devices() error\n");
    return 0;
}

static int rapdev_genl_write_reg(struct sk_buff* skb, struct genl_info* info)
{
    struct sk_buff* n_skb;
    void* n_msg_header;
    int errno;
    int err;
    struct rapdev* rap;
    struct rapdev* rap_it;

    /*** Get RAP write parameters ***/

    errno = 0;
    rap = 0;

    // Check netlink request
    if (!info->attrs[RAP_GENL_ATTR_DEV_ID] ||
        !info->attrs[RAP_GENL_ATTR_REG_ID] ||
        !info->attrs[RAP_GENL_ATTR_REG_SIZE] ||
        !info->attrs[RAP_GENL_ATTR_REG_DATA])
    {
        errno = RAP_EPACKET;
    }

    // Get RAP device from id
    if (!errno)
    {
        int dev_id = nla_get_u32(info->attrs[RAP_GENL_ATTR_DEV_ID]);

        mutex_lock(&rapdev_lock);
        list_for_each_entry(rap_it, &rapdev_devlist, list)
        {
            if (rap_it->dev_id == dev_id)
            {
                rap = rap_it;
                break;
            }
        }
        mutex_unlock(&rapdev_lock);

        if (!rap)
            errno = RAP_EID;
    }

    // Get RAP write parameters and send SPI command
    if (!errno)
    {
        int reg_id = nla_get_u8(info->attrs[RAP_GENL_ATTR_REG_ID]);
        int reg_size = nla_get_u8(info->attrs[RAP_GENL_ATTR_REG_SIZE]);
        void* reg_data = nla_data(info->attrs[RAP_GENL_ATTR_REG_DATA]);

        errno = rapdev_write_reg(rap, reg_id, reg_data, reg_size);
        // Positive value is OK, otherwise get the errno as a positive number
        errno = errno > 0 ? 0 : -errno;
    }

    /*** Craft GENL response packet ***/

    if (!info)
        goto error;

    n_skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
    if (!n_skb)
        goto error;

    n_msg_header = genlmsg_put(n_skb, 0, info->snd_seq + 1, &rapdev_genl_family, 0, RAP_GENL_LIST_DEVICES);
    if (!n_msg_header)
        goto error;

    /*** Put actual GENL response contents ***/

    // Put eventual error number
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_ERRNO, errno);
    if (err != 0)
        goto error;

    // Put this command's identifier for userspace side
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_CMD, RAP_GENL_WRITE_REG);
    if (err != 0)
        goto error;

    /*** Send answer back to userspace ***/

    genlmsg_end(n_skb, n_msg_header);

    err = genlmsg_unicast(genl_info_net(info), n_skb, info->snd_portid);
    if (err != 0)
        goto error;

    /*** Clean-up and error handling ***/
    return 0;

    error:
    printk(KERN_ERR "rapdev_genl_write_reg() error\n");
    return 0;
}

static int rapdev_genl_read_reg(struct sk_buff* skb, struct genl_info* info)
{
    struct sk_buff* n_skb;
    void* n_msg_header;
    int errno;
    int err;
    struct rapdev* rap;
    struct rapdev* rap_it;
    uint8_t reg_data[256];
    int reg_size;

    /*** Get RAP write parameters ***/

    errno = 0;
    rap = 0;

    // Check netlink request
    if (!info->attrs[RAP_GENL_ATTR_DEV_ID] ||
        !info->attrs[RAP_GENL_ATTR_REG_ID])
    {
        errno = RAP_EPACKET;
    }

    // Get RAP device from id
    if (!errno)
    {
        int dev_id = nla_get_u32(info->attrs[RAP_GENL_ATTR_DEV_ID]);

        mutex_lock(&rapdev_lock);
        list_for_each_entry(rap_it, &rapdev_devlist, list)
        {
            if (rap_it->dev_id == dev_id)
            {
                rap = rap_it;
                break;
            }
        }
        mutex_unlock(&rapdev_lock);

        if (!rap)
            errno = RAP_EID;
    }

    // Get RAP read parameters and send SPI command

    reg_size = 0;

    if (!errno)
    {
        int reg_id = nla_get_u8(info->attrs[RAP_GENL_ATTR_REG_ID]);

        reg_size = rapdev_read_reg(rap, reg_id, &reg_data[0]);
        // Positive value is OK, otherwise get the errno as a positive number
        if (reg_size < 0)
            errno = -reg_size;
    }

    /*** Craft GENL response packet ***/

    if (!info)
        goto error;

    n_skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
    if (!n_skb)
        goto error;

    n_msg_header = genlmsg_put(n_skb, 0, info->snd_seq + 1, &rapdev_genl_family, 0, RAP_GENL_LIST_DEVICES);
    if (!n_msg_header)
        goto error;

    /*** Put actual GENL response contents ***/

    // Put eventual error number
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_ERRNO, errno);
    if (err != 0)
        goto error;

    // Put this command's identifier for userspace side
    err = nla_put_u32(n_skb, RAP_GENL_ATTR_CMD, RAP_GENL_READ_REG);
    if (err != 0)
        goto error;

    if (errno == 0)
    {
        // Register size
        err = nla_put_u32(n_skb, RAP_GENL_ATTR_REG_SIZE, reg_size);
        if (err != 0)
            goto error;

        // Register data
        err = nla_put(n_skb, RAP_GENL_ATTR_REG_DATA, reg_size, &reg_data[0]);
        if (err != 0)
            goto error;
    }

    /*** Send answer back to userspace ***/

    genlmsg_end(n_skb, n_msg_header);

    err = genlmsg_unicast(genl_info_net(info), n_skb, info->snd_portid);
    if (err != 0)
        goto error;

    /*** Clean-up and error handling ***/
    return 0;

    error:
    printk(KERN_ERR "rapdev_genl_read_reg() error\n");
    return 0;
}

static struct genl_ops rapdev_genl_ops[] =
{
    {
        .cmd = RAP_GENL_LIST_DEVICES,
        .flags = 0,
        .policy = &rapdev_genl_policy[0],
        .doit = &rapdev_genl_list_devices,
        .dumpit = 0
    },
    {
        .cmd = RAP_GENL_WRITE_REG,
        .flags = 0,
        .policy = &rapdev_genl_policy[0],
        .doit = &rapdev_genl_write_reg,
        .dumpit = 0
    },
    {
        .cmd = RAP_GENL_READ_REG,
        .flags = 0,
        .policy = &rapdev_genl_policy[0],
        .doit = &rapdev_genl_read_reg,
        .dumpit = 0
    }
};

/************************/
/*** kobject bindings ***/
/************************/

static ssize_t debug_show(struct kobject* kobj, struct kobj_attribute* attr, char* buf)
{
    struct rapdev* rap;

    rap = rapdev_get_kobj(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?
    
    return sprintf(buf, "0x%02X\n", rap->regval);
}

static ssize_t debug_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buf, size_t size)
{
    int err;
    struct rapdev* rap;
    uint8_t data[] = {
        0xDE, 0xAD, 0xBE, 0xEF
    };

    rap = rapdev_get_kobj(kobj);
    if (!rap)
        return -ENOMEM; //TODO: correct error ?

    err = rapdev_write_reg(rap, 0xAB, &data[0], sizeof(data));
    printk(KERN_INFO "%d\n", err);

    return size;
}

static struct kobj_attribute debug_attr = __ATTR(debug, 0666, debug_show, debug_store);

static struct attribute* rapdev_kobj_attrs[] =
{
    &debug_attr.attr,
    0
};

static struct rapdev* rapdev_get_kobj(struct kobject* kobj)
{
    struct rapdev* rap;
    list_for_each_entry(rap, &rapdev_devlist, list)
    {
        if (rap->kobj == kobj)
            return rap;
    }

    return 0;
}

/**************************/
/*** Device match table ***/
/**************************/

static const struct of_device_id rapdev_dt_ids[] =
{
    { .compatible = "rohm,dh2228fv" },
    { .compatible = "lineartechnology,ltc2488" },
    {},
};

MODULE_DEVICE_TABLE(of, rapdev_dt_ids);

/******************/
/*** SPI driver ***/
/******************/

static struct spi_driver rapdev_driver =
{
    .driver =
    {
        .name = "rapdev",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rapdev_dt_ids)
    },
    .probe = &rapdev_driver_probe,
    .remove = &rapdev_driver_remove
};

static int rapdev_driver_probe(struct spi_device* spi)
{
    int err;
    struct rapdev* rap;
    char kobj_name[256];

    // Allocate memory for the device object
    rap = kzalloc(sizeof(struct rapdev), GFP_KERNEL);
    if (!rap)
        return -ENOMEM;


    mutex_lock(&rapdev_lock);
    rap->dev_id = rapdev_next_devid++;
    mutex_unlock(&rapdev_lock);
    snprintf(&kobj_name[0], sizeof(kobj_name), "rapdev%d\n", rap->dev_id);

    // Create kobject entry
    rap->kobj = kobject_create_and_add(&kobj_name[0], kernel_kobj);
    if (!rap->kobj)
    {
        printk(KERN_ALERT "rapdev: failed to create kobject\n");
        kfree(rap);
        return -ENOMEM;
    }

    // Setup an unnamed attribute group
    rap->kobj_attr_group.name = 0;
    rap->kobj_attr_group.attrs = rapdev_kobj_attrs;

    // And associated SysFS entries
    err = sysfs_create_group(rap->kobj, &rap->kobj_attr_group);
    if(err)
    {
        printk(KERN_ALERT "rapdev: failed to create sysfs group\n");
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

    mutex_lock(&rapdev_lock);
    list_add(&rap->list, &rapdev_devlist);
    mutex_unlock(&rapdev_lock);

    return 0;
}

static int rapdev_driver_remove(struct spi_device* spi)
{
    struct rapdev* rap;

    // Get corresponding RAP device
    rap = spi_get_drvdata(spi);

    // Release SysFS entries
    kobject_put(rap->kobj);

    // Remove the RAP device from list and free allocated memory
    mutex_lock(&rapdev_lock);
    list_del(&rap->list);
    kfree(rap);
    mutex_unlock(&rapdev_lock);

    return 0;
}

/*********************************/
/*** Kernel module init / exit ***/
/*********************************/

static int __init rapdev_init(void)
{
    int err;
    size_t i;
    size_t j;

    rapdev_next_devid = 0;

    // Register device driver
    err = spi_register_driver(&rapdev_driver);
    if (err < 0)
    {
        printk(KERN_ERR "spi_register_driver() failed with code %d\n", err);
        return err;
    }

    // Register Generic Netlink family

    err = genl_register_family(&rapdev_genl_family);
    if (err != 0)
    {
        printk(KERN_ERR "genl_register_family() failed with code %d\n", err);
        return err;
    }

    for (i = 0; i < sizeof(rapdev_genl_ops) / sizeof(struct genl_ops); ++i)
    {
        err = genl_register_ops(&rapdev_genl_family, &rapdev_genl_ops[i]);
        if (err != 0)
        {
            printk(KERN_ERR "genl_register_ops() failed on %d-th element with code %d\n", i, err);

            for (j = 0; j < i; ++j)
                genl_unregister_ops(&rapdev_genl_family, &rapdev_genl_ops[j]);
            
            genl_unregister_family(&rapdev_genl_family);
            spi_unregister_driver(&rapdev_driver);
        }
    }

    return 0;
}

static void __exit rapdev_exit(void)
{
    int i;

    for (i = 0; i < sizeof(rapdev_genl_ops) / sizeof(struct genl_ops); ++i)
        genl_register_ops(&rapdev_genl_family, &rapdev_genl_ops[i]);
    
    genl_unregister_family(&rapdev_genl_family);

    spi_unregister_driver(&rapdev_driver);
}

module_init(rapdev_init);
module_exit(rapdev_exit);
