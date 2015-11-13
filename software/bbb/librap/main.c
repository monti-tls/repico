#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <stdio.h>
#include <stdlib.h>

#include "rap.h"

#define DEF(name, nla_type) name,
enum
{
    RAP_GENL_ATTR_INVAL = 0,
    #include "../../common/defs/rap_genl_attrs.def"
    RAP_GENL_ATTR_COUNT
};
#undef DEF

#define DEF(n) n,
enum
{
    #include "../../common/defs/rap_genl_cmds.def"
};
#undef DEF

static int rap_genl_callback(struct nl_msg* msg, void* arg)
{
    // Get message contents
    struct genlmsghdr* gnlh = nlmsg_data(nlmsg_hdr(msg));

    // Get message attributes
    struct nlattr* attrs[RAP_GENL_ATTR_COUNT];
    nla_parse(attrs, RAP_GENL_ATTR_COUNT - 1, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

    // And the RAP handler
    struct rap_handle* hndl = (struct rap_handle*) arg;

    if (!attrs[RAP_GENL_ATTR_ERRNO] ||
        !attrs[RAP_GENL_ATTR_CMD])
    {
        hndl->errno = RAP_EPACKET;
        return NL_SKIP;
    }

    // Stop here if there was some error
    int errno = nla_get_u32(attrs[RAP_GENL_ATTR_ERRNO]);
    if (errno != 0)
    {
        hndl->errno = errno;
        return NL_SKIP;
    }

    int cmd = nla_get_u32(attrs[RAP_GENL_ATTR_CMD]);
    switch (cmd)
    {
        case RAP_GENL_LIST_DEVICES:
        {
            if (!attrs[RAP_GENL_ATTR_DEV_COUNT] ||
                !attrs[RAP_GENL_ATTR_DEV_IDS])
            {
                hndl->errno = RAP_EPACKET;
                break;
            }

            int dev_count = nla_get_u32(attrs[RAP_GENL_ATTR_DEV_COUNT]);
            uint32_t* dev_ids = nla_data(attrs[RAP_GENL_ATTR_DEV_IDS]);

            hndl->dev_count = dev_count;
            if (dev_count)
            {
                hndl->dev_ids = malloc(dev_count * sizeof(uint32_t));
                memcpy(hndl->dev_ids, dev_ids, dev_count * sizeof(uint32_t));
            }

            break;
        }

        case RAP_GENL_READ_REG:
        {
            if (!attrs[RAP_GENL_ATTR_REG_SIZE] ||
                !attrs[RAP_GENL_ATTR_REG_DATA])
            {
                hndl->errno = RAP_EPACKET;
                break;
            }

            int reg_size = nla_get_u32(attrs[RAP_GENL_ATTR_REG_SIZE]);
            uint8_t* reg_data = nla_data(attrs[RAP_GENL_ATTR_REG_DATA]);

            hndl->read_size = 0;
            for (int i = 0; i < reg_size; ++i)
            {
                if (i >= hndl->max_read_size)
                {
                    hndl->errno = RAP_EOVF;
                    break;
                }

                ((uint8_t*) hndl->read_data)[i] = reg_data[i];
                ++hndl->read_size;
            }

            break;
        }
    }

    return NL_SKIP;
}

struct rap_handle* rap_handle_alloc()
{
    struct rap_handle* hndl = malloc(sizeof(struct rap_handle));
    memset(hndl, 0, sizeof(struct rap_handle));

    return hndl;
}

int rap_init(struct rap_handle* hndl)
{
    if (!hndl)
        return -RAP_EINVAL;

    hndl->errno = 0;

    hndl->sock = nl_socket_alloc();
    nl_socket_disable_seq_check(hndl->sock);
    nl_socket_disable_auto_ack(hndl->sock);

    // Connect to socket and resolve GENL family
    genl_connect(hndl->sock);
    hndl->family_id = genl_ctrl_resolve(hndl->sock, "rapdev");

    // Hook receive callback
    nl_socket_modify_cb(hndl->sock, NL_CB_VALID, NL_CB_CUSTOM, &rap_genl_callback, hndl);

    /*** Get RAP device list from kernel ***/

    struct nl_msg* msg = nlmsg_alloc();

    genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ, hndl->family_id, 0, 0, RAP_GENL_LIST_DEVICES, 0);

    int err = nl_send_auto(hndl->sock, msg);
    if (err < 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    err = nl_recvmsgs_default(hndl->sock);
    if (err != 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    return -hndl->errno;
}

int rap_write(struct rap_handle* hndl, uint32_t dev_id, uint8_t reg_id, uint8_t reg_size, void* reg_data)
{
    if (!hndl || !reg_data)
        return -RAP_EINVAL;

    hndl->errno = 0;

    struct nl_msg* msg = nlmsg_alloc();

    genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ, hndl->family_id, 0, 0, RAP_GENL_WRITE_REG, 0);

    NLA_PUT_U32(msg, RAP_GENL_ATTR_DEV_ID, dev_id);
    NLA_PUT_U8(msg, RAP_GENL_ATTR_REG_ID, reg_id);
    NLA_PUT_U8(msg, RAP_GENL_ATTR_REG_SIZE, reg_size);
    nla_put(msg, RAP_GENL_ATTR_REG_DATA, reg_size, reg_data);

    int err = nl_send_auto(hndl->sock, msg);
    if (err < 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    err = nl_recvmsgs_default(hndl->sock);
    if (err != 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    return -hndl->errno;

    nla_put_failure:
    nlmsg_free(msg);
    hndl->nl_errno = 0;
    hndl->errno = RAP_ENL;
    return -hndl->errno;
}

int rap_read(struct rap_handle* hndl, uint32_t dev_id, uint8_t reg_id, void* reg_data, int max_reg_size)
{
    if (!hndl || !reg_data)
        return -RAP_EINVAL;

    hndl->errno = 0;
    hndl->read_data = reg_data;
    hndl->max_read_size = max_reg_size;

    struct nl_msg* msg = nlmsg_alloc();

    genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ, hndl->family_id, 0, 0, RAP_GENL_READ_REG, 0);

    NLA_PUT_U32(msg, RAP_GENL_ATTR_DEV_ID, dev_id);
    NLA_PUT_U8(msg, RAP_GENL_ATTR_REG_ID, reg_id);

    int err = nl_send_auto(hndl->sock, msg);
    if (err < 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    err = nl_recvmsgs_default(hndl->sock);
    if (err != 0)
    {
        hndl->nl_errno = err;
        hndl->errno = RAP_ENL;
        return -hndl->errno;
    }

    return hndl->errno ? -hndl->errno : hndl->read_size;

    nla_put_failure:
    nlmsg_free(msg);
    hndl->nl_errno = 0;
    hndl->errno = RAP_ENL;
    return -hndl->errno;
}

int main(int argc, char** argv)
{
    struct rap_handle* hndl = rap_handle_alloc();
    int err = rap_init(hndl);

    printf("err = %d\n", err);
    printf("Got %d RAP devices\n", hndl->dev_count);

    for (int i = 0; i < hndl->dev_count; ++i)
    {
        printf("Id #%d = %d\n", i, hndl->dev_ids[i]);
    }

    err = rap_write(hndl, hndl->dev_ids[0], 0xAB, 3, "ab");
    printf("rap_write(...) = %d\n", err);

    uint8_t data[256];
    err = rap_read(hndl, hndl->dev_ids[0], 0xAB, &data[0], sizeof(data));
    printf("rap_read(...) = %d / ", err);
    for (int i = 0; i < err; ++i)
        printf("%02X ", data[i]);
    printf("\n");

    return 0;
}
