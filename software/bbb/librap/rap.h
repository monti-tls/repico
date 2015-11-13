#ifndef LIBRAP_RAP_H
#define LIBRAP_RAP_H

#include <netlink/netlink.h>

#define DEF(n) n,
enum
{
    #include "../../common/defs/rap_errno.def"
};
#undef DEF

struct rap_handle
{
    struct nl_sock* sock;
    int family_id;
    int errno;
    int nl_errno;

    int dev_count;
    uint32_t* dev_ids;

    void* read_data;
    int max_read_size;
    int read_size;
};

struct rap_handle* rap_handle_alloc();
int rap_init(struct rap_handle* hndl);
int rap_write(struct rap_handle* hndl, uint32_t dev_id, uint8_t reg_id, uint8_t reg_size, void* reg_data);
int rap_read(struct rap_handle* hndl, uint32_t dev_id, uint8_t reg_id, void* reg_data, int max_reg_size);

#endif // LIBRAP_RAP_H
