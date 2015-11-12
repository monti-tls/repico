#include "libecu/platform.h"
#include "libecu/itm.h"
#include "libecu/gpio.h"
#include "librap/rap.h"

uint8_t reg0_read(uint8_t* data)
{
    data[0] = 0xCA;
    data[1] = 0xFE;
    return 2;
}

void reg0_write(uint8_t* data, uint8_t len)
{
    itm_printf("Write reg0 ");
    for (int i = 0; i < len; ++i)
        itm_printf("%02X ", data[i]);
    itm_printf("\n");
}

int reg1;

struct rap_register registers[] =
{
    {
        0xAB,
        RAP_REG_R | RAP_REG_W,
        RAP_REG_HANDLER,
        0, 0,
        &reg0_read,
        &reg0_write
    },
    {
        0xAC,
        RAP_REG_R | RAP_REG_W,
        RAP_REG_BUFFER,
        &reg1,
        sizeof(reg1),
        0, 0
    },
    { 0x00 }
};

int main()
{
    itm_init();

    rap_init(&registers[0], 0);

    for (;;);
}
