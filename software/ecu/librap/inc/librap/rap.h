/*
 * repico/software/ecu/librap
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

#ifndef LIBRAP_RAP_H
#define LIBRAP_RAP_H

#include "libecu/platform.h"

#define DEF(a) a,
enum
{
    #include "defs/rap_spi.def"
};
#undef DEF

struct rap_register
{
    uint8_t id;
    
    enum
    {
        RAP_REG_R = 0x01,
        RAP_REG_W = 0x02
    } attrs;

    enum
    {
        RAP_REG_BUFFER,
        RAP_REG_HANDLER
    } mode;

    void* buffer;
    int len;
    uint8_t (*read)(uint8_t* data);
    void (*write)(uint8_t* data, uint8_t size);
};

struct rap_device
{
    SPI_TypeDef* spi;
    struct
    {
        GPIO_TypeDef* port;
        int pin;
    } spi_nss;
    struct rap_register* registers;

    uint8_t rx[300]; // hold 256 bytes of payload + command
    unsigned int rx_len;

    uint8_t tx[300];
    unsigned int tx_len;
    unsigned int tx_ptr;

    enum
    {
        RAP_IDLE,
        RAP_COMMAND
    } state;

    void(*irq_exti15_10_handler)();
};

int rap_init(struct rap_register* registers, void(*irq_exti15_10_handler)());

#endif // LIBRAP_RAP_H
