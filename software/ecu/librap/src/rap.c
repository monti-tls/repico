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

#include "libecu/platform.h"
#include "libecu/gpio.h"
#include "libecu/exti.h"
#include "libecu/itm.h"
#include "librap/rap.h"

static struct rap_device rap;

static struct rap_register* rap_find_register(struct rap_device* rap, uint8_t reg)
{
    for (struct rap_register* r = rap->registers; r->id; ++r)
    {
        if (r->id == reg)
            return r;
    }

    return 0;
}

int rap_init(struct rap_register* registers, void(*irq_exti15_10_handler)())
{
    // Enable GPIOs
    gpio_enable(GPIOB);
    gpio_enable(GPIOC);
    
    // Configure GPIOs to AF5
    gpio_init_input(GPIOB, 12);
    gpio_init_pu(GPIOB, 12);
    gpio_init_hs(GPIOB, 12);

    gpio_init_af(GPIOB, 10);
    gpio_select_af(GPIOB, 10, 5);
    
    gpio_init_af(GPIOC, 2);
    gpio_select_af(GPIOC, 2, 5);
    
    gpio_init_af(GPIOC, 3);
    gpio_select_af(GPIOC, 3, 5);

    // Enable SPI2 clock
    RCC->APB1ENR |= (0x01 << 14);

    // SPI2 is interrupt #36
    NVIC->ISER[1] |= (0x01 << 4);
    NVIC->IP[36] = (0 << 4);

    rap.spi = SPI2;
    rap.spi_nss.port = GPIOB;
    rap.spi_nss.pin = 12;
    rap.registers = registers;
    rap.state = RAP_IDLE;
    rap.rx_len = 0;
    rap.tx_len = 0;
    rap.tx_ptr = 0;
    rap.irq_exti15_10_handler = irq_exti15_10_handler;

    // DFF = 0 (8 bit frames)
    SPI2->CR1 &= ~(0x01 << 11);

    // CPOL = 0 (SCK 0 when idle)
    SPI2->CR1 &= ~(0x01 << 1);
    // CPHA = 0 (first clock transition is first data capture)
    SPI2->CR1 |= (0x01 << 0);

    // LSBFIRST = 0
    SPI2->CR1 &= ~(0x01 << 7);
    // SSM = 1 (software NSS)
    SPI2->CR1 |= (0x01 << 9);
    SPI2->CR1 &= ~(0x01 << 8);

    // MSTR = 0 (slave mode)
    SPI2->CR1 &= ~(0x01 << 2);

    // TXEIE = 1
    SPI2->CR2 |= (0x01 << 7);
    // RXNEIE = 1
    SPI2->CR2 |= (0x01 << 6);
    // ERRIE = 1
    SPI2->CR2 |= (0x01 << 5);

    // Configure an EXTI on the NSS line
    exti_init(GPIOB, 12, 0, 1);

    // SPE = 1 (enable SPI2)
    SPI2->CR1 |= (0x01 << 6);

    return 0;
}

void irq_exti15_10_handler()
{
    NVIC->ICPR[1] |= (0x01 << 8);

    if (rap.irq_exti15_10_handler)
        (*rap.irq_exti15_10_handler)();

    if (exti_read(12))
    {
        exti_clear(12);

        if (rap.rx_len)
        {
            switch (rap.state)
            {
                case RAP_IDLE:
                    rap.state = RAP_COMMAND;
                    
                    rap.tx_len = 1;
                    rap.tx[0] = rap.rx[0]; // sync byte
                    
                    if (rap.rx[1] == RAP_CMD_WRITE)
                    {
                        struct rap_register* reg = rap_find_register(&rap, rap.rx[2]);

                        uint8_t status = RAP_STATUS_OK;
                        if (!reg)
                            status = RAP_STATUS_INVALID_REG;
                        else if (!(reg->attrs & RAP_REG_W))
                            status = RAP_STATUS_INVALID_ACC;
                        else
                        {
                            if (reg->mode == RAP_REG_BUFFER)
                            {
                                if (rap.rx[3]  != reg->len)
                                    status = RAP_STATUS_INVALID_ACC;
                                else
                                    __builtin_memcpy(reg->buffer, &rap.rx[4], rap.rx[3]);
                            }
                            else
                                (*reg->write)(&rap.rx[4], rap.rx[3]);
                        }

                        rap.tx[rap.tx_len++] = status;
                        rap.tx[rap.tx_len++] = 0; // 0 more bytes to read
                    }
                    else if (rap.rx[1] == RAP_CMD_READ)
                    {
                        struct rap_register* reg = rap_find_register(&rap, rap.rx[2]);

                        if (!reg || !(reg->attrs & RAP_REG_R))
                        {
                            rap.tx[rap.tx_len++] = reg ? RAP_STATUS_INVALID_ACC : RAP_STATUS_INVALID_REG;
                            rap.tx[rap.tx_len++] = 0;
                        }
                        else
                        {
                            rap.tx[rap.tx_len++] = RAP_STATUS_OK;
                            uint8_t* len = &rap.tx[rap.tx_len++];

                            if (reg->mode == RAP_REG_BUFFER)
                            {
                                __builtin_memcpy(&rap.tx[rap.tx_len], reg->buffer, reg->len);
                                *len = reg->len;
                            }
                            else
                                *len = (*reg->read)(&rap.tx[rap.tx_len]);

                            rap.tx_len += *len;
                        }
                    }
                    else
                    {
                        rap.tx[rap.tx_len++] = RAP_STATUS_INVALID_CMD;
                        rap.tx[rap.tx_len++] = 0; // 0 more bytes to read
                    }
                    break;

                case RAP_COMMAND:
                    if (!rap.tx_len)
                        rap.state = RAP_IDLE;
                    break;
            }

            rap.rx_len = 0;
        }
    }
}

void irq_spi2_handler()
{
    NVIC->ICPR[1] |= (0x01 << 4);

    volatile uint16_t sr = rap.spi->SR;

    if ((sr & (0x01 << 3)) || (sr & (0x01 << 6))) // UDR == 1 or OVR == 1
    {
        itm_printf("Error: 0x%08X\n", sr);

        // Clear error flag using this software sequence
        sr = rap.spi->DR;
        sr = rap.spi->SR;
    }
    else if (sr & (0x01 << 0)) // RXNE == 1
    {
        volatile uint16_t dr = rap.spi->DR;
        rap.rx[rap.rx_len++] = dr;
    }
    else if (sr & (0x01 << 1)) // TXE == 1
    {
        if (rap.tx_len)
        {
            rap.spi->DR = rap.tx[rap.tx_ptr++];

            if (rap.tx_ptr == rap.tx_len)
            {
                rap.tx_len = 0;
                rap.tx_ptr = 0;
            }
        }
        else
            rap.spi->DR = 0x00;
    }
}
