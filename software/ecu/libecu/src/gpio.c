/*
 * repico/software/ecu/libecu
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

void gpio_enable(GPIO_TypeDef* port)
{
    uint8_t bit =
        port == GPIOA ? 0 : (port == GPIOB ? 1 : (port == GPIOC ? 2 : (port == GPIOD ? 3 : (port == GPIOE ? 4 : -1))));

    RCC->AHB1ENR |= (0x01 << bit);
}

void gpio_disable(GPIO_TypeDef* port)
{
    uint8_t bit = port == GPIOA ? 0 : (port == GPIOB ? 1 : (port == GPIOC ? 2 : (port == GPIOD ? 3 : -1)));

    RCC->AHB1ENR &= ~(0x01 << bit);
}

void gpio_init_input(GPIO_TypeDef* port, uint8_t pin)
{
    port->MODER &= ~(0x01 << (2 * pin + 1)); // MODERy[1] = 0
    port->MODER &= ~(0x01 << (2 * pin));     // MODERy[0] = 0
}

void gpio_init_output(GPIO_TypeDef* port, uint8_t pin)
{
    port->MODER &= ~(0x01 << (2 * pin + 1)); // MODERy[1] = 0
    port->MODER |= (0x01 << (2 * pin));      // MODERy[0] = 1
}

void gpio_init_af(GPIO_TypeDef* port, uint8_t pin)
{
    port->MODER |= (0x01 << (2 * pin + 1)); // MODERy[1] = 1
    port->MODER &= ~(0x01 << (2 * pin));    // MODERy[0] = 0
}

void gpio_init_analog(GPIO_TypeDef* port, uint8_t pin)
{
    port->MODER |= (0x01 << (2 * pin + 1)); // MODERy[1] = 1
    port->MODER |= (0x01 << (2 * pin));     // MODERy[0] = 1
}

void gpio_init_pp(GPIO_TypeDef* port, uint8_t pin)
{
    port->OTYPER &= ~(0x01 << pin); // OTy = 0
}

void gpio_init_od(GPIO_TypeDef* port, uint8_t pin)
{
    port->OTYPER |= (0x01 << pin); // OTy = 1
}

void gpio_init_ls(GPIO_TypeDef* port, uint8_t pin)
{
    port->OSPEEDR &= ~(0x01 << (2 * pin + 1)); // OSPEEDy[1] = 0
    port->OSPEEDR &= ~(0x01 << (2 * pin));     // OSPEEDy[0] = 0
}

void gpio_init_ms(GPIO_TypeDef* port, uint8_t pin)
{
    port->OSPEEDR &= ~(0x01 << (2 * pin + 1)); // OSPEEDy[1] = 0
    port->OSPEEDR |= (0x01 << (2 * pin));      // OSPEEDy[0] = 1
}

void gpio_init_fs(GPIO_TypeDef* port, uint8_t pin)
{
    port->OSPEEDR |= (0x01 << (2 * pin + 1)); // OSPEEDy[1] = 1
    port->OSPEEDR &= ~(0x01 << (2 * pin));    // OSPEEDy[0] = 0
}

void gpio_init_hs(GPIO_TypeDef* port, uint8_t pin)
{
    port->OSPEEDR |= (0x01 << (2 * pin + 1)); // OSPEEDy[1] = 1
    port->OSPEEDR |= (0x01 << (2 * pin));     // OSPEEDy[0] = 1
}

void gpio_init_nopupd(GPIO_TypeDef* port, uint8_t pin)
{
    port->PUPDR &= ~(0x01 << (2 * pin + 1)); // PUPDRy[1] = 0
    port->PUPDR &= ~(0x01 << (2 * pin));     // PUPDRy[0] = 0
}

void gpio_init_pu(GPIO_TypeDef* port, uint8_t pin)
{
    port->PUPDR &= ~(0x01 << (2 * pin + 1)); // PUPDRy[1] = 0
    port->PUPDR |= (0x01 << (2 * pin));      // PUPDRy[0] = 1
}

void gpio_init_pd(GPIO_TypeDef* port, uint8_t pin)
{
    port->PUPDR |= (0x01 << (2 * pin + 1)); // PUPDRy[1] = 1
    port->PUPDR &= ~(0x01 << (2 * pin));    // PUPDRy[0] = 0
}

void gpio_select_af(GPIO_TypeDef* port, uint8_t pin, uint8_t afno)
{
    uint8_t off = pin < 8 ? 0 : 1;
    uint32_t id = pin < 8 ? pin : pin - 8;

    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        if ((afno >> i) & 0x01)
            port->AFR[off] |= (0x01 << (4 * id + i));
        else
            port->AFR[off] &= ~(0x01 << (4 * id + i));
    }
}

uint8_t gpio_read(GPIO_TypeDef* port, uint8_t pin)
{
    return (port->IDR & (0x01 << pin)) >> pin;
}

void gpio_write(GPIO_TypeDef* port, uint8_t pin, uint8_t val)
{
    if (val != 0)
        port->ODR |= (0x01 << pin);
    else
        port->ODR &= ~(0x01 << pin);
}
