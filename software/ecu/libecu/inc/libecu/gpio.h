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

#ifndef LIBCEU_GPIO_H
#define LIBCEU_GPIO_H

#include "libecu/platform.h"
#include <stdint.h>

void gpio_enable(GPIO_TypeDef* port);
void gpio_disable(GPIO_TypeDef* port);

void gpio_init_input(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_output(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_af(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_analog(GPIO_TypeDef* port, uint8_t pin);

void gpio_init_pp(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_od(GPIO_TypeDef* port, uint8_t pin);

void gpio_init_ls(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_ms(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_fs(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_hs(GPIO_TypeDef* port, uint8_t pin);

void gpio_init_nopupd(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_pu(GPIO_TypeDef* port, uint8_t pin);
void gpio_init_pd(GPIO_TypeDef* port, uint8_t pin);

void gpio_select_af(GPIO_TypeDef* port, uint8_t pin, uint8_t afno);

uint8_t gpio_read(GPIO_TypeDef* port, uint8_t pin);
void gpio_write(GPIO_TypeDef* port, uint8_t pin, uint8_t val);

#endif // LIBCEU_GPIO_H
