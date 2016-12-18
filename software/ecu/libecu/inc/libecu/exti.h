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

#ifndef LIBCEU_EXTI_H
#define LIBCEU_EXTI_H

#include "libecu/platform.h"
#include <stdint.h>

void exti_init(GPIO_TypeDef* port, uint8_t pin, uint8_t priority, uint8_t edge);
uint8_t exti_read(uint8_t line);
void exti_clear(uint8_t line);

#endif // LIBCEU_EXTI_H
