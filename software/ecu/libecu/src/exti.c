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

#include "libecu/exti.h"

void exti_init(GPIO_TypeDef* port, uint8_t pin, uint8_t priority, uint8_t edge)
{
    RCC->APB2ENR |= (0x01 << 14); // SYSCFGEN = 1
    
    uint8_t no_it;
    if (pin == 0)
        no_it = 6;
    else if (pin == 1)
        no_it = 7;
    else if (pin == 2)
        no_it = 8;
    else if (pin == 3)
        no_it = 9;
    else if (pin == 4)
        no_it = 10;
    else if (pin > 4 && pin < 10)
        no_it = 23;
    else if (pin > 9 && pin < 16)
        no_it = 40;
    
    EXTI->IMR |= (1 << pin);
    
    if (edge == 2)
    {
        EXTI->RTSR |= (1 << pin);
        EXTI->FTSR |= (1 << pin);
    }
    else if (edge == 1)
        EXTI->RTSR |= (1 << pin);
    else
        EXTI->FTSR |= (1 << pin);
    
    volatile uint32_t* reg;
    uint8_t idx;
    
    if (pin <= 3)
    {
        reg = &SYSCFG->EXTICR[0];
        idx = pin;
    }
    else if (pin <= 7)
    {
        reg = &SYSCFG->EXTICR[1];
        idx = pin - 4;
    }
    else if (pin <= 11)
    {
        reg = &SYSCFG->EXTICR[2];
        idx = pin - 8;
    }
    else if (pin <= 15)
    {
        reg = &SYSCFG->EXTICR[3];
        idx = pin - 12;
    }
    
    *reg &= ~((0x01 << (4*idx+0)) | (0x01 << (4*idx+1)) | (0x01 << (4*idx+2)) | (0x01 << (4*idx+3)));
    
    if (port == GPIOB)
        *reg |= (0x01 << (4*idx+0));
    else if (port == GPIOC)
        *reg |= (0x01 << (4*idx+1));
    
    if (no_it < 32)
        NVIC->ISER[0] |= (0x01 << no_it);
    else
        NVIC->ISER[1] |= (0x01 << (no_it-32));
    
    NVIC->IP[no_it] = priority << 4;
}

uint8_t exti_read(uint8_t line)
{
    return (EXTI->PR & (0x01 << line)) >> line;
}

void exti_clear(uint8_t line)
{
    EXTI->PR |= (0x01 << line);
}
