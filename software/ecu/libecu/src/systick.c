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
#include "libecu/systick.h"

void systick_init()
{
    SysTick->LOAD = 0x00014820;    // interrupt every 500 us (AHB prescaler is 1 so
                                   // its clock is 168 MHz)
    SysTick->VAL = 0x00000000;     // clear current value
    SysTick->CTRL |= (0x01 << 2);  // CLKSOURCE = 1 (processor clock select)
    SysTick->CTRL |= (0x01 << 1);  // TICKINT = 1 (interrupt enabled)
    SysTick->CTRL &= ~(0x01 << 0); // ENABLE = 0 (disabled)

    SCB->SHP[11] = 15 << 0x04; // PRI_15 = 16, minimal priority
}

void systick_start()
{
    SysTick->CTRL |= (0x01 << 0); // ENABLE = 1 (enabled)
}

void systick_stop()
{
    SysTick->CTRL &= ~(0x01 << 0); // ENABLE = 0 (disabled)
}
