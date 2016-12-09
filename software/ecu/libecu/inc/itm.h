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

#ifndef ALOS_KPRINT_H
#define ALOS_KPRINT_H

//! Init the ITM module (sets up
//!   SWO and ITP things)
//! \return 0 if successful, -1 otherwise
int itm_init();

//! Print some debug information to the kernel's
//!   debug port (printf-like).
//! \param format The printf-like format string
void __attribute__((format(printf, 1, 2))) itm_printf(const char* fmt, ...);

#endif // ALOS_KPRINT_H
