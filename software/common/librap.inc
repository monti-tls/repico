/*
 * repico/software/common
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

DEF(RAP_CMD_WRITE = 0x01)
DEF(RAP_CMD_READ = 0x02)
DEF(RAP_STATUS_OK = 0xF0)
DEF(RAP_STATUS_INVALID_CMD = 0xF1) // invalid command byte
DEF(RAP_STATUS_INVALID_REG = 0xF2) // no such register id
DEF(RAP_STATUS_INVALID_ACC = 0xF3)  // invalid reg access (R/W issue)
