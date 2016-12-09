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

#include "platform.h"
#include "itm.h"

#include <stdarg.h>
#include <string.h>

///////////////////////////////////
//// Module internal functions ////
///////////////////////////////////

//! Output a single character
//! \param buf Address of the buffer to output to (may be null)
//! \param c The character to print
//! \return 0 if OK, -1 otherwise
static int itm_putc(char** buf, int c)
{
    if (buf && *buf)
    {
        **buf = c;
        (*buf)++;
    }
    else
    {
        if ((ITM->TCR & (0x01 << 0)) && (ITM->TER & (0x01 << 0)))
        {
            while (ITM->PORT[0].u32 == 0)
                ;
            ITM->PORT[0].u8 = (uint8_t)c;
            return 0;
        }

        return -1;
    }

    return 0;
}

//! Output a string
//! \param buf Address of the buffer to output to (may be null)
//! \param s The string to print (may be null)
//! \param pad_right Set to 1 if you want to pad to the right
//! \param pad_char The character to pad with
//! \param pad_width The width of the pad (if no pad is desired, set
//!                  set to 0 or less
//! \return 0 if OK, -1 otherwise
static int itm_puts(char** buf, const char* s, int pad_right, int pad_char, int pad_width)
{
    if (!s)
        s = "(null)";
    int slen = strlen(s);

    // If left pad
    if (pad_width > 0 && !pad_right)
    {
        for (int i = 0; i < (pad_width - slen); ++i)
            if (itm_putc(buf, pad_char) < 0)
                return -1;
    }

    // Output string
    for (int i = 0; i < slen; ++i)
        if (itm_putc(buf, s[i]) < 0)
            return -1;

    // If right pad
    if (pad_width > 0 && pad_right)
    {
        for (int i = 0; i < (pad_width - slen); ++i)
            if (itm_putc(buf, pad_char) < 0)
                return -1;
    }

    return 0;
}

//! Print out an integer value
//! \param buf Address of the buffer to output to (may be null)
//! \param x The number to print
//! \param base The numeral base to use
//! \param base_char The first character after '9' to use (for base > 10)
//! \param pad_right Set to 1 if you want to pad to the right
//! \param pad_char The character to pad with
//! \param pad_width The width of the pad (if no pad is desired, set
//!                  set to 0 or less
//! \return 0 if OK, -1 otherwise
static int itm_puti(char** buf, int x, int sign, int base, int base_char, int pad_right, int pad_char, int pad_width)
{
    char scratch[16];
    char* p = scratch;
    int neg = 0;
    unsigned int u = x;

    // Trivial case
    if (x == 0)
    {
        *(p++) = '0';
        *(p++) = '\0';
        return itm_puts(buf, scratch, pad_right, pad_char, pad_width);
    }

    // Negative ?
    if (sign && x < 0)
    {
        neg = 1;
        u = -x;
    }

    // Initialize buffer
    p = scratch + sizeof(scratch) - 1;
    *(p--) = '\0';

    // Output number
    while (u)
    {
        int digit = u % base;
        if (digit >= 10)
        {
            digit += base_char - '0' - 10;
        }
        *(p--) = '0' + digit;
        u /= base;
    }

    if (neg)
    {
        if (pad_width > 0 && !pad_right)
        {
            if (itm_putc(buf, '-') < 0)
            {
                return -1;
            }
            --pad_width;
        }
        else
        {
            *(p--) = '-';
        }
    }

    return itm_puts(buf, p + 1, pad_right, pad_char, pad_width);
}

//! AP version of itm_printf
//! \param buf Address of the buffer to output to (may be null,
//!            used for string printfs)
//! \param fmt Format string (printf-like)
//! \param ap The stdarg'ed variable argument list
//! \param return 0 if OK, -1 otherwise
static int itm_vsprintf(char* buf, const char* fmt, va_list ap)
{
    int pad_right = 0;
    int pad_char = ' ';
    int pad_width = -1;

    for (const char* fp = fmt; *fp; ++fp)
    {
        if (*fp == '%')
        {
            ++fp;

            if (*fp == '%')
            {
                if (itm_putc(&buf, *fp) < 0)
                {
                    return -1;
                }
            }
            else
            {
                // Pad alignment
                if (*fp == '-')
                {
                    pad_right = 1;
                    ++fp;
                }

                // Pad char
                if (*fp == '0')
                {
                    pad_char = '0';
                    ++fp;
                }

                // Pad width
                if (*fp == '*')
                {
                    pad_width = va_arg(ap, int);
                    ++fp;
                }
                else
                {
                    for (; *fp >= '0' && *fp <= '9'; ++fp)
                    {
                        if (pad_width < 0)
                            pad_width = 0;
                        else
                            pad_width *= 10;

                        pad_width += *fp - '0';
                    }
                }

                // String
                if (*fp == 's')
                {
                    if (itm_puts(&buf, va_arg(ap, const char*), pad_right, pad_char, pad_width) < 0)
                    {
                        return -1;
                    }
                }
                // Signed integer
                else if (*fp == 'd')
                {
                    if (itm_puti(&buf, va_arg(ap, int), 1, 10, '0', pad_right, pad_char, pad_width) < 0)
                    {
                        return -1;
                    }
                }
                // Unigned integer
                else if (*fp == 'u')
                {
                    if (itm_puti(&buf, va_arg(ap, unsigned int), 0, 10, '0', pad_right, pad_char, pad_width) < 0)
                    {
                        return -1;
                    }
                }
                // Headecimal lowercase
                else if (*fp == 'x' || *fp == 'p')
                {
                    if (itm_puti(&buf, va_arg(ap, unsigned int), 0, 16, 'a', pad_right, pad_char, pad_width) < 0)
                    {
                        return -1;
                    }
                }
                // Headecimal uppercase
                else if (*fp == 'X' || *fp == 'P')
                {
                    if (itm_puti(&buf, va_arg(ap, unsigned int), 0, 16, 'A', pad_right, pad_char, pad_width) < 0)
                    {
                        return -1;
                    }
                }
            }
        }
        else
        {
            if (itm_putc(&buf, *fp) < 0)
            {
                return -1;
            }
        }
    }

    return 0;
}

///////////////////////////
//// Public module API ////
///////////////////////////

int itm_init()
{
    CoreDebug->DEMCR |= (0x01 << 24); // TRCENA = 1

    *((unsigned int*)0xE0000FB0) = 0xC5ACCE55; // Unlock ITM registers

    ITM->TCR |= (0x01 << 0); // ITMENA = 1
    ITM->TCR |= (0x01 << 4); // SWOENA = 1
    ITM->TCR |= (0x01 << 3); // TXENA = 1

    ITM->TER |= (0x01 << 0); // Enable channel 0
    ITM->TPR |= (0x01 << 0); // Unmask channels 0:7

    return 0;
}

void itm_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    itm_vsprintf(0, fmt, ap);
    va_end(ap);
}
