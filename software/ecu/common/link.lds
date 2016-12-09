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

/*******************/
/*** Entry Point ***/
/*******************/

ENTRY(irq_reset_handler)

/****************************/
/*** Constant definitions ***/
/****************************/

/* Highest address of the user mode stack :
 * end of 128K RAM (AHB bus) */
_ld_stack_end = 0x20020000;

/************************************/
/*** Physical memory area regions ***/
/************************************/

MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 192K
  MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

/***********************************/
/*** Output sections definitions ***/
/***********************************/

SECTIONS
{
    /* The startup code goes first into FLASH */
    .irq_vectors_section :
    {
        . = ALIGN(4);

        KEEP(*(.irq_vectors_section))

        . = ALIGN(4);
    } >FLASH

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);

        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);

        _ld_text_end = .;        /* define a global symbols at end of code */
        _exit = .;
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array     :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(.fini_array*))
        KEEP (*(SORT(.fini_array.*)))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* used by the startup to initialize data */
    _ld_idata_start = .;

    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data : AT (_ld_idata_start)
    {
        . = ALIGN(4);
        _ld_data_start = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _ld_data_end = .;        /* define a global symbol at data end */
    } >RAM

    /* Uninitialized data section */
    . = ALIGN(4);
    .bss :
    {
        /* This is used by the startup in order to initialize the .bss secion */
        _ld_bss_start = .;         /* define a global symbol at bss start */
        __bss_start__ = _ld_bss_start;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ld_bss_end = .;         /* define a global symbol at bss end */
        __bss_end__ = _ld_bss_end;
    } >RAM

    /* MEMORY_bank1 section, code must be located here explicitly            */
    /* Example: extern int foo(void) __attribute__ ((section (".mb1text"))); */
    .memory_b1_text :
    {
        *(.mb1text)        /* .mb1text sections (code) */
        *(.mb1text*)       /* .mb1text* sections (code)  */
        *(.mb1rodata)      /* read-only data (constants) */
        *(.mb1rodata*)
    } >MEMORY_B1

    /* Remove information from the standard libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
