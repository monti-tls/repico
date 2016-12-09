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

.syntax unified
.cpu cortex-m4
.thumb

/***************/
/*** Exports ***/
/***************/

.global  irq_vectors_table
.global  irq_default_handler

/**********************************************/
/*** Constants defined in the linker script ***/
/**********************************************/

.word  _ld_idata_start /* Start address of init values in .data section */
.word  _ld_data_start  /* Start address for the .data section */
.word  _ld_data_end  /* End address for the .data section */
.word  _ld_bss_start   /* start address for the .bss section */
.word  _ld_bss_end   /* end address for the .bss section */

/*********************/
/*** Reset handler ***/
/*********************/

.section .text.irq_reset_handler
.weak  irq_reset_handler
.type  irq_reset_handler, %function

irq_reset_handler:
    movs r1, #0
    b    loop_copy_di

copy_di:
    ldr  r3, =_ld_idata_start /* read .idata start address */
    ldr  r3, [r3, r1]         /* load word from .idata */
    str  r3, [r0, r1]         /* store word in .data */
    adds r1, r1, #4           /* advance by 4 bytes */

loop_copy_di:
    ldr  r0, =_ld_data_start  /* read .data start address */
    ldr  r3, =_ld_data_end    /* read .data end address */
    adds r2, r0, r1           /* loop until all data has been written */
    cmp  r2, r3
    bcc  copy_di
    ldr  r2, =_ld_bss_start   /* TODO: move to line 53 ? */
    b    loop_zero_bss        /* go zeroing .bss */

zero_bss:
    movs r3, #0
    str  r3, [r2], #4         /* store 0 at current .bss address */

loop_zero_bss:
    ldr  r3, = _ld_bss_end    /* loop until all .bss has been zeroed */
    cmp  r2, r3
    bcc  zero_bss

    ldr  r0, =0xE000ED88      /* enable CP10 and CP11 FPU units */
    ldr  r1, [r0]
    orr  r1, r1, #(0xF << 20)
    str  r1,[r0]

    bl   SystemInit           /* go to system.c and initialize clocks */

    bl  main                  /* go to application's entry point */
    b .                       /* just a safety measure in case the main returns (could be reset !) */

.size irq_reset_handler, .-irq_reset_handler

/***************************/
/*** Default IRQ handler ***/
/***************************/

.section .text.irq_default_handler, "ax", %progbits
.type  irq_reset_handler, %function

irq_default_handler:
    b .

.size  irq_default_handler, .-irq_default_handler

/***************************************/
/*** Minimal interrupt vector table  ***/
/*** Must be placed @ phy 0x00000000 ***/
/***************************************/

.section .irq_vectors_section, "a", %progbits
.type  irq_vectors_table, %object

irq_vectors_table:
    /* System interrupts */

    .word  _ld_stack_end
    .word  irq_reset_handler                 /* Reset Vector */
    .word  irq_nmi_handler                   /* NMI */
    .word  irq_hardfault_handler             /* Hard Fault */
    .word  irq_memmanage_handler             /* MemManage */
    .word  irq_busfault_handler              /* Bus Fault */
    .word  irq_usagefault_handler            /* Usage Fault */
    .word  0
    .word  0
    .word  0
    .word  0
    .word  irq_svc_handler                   /* Service Call */
    .word  irq_debugmon_handler              /* DebugMon */
    .word  0
    .word  irq_pendsv_handler                /* PendSV */
    .word  irq_systick_handler               /* SysTick */

    /* External interrupts */
    
    .word     irq_wwdg_handler               /* Window WatchDog */
    .word     irq_pvd_handler                /* PVD through EXTI Line detection */
    .word     irq_tamp_stamp_handler         /* Tamper and TimeStamps through the EXTI line */
    .word     irq_rtc_wkup_handler           /* RTC Wakeup through the EXTI line */
    .word     irq_flash_handler              /* FLASH */
    .word     irq_rcc_handler                /* RCC */
    .word     irq_exti0_handler              /* EXTI Line0 */
    .word     irq_exti1_handler              /* EXTI Line1 */
    .word     irq_exti2_handler              /* EXTI Line2 */
    .word     irq_exti3_handler              /* EXTI Line3 */
    .word     irq_exti4_handler              /* EXTI Line4 */
    .word     irq_dma1_stream0_handler       /* DMA1 Stream 0 */
    .word     irq_dma1_stream1_handler       /* DMA1 Stream 1 */
    .word     irq_dma1_stream2_handler       /* DMA1 Stream 2 */
    .word     irq_dma1_stream3_handler       /* DMA1 Stream 3 */
    .word     irq_dma1_stream4_handler       /* DMA1 Stream 4 */
    .word     irq_dma1_stream5_handler       /* DMA1 Stream 5 */
    .word     irq_dma1_stream6_handler       /* DMA1 Stream 6 */
    .word     irq_adc_handler                /* ADC1, ADC2 and ADC3s */
    .word     irq_can1_tx_handler            /* CAN1 TX */
    .word     irq_can1_rx0_handler           /* CAN1 RX0 */
    .word     irq_can1_rx1_handler           /* CAN1 RX1 */
    .word     irq_can1_sce_handler           /* CAN1 SCE */
    .word     irq_exti9_5_handler            /* External Line[9:5] */
    .word     irq_tim1_brk_tim9_handler      /* TIM1 Break and TIM9 */
    .word     irq_tim1_up_tim10_handler      /* TIM1 Update and TIM10 */
    .word     irq_tim1_trg_com_tim11_handler /* TIM1 Trigger and Commutation and TIM11 */
    .word     irq_tim1_cc_handler            /* TIM1 Capture Compare */
    .word     irq_tim2_handler               /* TIM2 */
    .word     irq_tim3_handler               /* TIM3 */
    .word     irq_tim4_handler               /* TIM4 */
    .word     irq_i2c1_ev_handler            /* I2C1 Event */
    .word     irq_i2c1_er_handler            /* I2C1 Error */
    .word     irq_i2c2_ev_handler            /* I2C2 Event */
    .word     irq_i2c2_er_handler            /* I2C2 Error */
    .word     irq_spi1_handler               /* SPI1 */
    .word     irq_spi2_handler               /* SPI2 */
    .word     irq_usart1_handler             /* USART1 */
    .word     irq_usart2_handler             /* USART2 */
    .word     irq_usart3_handler             /* USART3 */
    .word     irq_exti15_10_handler          /* External Line[15:10] */
    .word     irq_rtc_alarm_handler          /* RTC Alarm (A and B) through EXTI Line */
    .word     irq_otg_fs_wkup_handler        /* USB OTG FS Wakeup through EXTI line */
    .word     irq_tim8_brk_tim12_handler     /* TIM8 Break and TIM12 */
    .word     irq_tim8_up_tim13_handler      /* TIM8 Update and TIM13 */
    .word     irq_tim8_trg_com_tim14_handler /* TIM8 Trigger and Commutation and TIM14 */
    .word     irq_tim8_cc_handler            /* TIM8 Capture Compare */
    .word     irq_dma1_stream7_handler       /* DMA1 Stream7 */
    .word     irq_fsmc_handler               /* FSMC */
    .word     irq_sdio_handler               /* SDIO */
    .word     irq_tim5_handler               /* TIM5 */
    .word     irq_spi3_handler               /* SPI3 */
    .word     irq_uart4_handler              /* UART4 */
    .word     irq_uart5_handler              /* UART5 */
    .word     irq_tim6_dac_handler           /* TIM6 and DAC1&2 underrun errors */
    .word     irq_tim7_handler               /* TIM7 */
    .word     irq_dma2_stream0_handler       /* DMA2 Stream 0 */
    .word     irq_dma2_stream1_handler       /* DMA2 Stream 1 */
    .word     irq_dma2_stream2_handler       /* DMA2 Stream 2 */
    .word     irq_dma2_stream3_handler       /* DMA2 Stream 3 */
    .word     irq_dma2_stream4_handler       /* DMA2 Stream 4 */
    .word     irq_eth_handler                /* Ethernet */
    .word     irq_eth_wkup_handler           /* Ethernet Wakeup through EXTI line */
    .word     irq_can2_tx_handler            /* CAN2 TX */
    .word     irq_can2_rx0_handler           /* CAN2 RX0 */
    .word     irq_can2_rx1_handler           /* CAN2 RX1 */
    .word     irq_can2_sce_handler           /* CAN2 SCE */
    .word     irq_otg_fs_handler             /* USB OTG FS */
    .word     irq_dma2_stream5_handler       /* DMA2 Stream 5 */
    .word     irq_dma2_stream6_handler       /* DMA2 Stream 6 */
    .word     irq_dma2_stream7_handler       /* DMA2 Stream 7 */
    .word     irq_usart6_handler             /* USART6 */
    .word     irq_i2c3_ev_handler            /* I2C3 event */
    .word     irq_i2c3_er_handler            /* I2C3 error */
    .word     irq_otg_hs_ep1_out_handler     /* USB OTG HS End Point 1 Out */
    .word     irq_otg_hs_ep1_in_handler      /* USB OTG HS End Point 1 In */
    .word     irq_otg_hs_wkup_handler        /* USB OTG HS Wakeup through EXTI */
    .word     irq_otg_hs_handler             /* USB OTG HS */
    .word     irq_dcmi_handler               /* DCMI */
    .word     irq_cryp_handler               /* CRYP crypto */
    .word     irq_hash_rng_handler           /* Hash and Rng */
    .word     irq_fpu_handler                /* FPU */

.size  irq_vectors_table, .-irq_vectors_table

/* System interrupts aliasing */

.weak      irq_nmi_handler
.thumb_set irq_nmi_handler, irq_default_handler

.weak      irq_hardfault_handler
.thumb_set irq_hardfault_handler, irq_default_handler

.weak      irq_memmanage_handler
.thumb_set irq_memmanage_handler, irq_default_handler

.weak      irq_busfault_handler
.thumb_set irq_busfault_handler, irq_default_handler

.weak      irq_usagefault_handler
.thumb_set irq_usagefault_handler, irq_default_handler

.weak      irq_svc_handler
.thumb_set irq_svc_handler, irq_default_handler

.weak      irq_debugmon_handler
.thumb_set irq_debugmon_handler, irq_default_handler

.weak      irq_pendsv_handler
.thumb_set irq_pendsv_handler, irq_default_handler

.weak      irq_systick_handler
.thumb_set irq_systick_handler, irq_default_handler

/* External interrupts aliasing */

.weak      irq_wwdg_handler
.thumb_set irq_wwdg_handler, irq_default_handler

.weak      irq_pvd_handler
.thumb_set irq_pvd_handler, irq_default_handler

.weak      irq_tamp_stamp_handler
.thumb_set irq_tamp_stamp_handler, irq_default_handler

.weak      irq_rtc_wkup_handler
.thumb_set irq_rtc_wkup_handler, irq_default_handler

.weak      irq_flash_handler
.thumb_set irq_flash_handler, irq_default_handler

.weak      irq_rcc_handler
.thumb_set irq_rcc_handler, irq_default_handler

.weak      irq_exti0_handler
.thumb_set irq_exti0_handler, irq_default_handler

.weak      irq_exti1_handler
.thumb_set irq_exti1_handler, irq_default_handler

.weak      irq_exti2_handler
.thumb_set irq_exti2_handler, irq_default_handler

.weak      irq_exti3_handler
.thumb_set irq_exti3_handler, irq_default_handler

.weak      irq_exti4_handler
.thumb_set irq_exti4_handler, irq_default_handler

.weak      irq_dma1_stream0_handler
.thumb_set irq_dma1_stream0_handler, irq_default_handler

.weak      irq_dma1_stream1_handler
.thumb_set irq_dma1_stream1_handler, irq_default_handler

.weak      irq_dma1_stream2_handler
.thumb_set irq_dma1_stream2_handler, irq_default_handler

.weak      irq_dma1_stream3_handler
.thumb_set irq_dma1_stream3_handler, irq_default_handler

.weak      irq_dma1_stream4_handler
.thumb_set irq_dma1_stream4_handler, irq_default_handler

.weak      irq_dma1_stream5_handler
.thumb_set irq_dma1_stream5_handler, irq_default_handler

.weak      irq_dma1_stream6_handler
.thumb_set irq_dma1_stream6_handler, irq_default_handler

.weak      irq_adc_handler
.thumb_set irq_adc_handler, irq_default_handler

.weak      irq_can1_tx_handler
.thumb_set irq_can1_tx_handler, irq_default_handler

.weak      irq_can1_rx0_handler
.thumb_set irq_can1_rx0_handler, irq_default_handler

.weak      irq_can1_rx1_handler
.thumb_set irq_can1_rx1_handler, irq_default_handler

.weak      irq_can1_sce_handler
.thumb_set irq_can1_sce_handler, irq_default_handler

.weak      irq_exti9_5_handler
.thumb_set irq_exti9_5_handler, irq_default_handler

.weak      irq_tim1_brk_tim9_handler
.thumb_set irq_tim1_brk_tim9_handler, irq_default_handler

.weak      irq_tim1_up_tim10_handler
.thumb_set irq_tim1_up_tim10_handler, irq_default_handler

.weak      irq_tim1_trg_com_tim11_handler
.thumb_set irq_tim1_trg_com_tim11_handler, irq_default_handler

.weak      irq_tim1_cc_handler
.thumb_set irq_tim1_cc_handler, irq_default_handler

.weak      irq_tim2_handler
.thumb_set irq_tim2_handler, irq_default_handler

.weak      irq_tim3_handler
.thumb_set irq_tim3_handler, irq_default_handler

.weak      irq_tim4_handler
.thumb_set irq_tim4_handler, irq_default_handler

.weak      irq_i2c1_ev_handler
.thumb_set irq_i2c1_ev_handler, irq_default_handler

.weak      irq_i2c1_er_handler
.thumb_set irq_i2c1_er_handler, irq_default_handler

.weak      irq_i2c2_ev_handler
.thumb_set irq_i2c2_ev_handler, irq_default_handler

.weak      irq_i2c2_er_handler
.thumb_set irq_i2c2_er_handler, irq_default_handler

.weak      irq_spi1_handler
.thumb_set irq_spi1_handler, irq_default_handler

.weak      irq_spi2_handler
.thumb_set irq_spi2_handler, irq_default_handler

.weak      irq_usart1_handler
.thumb_set irq_usart1_handler, irq_default_handler

.weak      irq_usart2_handler
.thumb_set irq_usart2_handler, irq_default_handler

.weak      irq_usart3_handler
.thumb_set irq_usart3_handler, irq_default_handler

.weak      irq_exti15_10_handler
.thumb_set irq_exti15_10_handler, irq_default_handler

.weak      irq_rtc_alarm_handler
.thumb_set irq_rtc_alarm_handler, irq_default_handler

.weak      irq_otg_fs_wkup_handler
.thumb_set irq_otg_fs_wkup_handler, irq_default_handler

.weak      irq_tim8_brk_tim12_handler
.thumb_set irq_tim8_brk_tim12_handler, irq_default_handler

.weak      irq_tim8_up_tim13_handler
.thumb_set irq_tim8_up_tim13_handler, irq_default_handler

.weak      irq_tim8_trg_com_tim14_handler
.thumb_set irq_tim8_trg_com_tim14_handler, irq_default_handler

.weak      irq_tim8_cc_handler
.thumb_set irq_tim8_cc_handler, irq_default_handler

.weak      irq_dma1_stream7_handler
.thumb_set irq_dma1_stream7_handler, irq_default_handler

.weak      irq_fsmc_handler
.thumb_set irq_fsmc_handler, irq_default_handler

.weak      irq_sdio_handler
.thumb_set irq_sdio_handler, irq_default_handler

.weak      irq_tim5_handler
.thumb_set irq_tim5_handler, irq_default_handler

.weak      irq_spi3_handler
.thumb_set irq_spi3_handler, irq_default_handler

.weak      irq_uart4_handler
.thumb_set irq_uart4_handler, irq_default_handler

.weak      irq_uart5_handler
.thumb_set irq_uart5_handler, irq_default_handler

.weak      irq_tim6_dac_handler
.thumb_set irq_tim6_dac_handler, irq_default_handler

.weak      irq_tim7_handler
.thumb_set irq_tim7_handler, irq_default_handler

.weak      irq_dma2_stream0_handler
.thumb_set irq_dma2_stream0_handler, irq_default_handler

.weak      irq_dma2_stream1_handler
.thumb_set irq_dma2_stream1_handler, irq_default_handler

.weak      irq_dma2_stream2_handler
.thumb_set irq_dma2_stream2_handler, irq_default_handler

.weak      irq_dma2_stream3_handler
.thumb_set irq_dma2_stream3_handler, irq_default_handler

.weak      irq_dma2_stream4_handler
.thumb_set irq_dma2_stream4_handler, irq_default_handler

.weak      irq_eth_handler
.thumb_set irq_eth_handler, irq_default_handler

.weak      irq_eth_wkup_handler
.thumb_set irq_eth_wkup_handler, irq_default_handler

.weak      irq_can2_tx_handler
.thumb_set irq_can2_tx_handler, irq_default_handler

.weak      irq_can2_rx0_handler
.thumb_set irq_can2_rx0_handler, irq_default_handler

.weak      irq_can2_rx1_handler
.thumb_set irq_can2_rx1_handler, irq_default_handler

.weak      irq_can2_sce_handler
.thumb_set irq_can2_sce_handler, irq_default_handler

.weak      irq_otg_fs_handler
.thumb_set irq_otg_fs_handler, irq_default_handler

.weak      irq_dma2_stream5_handler
.thumb_set irq_dma2_stream5_handler, irq_default_handler

.weak      irq_dma2_stream6_handler
.thumb_set irq_dma2_stream6_handler, irq_default_handler

.weak      irq_dma2_stream7_handler
.thumb_set irq_dma2_stream7_handler, irq_default_handler

.weak      irq_usart6_handler
.thumb_set irq_usart6_handler, irq_default_handler

.weak      irq_i2c3_ev_handler
.thumb_set irq_i2c3_ev_handler, irq_default_handler

.weak      irq_i2c3_er_handler
.thumb_set irq_i2c3_er_handler, irq_default_handler

.weak      irq_otg_hs_ep1_out_handler
.thumb_set irq_otg_hs_ep1_out_handler, irq_default_handler

.weak      irq_otg_hs_ep1_in_handler
.thumb_set irq_otg_hs_ep1_in_handler, irq_default_handler

.weak      irq_otg_hs_wkup_handler
.thumb_set irq_otg_hs_wkup_handler, irq_default_handler

.weak      irq_otg_hs_handler
.thumb_set irq_otg_hs_handler, irq_default_handler

.weak      irq_dcmi_handler
.thumb_set irq_dcmi_handler, irq_default_handler

.weak      irq_cryp_handler
.thumb_set irq_cryp_handler, irq_default_handler

.weak      irq_hash_rng_handler
.thumb_set irq_hash_rng_handler, irq_default_handler

.weak      irq_fpu_handler
.thumb_set irq_fpu_handler, irq_default_handler
