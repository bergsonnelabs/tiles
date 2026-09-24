/**
 * Startup file for STM32L422xx
 * Minimal vector table + Reset_Handler with .data/.bss init
 *
 * This is a clean, hand-written startup — no CubeIDE code generation.
 */

    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .thumb

/* Symbols from linker script */
.word _sidata       /* Start of .data initializers in FLASH */
.word _sdata        /* Start of .data in SRAM */
.word _edata        /* End of .data in SRAM */
.word _sbss         /* Start of .bss in SRAM */
.word _ebss         /* End of .bss in SRAM */
.word _estack        /* Initial stack pointer (top of SRAM) */

/**
 * Reset_Handler — called on power-on or reset.
 * Copies .data from FLASH to SRAM, zeros .bss, then calls main().
 */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer (redundant — hardware does this from vector[0],
       but nice for debugger resets) */
    ldr r0, =_estack
    mov sp, r0

    /* Enable FPU (CP10 + CP11 full access). The build is -mfloat-abi=hard,
       so without this the first float instruction is a NOCP UsageFault. */
    ldr r0, =0xE000ED88    /* SCB->CPACR */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20)
    str r1, [r0]
    dsb
    isb

    /* Copy .data section from FLASH to SRAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    movs r3, #0
    b .Ldata_check
.Ldata_copy:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4
.Ldata_check:
    adds r4, r0, r3
    cmp r4, r1
    bcc .Ldata_copy

    /* Zero-fill .bss section */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    b .Lbss_check
.Lbss_zero:
    str r2, [r0]
    adds r0, r0, #4
.Lbss_check:
    cmp r0, r1
    bcc .Lbss_zero

    /* Call main() */
    bl main

    /* If main() returns, loop forever */
.Lhang:
    b .Lhang

    .size Reset_Handler, .-Reset_Handler

/**
 * Default handler for unimplemented interrupts — infinite loop.
 */
    .section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b Default_Handler
    .size Default_Handler, .-Default_Handler

/**
 * Vector table — placed at 0x0000_0000 in FLASH via .isr_vector section.
 *
 * STM32L422 has Cortex-M4 system exceptions + peripheral interrupts.
 * We define all of them as weak aliases to Default_Handler so any can
 * be overridden by simply defining the function in C.
 */
    .section .isr_vector, "a", %progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    /* Cortex-M4 system exceptions */
    .word _estack                   /*  0: Initial stack pointer */
    .word Reset_Handler             /*  1: Reset */
    .word NMI_Handler               /*  2: NMI */
    .word HardFault_Handler         /*  3: Hard fault */
    .word MemManage_Handler         /*  4: Memory management fault */
    .word BusFault_Handler          /*  5: Bus fault */
    .word UsageFault_Handler        /*  6: Usage fault */
    .word 0                         /*  7: Reserved */
    .word 0                         /*  8: Reserved */
    .word 0                         /*  9: Reserved */
    .word 0                         /* 10: Reserved */
    .word SVC_Handler               /* 11: SVCall */
    .word DebugMon_Handler          /* 12: Debug monitor */
    .word 0                         /* 13: Reserved */
    .word PendSV_Handler            /* 14: PendSV */
    .word SysTick_Handler           /* 15: SysTick */

    /* STM32L422 peripheral interrupts */
    .word WWDG_IRQHandler           /*  0: Window Watchdog */
    .word PVD_PVM_IRQHandler        /*  1: PVD/PVM through EXTI */
    .word TAMP_STAMP_IRQHandler     /*  2: Tamper and TimeStamp */
    .word RTC_WKUP_IRQHandler       /*  3: RTC Wakeup */
    .word FLASH_IRQHandler          /*  4: Flash */
    .word RCC_IRQHandler            /*  5: RCC */
    .word EXTI0_IRQHandler          /*  6: EXTI Line 0 */
    .word EXTI1_IRQHandler          /*  7: EXTI Line 1 */
    .word EXTI2_IRQHandler          /*  8: EXTI Line 2 */
    .word EXTI3_IRQHandler          /*  9: EXTI Line 3 */
    .word EXTI4_IRQHandler          /* 10: EXTI Line 4 */
    .word DMA1_Channel1_IRQHandler  /* 11: DMA1 Channel 1 */
    .word DMA1_Channel2_IRQHandler  /* 12: DMA1 Channel 2 */
    .word DMA1_Channel3_IRQHandler  /* 13: DMA1 Channel 3 */
    .word DMA1_Channel4_IRQHandler  /* 14: DMA1 Channel 4 */
    .word DMA1_Channel5_IRQHandler  /* 15: DMA1 Channel 5 */
    .word DMA1_Channel6_IRQHandler  /* 16: DMA1 Channel 6 */
    .word DMA1_Channel7_IRQHandler  /* 17: DMA1 Channel 7 */
    .word ADC1_IRQHandler           /* 18: ADC1 */
    .word CAN1_TX_IRQHandler        /* 19: CAN1 TX */
    .word CAN1_RX0_IRQHandler       /* 20: CAN1 RX0 */
    .word CAN1_RX1_IRQHandler       /* 21: CAN1 RX1 */
    .word CAN1_SCE_IRQHandler       /* 22: CAN1 SCE */
    .word EXTI9_5_IRQHandler        /* 23: EXTI Lines 5-9 */
    .word TIM1_BRK_TIM15_IRQHandler /* 24: TIM1 Break / TIM15 */
    .word TIM1_UP_TIM16_IRQHandler  /* 25: TIM1 Update / TIM16 */
    .word TIM1_TRG_COM_IRQHandler   /* 26: TIM1 Trigger and Commutation */
    .word TIM1_CC_IRQHandler        /* 27: TIM1 Capture Compare */
    .word TIM2_IRQHandler           /* 28: TIM2 */
    .word 0                         /* 29: Reserved */
    .word 0                         /* 30: Reserved */
    .word I2C1_EV_IRQHandler        /* 31: I2C1 Event */
    .word I2C1_ER_IRQHandler        /* 32: I2C1 Error */
    .word I2C2_EV_IRQHandler        /* 33: I2C2 Event */
    .word I2C2_ER_IRQHandler        /* 34: I2C2 Error */
    .word SPI1_IRQHandler           /* 35: SPI1 */
    .word 0                         /* 36: Reserved */
    .word USART1_IRQHandler         /* 37: USART1 */
    .word USART2_IRQHandler         /* 38: USART2 */
    .word USART3_IRQHandler         /* 39: USART3 */
    .word EXTI15_10_IRQHandler      /* 40: EXTI Lines 10-15 */
    .word RTC_Alarm_IRQHandler      /* 41: RTC Alarm */
    .word 0                         /* 42: Reserved */
    .word 0                         /* 43: Reserved */
    .word 0                         /* 44: Reserved */
    .word 0                         /* 45: Reserved */
    .word 0                         /* 46: Reserved */
    .word 0                         /* 47: Reserved */
    .word 0                         /* 48: Reserved */
    .word 0                         /* 49: Reserved */
    .word 0                         /* 50: Reserved */
    .word SPI3_IRQHandler           /* 51: SPI3 */
    .word 0                         /* 52: Reserved */
    .word 0                         /* 53: Reserved */
    .word TIM6_DAC_IRQHandler       /* 54: TIM6 / DAC underrun */
    .word TIM7_IRQHandler           /* 55: TIM7 */
    .word DMA2_Channel1_IRQHandler  /* 56: DMA2 Channel 1 */
    .word DMA2_Channel2_IRQHandler  /* 57: DMA2 Channel 2 */
    .word DMA2_Channel3_IRQHandler  /* 58: DMA2 Channel 3 */
    .word DMA2_Channel4_IRQHandler  /* 59: DMA2 Channel 4 */
    .word DMA2_Channel5_IRQHandler  /* 60: DMA2 Channel 5 */
    .word 0                         /* 61: Reserved */
    .word 0                         /* 62: Reserved */
    .word 0                         /* 63: Reserved */
    .word COMP_IRQHandler           /* 64: COMP1/COMP2 */
    .word LPTIM1_IRQHandler         /* 65: LPTIM1 */
    .word LPTIM2_IRQHandler         /* 66: LPTIM2 */
    .word USB_IRQHandler            /* 67: USB */
    .word DMA2_Channel6_IRQHandler  /* 68: DMA2 Channel 6 */
    .word DMA2_Channel7_IRQHandler  /* 69: DMA2 Channel 7 */
    .word 0                         /* 70: Reserved */
    .word QUADSPI_IRQHandler        /* 71: QuadSPI */
    .word I2C3_EV_IRQHandler        /* 72: I2C3 Event */
    .word I2C3_ER_IRQHandler        /* 73: I2C3 Error */
    .word 0                         /* 74: Reserved */
    .word 0                         /* 75: Reserved */
    .word 0                         /* 76: Reserved */
    .word TSC_IRQHandler            /* 77: Touch Sensing Controller */
    .word 0                         /* 78: Reserved */
    .word AES_IRQHandler            /* 79: AES */
    .word RNG_IRQHandler            /* 80: RNG */
    .word FPU_IRQHandler            /* 81: FPU */
    .word CRS_IRQHandler            /* 82: CRS */

/**
 * Weak aliases — any of these can be overridden by defining the
 * function in C code. By default they all land in Default_Handler.
 */
    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler
    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler
    .weak PVD_PVM_IRQHandler
    .thumb_set PVD_PVM_IRQHandler, Default_Handler
    .weak TAMP_STAMP_IRQHandler
    .thumb_set TAMP_STAMP_IRQHandler, Default_Handler
    .weak RTC_WKUP_IRQHandler
    .thumb_set RTC_WKUP_IRQHandler, Default_Handler
    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler
    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler
    .weak EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler
    .weak EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler
    .weak EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler
    .weak EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler
    .weak EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler
    .weak DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler, Default_Handler
    .weak DMA1_Channel2_IRQHandler
    .thumb_set DMA1_Channel2_IRQHandler, Default_Handler
    .weak DMA1_Channel3_IRQHandler
    .thumb_set DMA1_Channel3_IRQHandler, Default_Handler
    .weak DMA1_Channel4_IRQHandler
    .thumb_set DMA1_Channel4_IRQHandler, Default_Handler
    .weak DMA1_Channel5_IRQHandler
    .thumb_set DMA1_Channel5_IRQHandler, Default_Handler
    .weak DMA1_Channel6_IRQHandler
    .thumb_set DMA1_Channel6_IRQHandler, Default_Handler
    .weak DMA1_Channel7_IRQHandler
    .thumb_set DMA1_Channel7_IRQHandler, Default_Handler
    .weak ADC1_IRQHandler
    .thumb_set ADC1_IRQHandler, Default_Handler
    .weak CAN1_TX_IRQHandler
    .thumb_set CAN1_TX_IRQHandler, Default_Handler
    .weak CAN1_RX0_IRQHandler
    .thumb_set CAN1_RX0_IRQHandler, Default_Handler
    .weak CAN1_RX1_IRQHandler
    .thumb_set CAN1_RX1_IRQHandler, Default_Handler
    .weak CAN1_SCE_IRQHandler
    .thumb_set CAN1_SCE_IRQHandler, Default_Handler
    .weak EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler, Default_Handler
    .weak TIM1_BRK_TIM15_IRQHandler
    .thumb_set TIM1_BRK_TIM15_IRQHandler, Default_Handler
    .weak TIM1_UP_TIM16_IRQHandler
    .thumb_set TIM1_UP_TIM16_IRQHandler, Default_Handler
    .weak TIM1_TRG_COM_IRQHandler
    .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler
    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler
    .weak TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler
    .weak I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler
    .weak I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler
    .weak I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler
    .weak I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler
    .weak SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler
    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler
    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler
    .weak USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler
    .weak EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler, Default_Handler
    .weak RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler
    .weak SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler
    .weak TIM6_DAC_IRQHandler
    .thumb_set TIM6_DAC_IRQHandler, Default_Handler
    .weak TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler
    .weak DMA2_Channel1_IRQHandler
    .thumb_set DMA2_Channel1_IRQHandler, Default_Handler
    .weak DMA2_Channel2_IRQHandler
    .thumb_set DMA2_Channel2_IRQHandler, Default_Handler
    .weak DMA2_Channel3_IRQHandler
    .thumb_set DMA2_Channel3_IRQHandler, Default_Handler
    .weak DMA2_Channel4_IRQHandler
    .thumb_set DMA2_Channel4_IRQHandler, Default_Handler
    .weak DMA2_Channel5_IRQHandler
    .thumb_set DMA2_Channel5_IRQHandler, Default_Handler
    .weak COMP_IRQHandler
    .thumb_set COMP_IRQHandler, Default_Handler
    .weak LPTIM1_IRQHandler
    .thumb_set LPTIM1_IRQHandler, Default_Handler
    .weak LPTIM2_IRQHandler
    .thumb_set LPTIM2_IRQHandler, Default_Handler
    .weak USB_IRQHandler
    .thumb_set USB_IRQHandler, Default_Handler
    .weak DMA2_Channel6_IRQHandler
    .thumb_set DMA2_Channel6_IRQHandler, Default_Handler
    .weak DMA2_Channel7_IRQHandler
    .thumb_set DMA2_Channel7_IRQHandler, Default_Handler
    .weak QUADSPI_IRQHandler
    .thumb_set QUADSPI_IRQHandler, Default_Handler
    .weak I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler
    .weak I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler
    .weak TSC_IRQHandler
    .thumb_set TSC_IRQHandler, Default_Handler
    .weak AES_IRQHandler
    .thumb_set AES_IRQHandler, Default_Handler
    .weak RNG_IRQHandler
    .thumb_set RNG_IRQHandler, Default_Handler
    .weak FPU_IRQHandler
    .thumb_set FPU_IRQHandler, Default_Handler
    .weak CRS_IRQHandler
    .thumb_set CRS_IRQHandler, Default_Handler
