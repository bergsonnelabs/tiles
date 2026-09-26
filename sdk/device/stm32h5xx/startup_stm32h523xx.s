/**
 * Startup file for STM32H523xx
 * Minimal vector table + Reset_Handler with .data/.bss init
 *
 * This is a clean, hand-written startup — no CubeIDE code generation.
 */

    .syntax unified
    .cpu cortex-m33
    .fpu fpv5-sp-d16
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
    /* Start as if from a reset even when entered by a jump. The ST ROM
       bootloader's DFU "leave" and the serial-update flasher both jump here
       without resetting the chip, and would otherwise leave their own
       vector table, masked interrupts, NVIC enables and SysTick behind. */
    cpsid i
    movs r0, #0
    msr control, r0                 /* thread mode on MSP, privileged */
    msr basepri, r0
    isb
    ldr r0, =_estack
    msr msp, r0
    ldr r0, =0xE000ED08             /* SCB->VTOR = this image's table */
    ldr r1, =g_pfnVectors
    str r1, [r0]
    ldr r0, =0xE000E010             /* SysTick CTRL = 0 */
    movs r1, #0
    str r1, [r0]
    ldr r0, =0xE000ED94             /* MPU CTRL = 0 (off, its reset value) */
    str r1, [r0]
    ldr r0, =0xE000E180             /* NVIC ICER0..7: disable every IRQ */
    ldr r1, =0xE000E280             /* NVIC ICPR0..7: clear every pending */
    mvn r2, #0
    movs r3, #8
.Lnvic_clr:
    str r2, [r0], #4
    str r2, [r1], #4
    subs r3, r3, #1
    bne .Lnvic_clr
    dsb
    isb
    cpsie i

    /* Enable FPU (CP10 + CP11 full access) */
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
 * STM32H523 has Cortex-M33 system exceptions + peripheral interrupts.
 * We define all of them as weak aliases to Default_Handler so any can
 * be overridden by simply defining the function in C.
 */
    .section .isr_vector, "a", %progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    /* Cortex-M33 system exceptions */
    .word _estack                   /*  0: Initial stack pointer */
    .word Reset_Handler             /*  1: Reset */
    .word NMI_Handler               /*  2: NMI */
    .word HardFault_Handler         /*  3: Hard fault */
    .word MemManage_Handler         /*  4: Memory management fault */
    .word BusFault_Handler          /*  5: Bus fault */
    .word UsageFault_Handler        /*  6: Usage fault */
    .word SecureFault_Handler       /*  7: Secure fault */
    .word 0                         /*  8: Reserved */
    .word 0                         /*  9: Reserved */
    .word 0                         /* 10: Reserved */
    .word SVC_Handler               /* 11: SVCall */
    .word DebugMon_Handler          /* 12: Debug monitor */
    .word 0                         /* 13: Reserved */
    .word PendSV_Handler            /* 14: PendSV */
    .word SysTick_Handler           /* 15: SysTick */

    /* STM32H523 peripheral interrupts: RM0481 Rev 4 Table 147 (STM32H523/533xx
     * vector table), positions 0-132. Every position is listed, reserved ones
     * as 0, so each handler lands in its own slot. Until 2026-09-26 this table
     * was copied from the H503: from position 27 on it was shifted (IWDG at 27,
     * GPDMA1 at 29) and it stopped at 74, so CRS, I2C3, ICACHE, RNG and I3C
     * went to whatever followed the table. Cross-checked against IRQn_Type in
     * CMSIS stm32h523xx.h (it omits SAES/OTFDEC1/AES, which the H523 lacks). */
    .word WWDG_IRQHandler                 /*   0: WWDG */
    .word PVD_AVD_IRQHandler              /*   1: PVD_AVD */
    .word RTC_IRQHandler                  /*   2: RTC */
    .word RTC_S_IRQHandler                /*   3: RTC_S */
    .word TAMP_IRQHandler                 /*   4: TAMP */
    .word RAMCFG_IRQHandler               /*   5: RAMCFG */
    .word FLASH_IRQHandler                /*   6: FLASH */
    .word FLASH_S_IRQHandler              /*   7: FLASH_S */
    .word GTZC_IRQHandler                 /*   8: GTZC */
    .word RCC_IRQHandler                  /*   9: RCC */
    .word RCC_S_IRQHandler                /*  10: RCC_S */
    .word EXTI0_IRQHandler                /*  11: EXTI0 */
    .word EXTI1_IRQHandler                /*  12: EXTI1 */
    .word EXTI2_IRQHandler                /*  13: EXTI2 */
    .word EXTI3_IRQHandler                /*  14: EXTI3 */
    .word EXTI4_IRQHandler                /*  15: EXTI4 */
    .word EXTI5_IRQHandler                /*  16: EXTI5 */
    .word EXTI6_IRQHandler                /*  17: EXTI6 */
    .word EXTI7_IRQHandler                /*  18: EXTI7 */
    .word EXTI8_IRQHandler                /*  19: EXTI8 */
    .word EXTI9_IRQHandler                /*  20: EXTI9 */
    .word EXTI10_IRQHandler               /*  21: EXTI10 */
    .word EXTI11_IRQHandler               /*  22: EXTI11 */
    .word EXTI12_IRQHandler               /*  23: EXTI12 */
    .word EXTI13_IRQHandler               /*  24: EXTI13 */
    .word EXTI14_IRQHandler               /*  25: EXTI14 */
    .word EXTI15_IRQHandler               /*  26: EXTI15 */
    .word GPDMA1_Channel0_IRQHandler      /*  27: GPDMA1_Channel0 */
    .word GPDMA1_Channel1_IRQHandler      /*  28: GPDMA1_Channel1 */
    .word GPDMA1_Channel2_IRQHandler      /*  29: GPDMA1_Channel2 */
    .word GPDMA1_Channel3_IRQHandler      /*  30: GPDMA1_Channel3 */
    .word GPDMA1_Channel4_IRQHandler      /*  31: GPDMA1_Channel4 */
    .word GPDMA1_Channel5_IRQHandler      /*  32: GPDMA1_Channel5 */
    .word GPDMA1_Channel6_IRQHandler      /*  33: GPDMA1_Channel6 */
    .word GPDMA1_Channel7_IRQHandler      /*  34: GPDMA1_Channel7 */
    .word IWDG_IRQHandler                 /*  35: IWDG */
    .word SAES_IRQHandler                 /*  36: SAES (not on the H523) */
    .word ADC1_IRQHandler                 /*  37: ADC1 */
    .word DAC1_IRQHandler                 /*  38: DAC1 */
    .word FDCAN1_IT0_IRQHandler           /*  39: FDCAN1_IT0 */
    .word FDCAN1_IT1_IRQHandler           /*  40: FDCAN1_IT1 */
    .word TIM1_BRK_IRQHandler             /*  41: TIM1_BRK */
    .word TIM1_UP_IRQHandler              /*  42: TIM1_UP */
    .word TIM1_TRG_COM_IRQHandler         /*  43: TIM1_TRG_COM */
    .word TIM1_CC_IRQHandler              /*  44: TIM1_CC */
    .word TIM2_IRQHandler                 /*  45: TIM2 */
    .word TIM3_IRQHandler                 /*  46: TIM3 */
    .word TIM4_IRQHandler                 /*  47: TIM4 */
    .word TIM5_IRQHandler                 /*  48: TIM5 */
    .word TIM6_IRQHandler                 /*  49: TIM6 */
    .word TIM7_IRQHandler                 /*  50: TIM7 */
    .word I2C1_EV_IRQHandler              /*  51: I2C1_EV */
    .word I2C1_ER_IRQHandler              /*  52: I2C1_ER */
    .word I2C2_EV_IRQHandler              /*  53: I2C2_EV */
    .word I2C2_ER_IRQHandler              /*  54: I2C2_ER */
    .word SPI1_IRQHandler                 /*  55: SPI1 */
    .word SPI2_IRQHandler                 /*  56: SPI2 */
    .word SPI3_IRQHandler                 /*  57: SPI3 */
    .word USART1_IRQHandler               /*  58: USART1 */
    .word USART2_IRQHandler               /*  59: USART2 */
    .word USART3_IRQHandler               /*  60: USART3 */
    .word UART4_IRQHandler                /*  61: UART4 */
    .word UART5_IRQHandler                /*  62: UART5 */
    .word LPUART1_IRQHandler              /*  63: LPUART1 */
    .word LPTIM1_IRQHandler               /*  64: LPTIM1 */
    .word TIM8_BRK_IRQHandler             /*  65: TIM8_BRK */
    .word TIM8_UP_IRQHandler              /*  66: TIM8_UP */
    .word TIM8_TRG_COM_IRQHandler         /*  67: TIM8_TRG_COM */
    .word TIM8_CC_IRQHandler              /*  68: TIM8_CC */
    .word ADC2_IRQHandler                 /*  69: ADC2 */
    .word LPTIM2_IRQHandler               /*  70: LPTIM2 */
    .word TIM15_IRQHandler                /*  71: TIM15 */
    .word 0                             /*  72: reserved */
    .word 0                             /*  73: reserved */
    .word USB_DRD_FS_IRQHandler           /*  74: USB FS (RM: USB_FS) */
    .word CRS_IRQHandler                  /*  75: CRS */
    .word UCPD1_IRQHandler                /*  76: UCPD1 */
    .word FMC_IRQHandler                  /*  77: FMC */
    .word OCTOSPI1_IRQHandler             /*  78: OCTOSPI1 */
    .word SDMMC1_IRQHandler               /*  79: SDMMC1 */
    .word I2C3_EV_IRQHandler              /*  80: I2C3_EV */
    .word I2C3_ER_IRQHandler              /*  81: I2C3_ER */
    .word SPI4_IRQHandler                 /*  82: SPI4 */
    .word 0                             /*  83: reserved */
    .word 0                             /*  84: reserved */
    .word USART6_IRQHandler               /*  85: USART6 */
    .word 0                             /*  86: reserved */
    .word 0                             /*  87: reserved */
    .word 0                             /*  88: reserved */
    .word 0                             /*  89: reserved */
    .word GPDMA2_Channel0_IRQHandler      /*  90: GPDMA2_Channel0 */
    .word GPDMA2_Channel1_IRQHandler      /*  91: GPDMA2_Channel1 */
    .word GPDMA2_Channel2_IRQHandler      /*  92: GPDMA2_Channel2 */
    .word GPDMA2_Channel3_IRQHandler      /*  93: GPDMA2_Channel3 */
    .word GPDMA2_Channel4_IRQHandler      /*  94: GPDMA2_Channel4 */
    .word GPDMA2_Channel5_IRQHandler      /*  95: GPDMA2_Channel5 */
    .word GPDMA2_Channel6_IRQHandler      /*  96: GPDMA2_Channel6 */
    .word GPDMA2_Channel7_IRQHandler      /*  97: GPDMA2_Channel7 */
    .word 0                             /*  98: reserved */
    .word 0                             /*  99: reserved */
    .word 0                             /* 100: reserved */
    .word 0                             /* 101: reserved */
    .word 0                             /* 102: reserved */
    .word FPU_IRQHandler                  /* 103: FPU */
    .word ICACHE_IRQHandler               /* 104: ICACHE */
    .word DCACHE1_IRQHandler              /* 105: DCACHE (RM: DCACHE) */
    .word 0                             /* 106: reserved */
    .word 0                             /* 107: reserved */
    .word DCMI_PSSI_IRQHandler            /* 108: DCMI_PSSI */
    .word FDCAN2_IT0_IRQHandler           /* 109: FDCAN2_IT0 */
    .word FDCAN2_IT1_IRQHandler           /* 110: FDCAN2_IT1 */
    .word 0                             /* 111: reserved */
    .word 0                             /* 112: reserved */
    .word DTS_IRQHandler                  /* 113: DTS */
    .word RNG_IRQHandler                  /* 114: RNG */
    .word OTFDEC1_IRQHandler              /* 115: OTFDEC1 (not on the H523) */
    .word AES_IRQHandler                  /* 116: AES */
    .word HASH_IRQHandler                 /* 117: HASH */
    .word PKA_IRQHandler                  /* 118: PKA (not on the H523) */
    .word CEC_IRQHandler                  /* 119: CEC */
    .word TIM12_IRQHandler                /* 120: TIM12 */
    .word 0                             /* 121: reserved */
    .word 0                             /* 122: reserved */
    .word I3C1_EV_IRQHandler              /* 123: I3C1_EV */
    .word I3C1_ER_IRQHandler              /* 124: I3C1_ER */
    .word 0                             /* 125: reserved */
    .word 0                             /* 126: reserved */
    .word 0                             /* 127: reserved */
    .word 0                             /* 128: reserved */
    .word 0                             /* 129: reserved */
    .word 0                             /* 130: reserved */
    .word I3C2_EV_IRQHandler              /* 131: I3C2_EV */
    .word I3C2_ER_IRQHandler              /* 132: I3C2_ER */

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
    .weak SecureFault_Handler
    .thumb_set SecureFault_Handler, Default_Handler
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
    .weak PVD_AVD_IRQHandler
    .thumb_set PVD_AVD_IRQHandler, Default_Handler
    .weak RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler
    .weak RTC_S_IRQHandler
    .thumb_set RTC_S_IRQHandler, Default_Handler
    .weak TAMP_IRQHandler
    .thumb_set TAMP_IRQHandler, Default_Handler
    .weak RAMCFG_IRQHandler
    .thumb_set RAMCFG_IRQHandler, Default_Handler
    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler
    .weak FLASH_S_IRQHandler
    .thumb_set FLASH_S_IRQHandler, Default_Handler
    .weak GTZC_IRQHandler
    .thumb_set GTZC_IRQHandler, Default_Handler
    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler
    .weak RCC_S_IRQHandler
    .thumb_set RCC_S_IRQHandler, Default_Handler
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
    .weak EXTI5_IRQHandler
    .thumb_set EXTI5_IRQHandler, Default_Handler
    .weak EXTI6_IRQHandler
    .thumb_set EXTI6_IRQHandler, Default_Handler
    .weak EXTI7_IRQHandler
    .thumb_set EXTI7_IRQHandler, Default_Handler
    .weak EXTI8_IRQHandler
    .thumb_set EXTI8_IRQHandler, Default_Handler
    .weak EXTI9_IRQHandler
    .thumb_set EXTI9_IRQHandler, Default_Handler
    .weak EXTI10_IRQHandler
    .thumb_set EXTI10_IRQHandler, Default_Handler
    .weak EXTI11_IRQHandler
    .thumb_set EXTI11_IRQHandler, Default_Handler
    .weak EXTI12_IRQHandler
    .thumb_set EXTI12_IRQHandler, Default_Handler
    .weak EXTI13_IRQHandler
    .thumb_set EXTI13_IRQHandler, Default_Handler
    .weak EXTI14_IRQHandler
    .thumb_set EXTI14_IRQHandler, Default_Handler
    .weak EXTI15_IRQHandler
    .thumb_set EXTI15_IRQHandler, Default_Handler
    .weak GPDMA1_Channel0_IRQHandler
    .thumb_set GPDMA1_Channel0_IRQHandler, Default_Handler
    .weak GPDMA1_Channel1_IRQHandler
    .thumb_set GPDMA1_Channel1_IRQHandler, Default_Handler
    .weak GPDMA1_Channel2_IRQHandler
    .thumb_set GPDMA1_Channel2_IRQHandler, Default_Handler
    .weak GPDMA1_Channel3_IRQHandler
    .thumb_set GPDMA1_Channel3_IRQHandler, Default_Handler
    .weak GPDMA1_Channel4_IRQHandler
    .thumb_set GPDMA1_Channel4_IRQHandler, Default_Handler
    .weak GPDMA1_Channel5_IRQHandler
    .thumb_set GPDMA1_Channel5_IRQHandler, Default_Handler
    .weak GPDMA1_Channel6_IRQHandler
    .thumb_set GPDMA1_Channel6_IRQHandler, Default_Handler
    .weak GPDMA1_Channel7_IRQHandler
    .thumb_set GPDMA1_Channel7_IRQHandler, Default_Handler
    .weak IWDG_IRQHandler
    .thumb_set IWDG_IRQHandler, Default_Handler
    .weak SAES_IRQHandler
    .thumb_set SAES_IRQHandler, Default_Handler
    .weak ADC1_IRQHandler
    .thumb_set ADC1_IRQHandler, Default_Handler
    .weak DAC1_IRQHandler
    .thumb_set DAC1_IRQHandler, Default_Handler
    .weak FDCAN1_IT0_IRQHandler
    .thumb_set FDCAN1_IT0_IRQHandler, Default_Handler
    .weak FDCAN1_IT1_IRQHandler
    .thumb_set FDCAN1_IT1_IRQHandler, Default_Handler
    .weak TIM1_BRK_IRQHandler
    .thumb_set TIM1_BRK_IRQHandler, Default_Handler
    .weak TIM1_UP_IRQHandler
    .thumb_set TIM1_UP_IRQHandler, Default_Handler
    .weak TIM1_TRG_COM_IRQHandler
    .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler
    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler
    .weak TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler
    .weak TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler
    .weak TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler
    .weak TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler, Default_Handler
    .weak TIM6_IRQHandler
    .thumb_set TIM6_IRQHandler, Default_Handler
    .weak TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler
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
    .weak SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler
    .weak SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler
    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler
    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler
    .weak USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler
    .weak UART4_IRQHandler
    .thumb_set UART4_IRQHandler, Default_Handler
    .weak UART5_IRQHandler
    .thumb_set UART5_IRQHandler, Default_Handler
    .weak LPUART1_IRQHandler
    .thumb_set LPUART1_IRQHandler, Default_Handler
    .weak LPTIM1_IRQHandler
    .thumb_set LPTIM1_IRQHandler, Default_Handler
    .weak TIM8_BRK_IRQHandler
    .thumb_set TIM8_BRK_IRQHandler, Default_Handler
    .weak TIM8_UP_IRQHandler
    .thumb_set TIM8_UP_IRQHandler, Default_Handler
    .weak TIM8_TRG_COM_IRQHandler
    .thumb_set TIM8_TRG_COM_IRQHandler, Default_Handler
    .weak TIM8_CC_IRQHandler
    .thumb_set TIM8_CC_IRQHandler, Default_Handler
    .weak ADC2_IRQHandler
    .thumb_set ADC2_IRQHandler, Default_Handler
    .weak LPTIM2_IRQHandler
    .thumb_set LPTIM2_IRQHandler, Default_Handler
    .weak TIM15_IRQHandler
    .thumb_set TIM15_IRQHandler, Default_Handler
    .weak USB_DRD_FS_IRQHandler
    .thumb_set USB_DRD_FS_IRQHandler, Default_Handler
    .weak CRS_IRQHandler
    .thumb_set CRS_IRQHandler, Default_Handler
    .weak UCPD1_IRQHandler
    .thumb_set UCPD1_IRQHandler, Default_Handler
    .weak FMC_IRQHandler
    .thumb_set FMC_IRQHandler, Default_Handler
    .weak OCTOSPI1_IRQHandler
    .thumb_set OCTOSPI1_IRQHandler, Default_Handler
    .weak SDMMC1_IRQHandler
    .thumb_set SDMMC1_IRQHandler, Default_Handler
    .weak I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler
    .weak I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler
    .weak SPI4_IRQHandler
    .thumb_set SPI4_IRQHandler, Default_Handler
    .weak USART6_IRQHandler
    .thumb_set USART6_IRQHandler, Default_Handler
    .weak GPDMA2_Channel0_IRQHandler
    .thumb_set GPDMA2_Channel0_IRQHandler, Default_Handler
    .weak GPDMA2_Channel1_IRQHandler
    .thumb_set GPDMA2_Channel1_IRQHandler, Default_Handler
    .weak GPDMA2_Channel2_IRQHandler
    .thumb_set GPDMA2_Channel2_IRQHandler, Default_Handler
    .weak GPDMA2_Channel3_IRQHandler
    .thumb_set GPDMA2_Channel3_IRQHandler, Default_Handler
    .weak GPDMA2_Channel4_IRQHandler
    .thumb_set GPDMA2_Channel4_IRQHandler, Default_Handler
    .weak GPDMA2_Channel5_IRQHandler
    .thumb_set GPDMA2_Channel5_IRQHandler, Default_Handler
    .weak GPDMA2_Channel6_IRQHandler
    .thumb_set GPDMA2_Channel6_IRQHandler, Default_Handler
    .weak GPDMA2_Channel7_IRQHandler
    .thumb_set GPDMA2_Channel7_IRQHandler, Default_Handler
    .weak FPU_IRQHandler
    .thumb_set FPU_IRQHandler, Default_Handler
    .weak ICACHE_IRQHandler
    .thumb_set ICACHE_IRQHandler, Default_Handler
    .weak DCACHE1_IRQHandler
    .thumb_set DCACHE1_IRQHandler, Default_Handler
    .weak DCMI_PSSI_IRQHandler
    .thumb_set DCMI_PSSI_IRQHandler, Default_Handler
    .weak FDCAN2_IT0_IRQHandler
    .thumb_set FDCAN2_IT0_IRQHandler, Default_Handler
    .weak FDCAN2_IT1_IRQHandler
    .thumb_set FDCAN2_IT1_IRQHandler, Default_Handler
    .weak DTS_IRQHandler
    .thumb_set DTS_IRQHandler, Default_Handler
    .weak RNG_IRQHandler
    .thumb_set RNG_IRQHandler, Default_Handler
    .weak OTFDEC1_IRQHandler
    .thumb_set OTFDEC1_IRQHandler, Default_Handler
    .weak AES_IRQHandler
    .thumb_set AES_IRQHandler, Default_Handler
    .weak HASH_IRQHandler
    .thumb_set HASH_IRQHandler, Default_Handler
    .weak PKA_IRQHandler
    .thumb_set PKA_IRQHandler, Default_Handler
    .weak CEC_IRQHandler
    .thumb_set CEC_IRQHandler, Default_Handler
    .weak TIM12_IRQHandler
    .thumb_set TIM12_IRQHandler, Default_Handler
    .weak I3C1_EV_IRQHandler
    .thumb_set I3C1_EV_IRQHandler, Default_Handler
    .weak I3C1_ER_IRQHandler
    .thumb_set I3C1_ER_IRQHandler, Default_Handler
    .weak I3C2_EV_IRQHandler
    .thumb_set I3C2_EV_IRQHandler, Default_Handler
    .weak I3C2_ER_IRQHandler
    .thumb_set I3C2_ER_IRQHandler, Default_Handler
