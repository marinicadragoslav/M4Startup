.syntax unified
.cpu cortex-m4
.thumb

.global g_vectors

.word _data_start_addr
.word _data_end_addr
.word _data_phys_addr
.word _bss_start_addr
.word _bss_end_addr

/* Reset_Handler function */
  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
    ldr   r0, =_stack_top_addr
    mov   sp, r0  /* set stack pointer */
    
    /* Copy the data segment initializers from Flash to Ram */
    ldr r0, =_data_start_addr
    ldr r1, =_data_end_addr
    ldr r2, =_data_phys_addr
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

    /* Zero fill the bss segment. */
    ldr r2, =_bss_start_addr
    ldr r4, =_bss_end_addr
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str  r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss
    
    /* Jump to main */
    bl main
    b hang

.size Reset_Handler, .-Reset_Handler

/* Vector table */
.section .isr_vector,"a",%progbits
  .type g_vectors, %object
  .size g_vectors, .-g_vectors

g_vectors:
    .word _stack_top_addr
    .word Reset_Handler     /* 1 Reset */
    .word hang              /* 2 NMI */
    .word hang              /* 3 HardFault */
    .word hang              /* 4 MemManage */
    .word hang              /* 5 BusFault */
    .word hang              /* 6 UsageFault */
    .word hang              /* 7 RESERVED */
    .word hang              /* 8 RESERVED */
    .word hang              /* 9 RESERVED*/
    .word hang              /* 10 RESERVED */
    .word hang              /* 11 SVCall */
    .word hang              /* 12 Debug Monitor */
    .word hang              /* 13 RESERVED */
    .word hang              /* 14 PendSV */
    .word SysTick_Handler   /* 15 SysTick */
    .word hang              /* 16 WWDG */
    .word hang              /* 17 PVD */
    .word hang              /* 18 TAMP_STAMP */
    .word hang              /* 19 RTC_WKUP */
    .word hang              /* 20 FLASH */
    .word hang              /* 21 RCC */
    .word Exti0_Handler     /* 22 EXTI0 */
    .word Exti1_Handler     /* 23 EXTI1 */
    .word Exti2_Handler     /* 24 EXTI2 */
    .word Exti3_Handler     /* 25 EXTI3 */
    .word Exti4_Handler     /* 26 EXTI4 */
    .word hang              /* 27 DMA Stream 0 */
    .word hang              /* 28 DMA Stream 1 */
    .word hang              /* 29 DMA Stream 2 */
    .word hang              /* 30 DMA Stream 3 */
    .word hang              /* 31 DMA Stream 4 */
    .word hang              /* 32 DMA Stream 5 */
    .word hang              /* 33 DMA Stream 6 */
    .word hang              /* 34 ADC */
    .word hang              /* 35 CAN1 TX */
    .word hang              /* 36 CAN1 RX0 */
    .word hang              /* 37 CAN1 RX1 */
    .word hang              /* 38 CAN1 SCE */
    .word Exti9_5_Handler   /* 39 EXTI9_5 */
    .word hang              /* 40 TIM1_BRK_TIM9 */
    .word hang              /* 41 TIM1_UP_TIM10 */
    .word hang              /* 42 TIM1_TRG_COM_TIM11 */
    .word hang              /* 43 TIM1_CC */
    .word hang              /* 44 TIM2 */
    .word hang              /* 45 TIM3 */
    .word hang              /* 46 TIM4 */
    .word hang              /* 47 I2C1_EV */
    .word hang              /* 48 I2C1_ER */
    .word hang              /* 49 I2C2_EV */
    .word hang              /* 50 I2C2_ER */
    .word hang              /* 51 SPI1 */
    .word hang              /* 52 SPI2 */
    .word hang              /* 53 USART1 */
    .word hang              /* 54 USART2 */
    .word hang              /* 55 USART3 */
    .word Exti15_10_Handler /* 56 EXTI15_10 */

.thumb_func
hang: b .
