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
    .word Reset_Handler /* 1 Reset */
    .word hang /* 2 NMI */
    .word hang /* 3 HardFault */
    .word hang /* 4 MemManage */
    .word hang /* 5 BusFault */
    .word hang /* 6 UsageFault */
    .word hang /* 7 RESERVED */
    .word hang /* 8 RESERVED */
    .word hang /* 9 RESERVED*/
    .word hang /* 10 RESERVED */
    .word hang /* 11 SVCall */
    .word hang /* 12 Debug Monitor */
    .word hang /* 13 RESERVED */
    .word hang /* 14 PendSV */
    .word SysTick_Handler /* 15 SysTick */
    .word hang /* 16 IRQ 0 */
    .word hang /* 17 IRQ 1 */
    .word hang /* 18 IRQ 2 */
    .word hang /* 19 ... */

.thumb_func
hang: b .
