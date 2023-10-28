/*
 * system.c
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#include "stm32f407.h"
#include "gpio_driver.h"
#include "system.h"

/* enable HSE oscillator */
#define HSE_ENABLE()          (RCC->CR |= RCC_CR_HSEON_MASK)

/* wait until HSE osc is Ready */
#define WAIT_HSE_READY()      do {} while(!(RCC->CR & RCC_CR_HSERDY_MASK))

/* set Flash Latency */
#define SET_FLASH_LATENCY(x)  (FLASH_IF->ACR = (FLASH_IF->ACR & (~FLASH_ACR_LATENCY_MASK)) |   \
                              ((x << FLASH_ACR_LATENCY_SHIFT) & FLASH_ACR_LATENCY_MASK))

/* set PLL Clock to 168 MHz. PLL Clock output = (input * (PLLN / PLLM)) / PLLP. Input = 8 Mhz, PLLN = 336, PLLM = 8.
    PLLP is 2 when cleared. */
#define CFG_PLL_FOR_168MHZ()  (RCC->PLLCFGR = (RCC->PLLCFGR &   \
            (~(RCC_PLLCFGR_PLLSRC_MASK | RCC_PLLCFGR_PLLM_MASK | RCC_PLLCFGR_PLLN_MASK | RCC_PLLCFGR_PLLP_MASK))) | \
            (RCC_PLLCFGR_PLLSRC_MASK | (8U << RCC_PLLCFGR_PLLM_SHIFT) | (336U << RCC_PLLCFGR_PLLN_SHIFT)))

/* enable PLL */
#define PLL_ENABLE()          (RCC->CR |= RCC_CR_PLLON_MASK)

/* wait until PLL is Ready */
#define WAIT_PLL_READY()      do {} while(!(RCC->CR & RCC_CR_PLLRDY_MASK))

/* clear AHB Prescaler (field val 0 == div by 1) */
#define CLEAR_AHB_PRESCALER() (RCC->CFGR &= ~RCC_CFGR_HPRE_MSK)

/* set Sys Clk source to PLL */
#define SET_SYS_CLK_PLL()     (RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW_MASK)) | (0x2U << RCC_CFGR_SW_SHIFT))

/* wait until Sys Clk source is switched to PLL */
#define WAIT_SYS_CLK_PLL()    do {} while((RCC->CFGR & RCC_CFGR_SWS_MASK) != (0x2U << RCC_CFGR_SWS_SHIFT))

/* set APB1 prescaler - possible values for x: 3 (div_by_1), 4 (div_by_2), 5 (div_by_4), 6 (div_by_8), 7 (div_by_16) */
#define SET_APB1_PRESCALER(x) (RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1_MASK)) |   \
                                ((x << RCC_CFGR_PPRE1_SHIFT) & RCC_CFGR_PPRE1_MASK))

/* set APB2 prescaler - possible values for x: 3 (div_by_1), 4 (div_by_2), 5 (div_by_4), 6 (div_by_8), 7 (div_by_16) */
#define SET_APB2_PRESCALER(x) (RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2_MASK)) |   \
                                ((x << RCC_CFGR_PPRE2_SHIFT) & RCC_CFGR_PPRE2_MASK))


static uint32_t SysMs = 0; /* milliseconds since power on */
static void _SysTick_Cfg(uint32_t Ticks, uint8_t IrqPrio);


void SysNvic_SetPriority(int8_t IrqNum, uint8_t Prio)
{
    /* check params */
    if(IrqNum >= 0)
    {
        /* Each NVIC_IPRn register is divided into four bytes, each byte has a number of NVIC_PRIO_BITS relevant bits 
           that need to be programmed, and they are the most significant ones (MSB)! */
        uint32_t Reg = IrqNum / 4;                       /* IPRn register number */
        uint32_t Byte = IrqNum % 4;                      /* Byte number inside register */
        uint32_t Offset = (8U - NVIC_PRIO_BITS);         /* Offset of priority bits relative to byte's LSB bit*/
        uint32_t BMsk = (0xFFU << (8 * Byte));           /* Byte Mask */
        uint32_t PMsk = (Prio << ((8 * Byte) + Offset)); /* Priority Mask */

        volatile uint32_t* IPR_RegPtr = (NVIC_PRI_BASE_ADDR + Reg);
        (*IPR_RegPtr) = ((*IPR_RegPtr) & (~BMsk)) | PMsk;
    }
    else
    {
        /* System Irq Numbers range: -1 (Systick, SHP[11] register) to -14 (MemManage Fault, SHP[0]). */
        /* SHP[n] is one byte access! */
        SCB->SHP[((uint8_t)IrqNum & 0xFU) - 4U] = (uint8_t)(Prio << (8U - NVIC_PRIO_BITS));
    }
}

/* Systick interrupt handler - should be fired every millisecond */
void SysTick_Handler(void)
{
    SysMs++;
}

/* Get number of milliseconds since power on */
uint32_t SysClock_GetSystemMs(void)
{
    return SysMs;
}

/* Set up system clock for 168 Mhz (max) */
void SysClock_Set168Mhz(void)
{
    HSE_ENABLE();
    WAIT_HSE_READY();

    SET_FLASH_LATENCY(5); /* Needs to be 5 for 168Mhz */

    CFG_PLL_FOR_168MHZ();
    PLL_ENABLE();
    WAIT_PLL_READY();

    CLEAR_AHB_PRESCALER();
    SET_SYS_CLK_PLL();
    WAIT_SYS_CLK_PLL();

    SET_APB1_PRESCALER(5); /* Div by 4 */
    SET_APB2_PRESCALER(4); /* Div by 2 */

    /* Set systick to 1ms and priority 10 */
    _SysTick_Cfg(168000000 / 1000, NVIC_IRQ_PRIO10);
}

/* Set number of ticks between two consecutive SysTick interrupts and the interrupt priority */
static void _SysTick_Cfg(uint32_t Ticks, uint8_t IrqPrio)
{
    if ((Ticks - 1U) < (SYSTICK_RELOAD_MASK >> SYSTICK_RELOAD_SHIFT))
    {
        SYSTICK->LOAD = (Ticks - 1U);
        SYSTICK->VAL = 0U;
        SysNvic_SetPriority(IRQ_NUM_SYSTICK, IrqPrio);
        SYSTICK->CTRL = SYSTICKCTRL_CLKSOURCE_MASK | SYSTICK_CTRL_TICKINT_MASK | SYSTICK_CTRL_ENABLE_MASK;
    }
}

/* Interrupt handler for EXTI0 */
__attribute__((weak)) void Exti0_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI1 */
__attribute__((weak)) void Exti1_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI2 */
__attribute__((weak)) void Exti2_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI3 */
__attribute__((weak)) void Exti3_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI4 */
__attribute__((weak)) void Exti4_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI9_5 */
__attribute__((weak)) void Exti9_5_Handler(void)
{
    /* User can define his own version for this function */
}

/* Interrupt handler for EXTI15_10 */
__attribute__((weak)) void Exti15_10_Handler(void)
{
    /* User can define his own version for this function */
}





