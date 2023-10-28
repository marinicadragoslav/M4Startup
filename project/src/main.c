/*
 * main.c
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#include "gpio_driver.h"
#include "system.h"

GPIO_Pin_t GreenLedPin, RedLedPin, ButtonPin;

/* Interrupt handler for EXTI0 - overrides the one from system.c. Red Led is toggled on Button Pin rising edge.
   Without HW or SW debouncing, a rising edge might be detected multiple times per button push => heretic behaviour. */
void Exti0_Handler(void)
{
    (void)GPIO_PinToggle(&RedLedPin);
    (void)GPIO_ClearPendingInterrupt(ButtonPin.PinNum);
}

int main(void)
{
    SysClock_Set168Mhz();

    GPIO_PortClockEnable(GPIO_PORT_D);
    GPIO_PortClockEnable(GPIO_PORT_A);

    /* config Green Led Pin, PD12 */
    GPIO_PinAssign(&GreenLedPin, GPIO_PORT_D, GPIO_PIN_NUM_12);
    GPIO_PinConfig(&GreenLedPin, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_PULL_NONE, 
                                GPIO_OUT_PP, GPIO_AF_NOT_APPLICABLE);
    GPIO_PinApplyConfig(&GreenLedPin);

    /* config Red Led Pin, PD14 */
    GPIO_PinAssign(&RedLedPin, GPIO_PORT_D, GPIO_PIN_NUM_14);
    GPIO_PinConfig(&RedLedPin, GPIO_MODE_OUT, GPIO_SPEED_LOW, GPIO_PULL_NONE, 
                                GPIO_OUT_PP, GPIO_AF_NOT_APPLICABLE);
    GPIO_PinApplyConfig(&RedLedPin);

    /* config Button Pin, PA0 */
    GPIO_PinAssign(&ButtonPin, GPIO_PORT_A, GPIO_PIN_NUM_0);
    GPIO_PinConfig(&ButtonPin, GPIO_MODE_IT_RT, GPIO_SPEED_NOT_APPLICABLE, GPIO_PULL_NONE, 
                                GPIO_OUT_NOT_APPLICABLE, GPIO_AF_NOT_APPLICABLE);
    GPIO_PinApplyConfig(&ButtonPin);
    GPIO_EnableInterrupt(ButtonPin.PinNum, NVIC_IRQ_PRIO11);

    /* toggle Green Led @ 1 second */
    uint32_t StartMs = SysClock_GetSystemMs();
    while(1)
    {
        uint32_t EndMs = SysClock_GetSystemMs();
        if (EndMs - StartMs >= 1000)
        {
            GPIO_PinToggle(&GreenLedPin);
            StartMs = EndMs;
        }
    }

    return 0;
}


