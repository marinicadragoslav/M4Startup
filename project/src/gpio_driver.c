/*
 * gpio_driver.c
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#include <stdbool.h>
#include "gpio_driver.h"
#include "system.h"

/* Port enum to Port instance mapping */
static GPIO_PortRegDef_t* const PortInstances[] = {
      [GPIO_PORT_A] = GPIOA,
      [GPIO_PORT_B] = GPIOB,
      [GPIO_PORT_C] = GPIOC,
      [GPIO_PORT_D] = GPIOD,
      [GPIO_PORT_E] = GPIOE,
      [GPIO_PORT_F] = GPIOF,
      [GPIO_PORT_G] = GPIOG,
      [GPIO_PORT_H] = GPIOH,
      [GPIO_PORT_I] = GPIOI
};

/* Pin number to Irq number mapping */
static const uint8_t IrqNumbers[] = {
      [GPIO_PIN_NUM_0] = IRQ_NUM_EXTI_0,
      [GPIO_PIN_NUM_1] = IRQ_NUM_EXTI_1,
      [GPIO_PIN_NUM_2] = IRQ_NUM_EXTI_2,
      [GPIO_PIN_NUM_3] = IRQ_NUM_EXTI_3,
      [GPIO_PIN_NUM_4] = IRQ_NUM_EXTI_4,
      [GPIO_PIN_NUM_5] = IRQ_NUM_EXTI_5_9,
      [GPIO_PIN_NUM_6] = IRQ_NUM_EXTI_5_9,
      [GPIO_PIN_NUM_7] = IRQ_NUM_EXTI_5_9,
      [GPIO_PIN_NUM_8] = IRQ_NUM_EXTI_5_9,
      [GPIO_PIN_NUM_9] = IRQ_NUM_EXTI_5_9,
      [GPIO_PIN_NUM_10] = IRQ_NUM_EXTI_10_15,
      [GPIO_PIN_NUM_11] = IRQ_NUM_EXTI_10_15,
      [GPIO_PIN_NUM_12] = IRQ_NUM_EXTI_10_15,
      [GPIO_PIN_NUM_13] = IRQ_NUM_EXTI_10_15,
      [GPIO_PIN_NUM_14] = IRQ_NUM_EXTI_10_15,
      [GPIO_PIN_NUM_15] = IRQ_NUM_EXTI_10_15
};

/**
 * @brief   Enables the clock for the given GPIO port. This must be called before GPIO_PinInit.
 */
GPIO_Status_t GPIO_PortClockEnable(GPIO_Port_t Port)
{
   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1ENR |= (1U << ((uint8_t)Port));

   return GPIO_STATUS_OK;
}

/**
 * @brief   Disables the clock for the given GPIO port
 */
GPIO_Status_t GPIO_PortClockDisable(GPIO_Port_t Port)
{
   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1ENR &= ~(1U << ((uint8_t)Port));

   return GPIO_STATUS_OK;
}

/**
 * @brief   Resets the corresponding GPIO registers of the given port to default
 */
GPIO_Status_t GPIO_PortReset(GPIO_Port_t Port)
{
   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1RSTR |=  (1U << ((uint8_t)Port));
   RCC->AHB1RSTR &= ~(1U << ((uint8_t)Port));

   return GPIO_STATUS_OK;
}

/**
 * @brief   Reads the value of the given GPIO Port (16 bits, 1 bit for each pin)
 */
GPIO_Status_t GPIO_PortRead(GPIO_Port_t Port, uint16_t* Val)
{
   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   *Val = (uint16_t)(PortInstances[(uint8_t)Port]->IDR); /* The lower 16 bits are relevant from the register */

   return GPIO_STATUS_OK;
}

/**
 * @brief   Writes the given value (16 bits, 1 bit for each pin) to the given GPIO Port
 */
GPIO_Status_t GPIO_PortWrite(GPIO_Port_t Port, uint16_t Val)
{
   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   PortInstances[(uint8_t)Port]->ODR = (uint32_t)Val;

   return GPIO_STATUS_OK;
}


/**
 * @brief   Assigns a Port and PinNum to a Pin.
 */
GPIO_Status_t GPIO_PinAssign(GPIO_Pin_t* Pin, GPIO_Port_t Port, GPIO_PinNum_t PinNum)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)Port >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint8_t)PinNum >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   Pin->Port = Port;
   Pin->PinNum = PinNum;
   Pin->ConfigFlags |= (GPIO_FLAG_PORT | GPIO_FLAG_PIN_NUM);

   return GPIO_STATUS_OK;
}


/**
 * @brief   Sets the characteristics of a Pin to their default configuration, as they would be after a reset.
 *          This default config depends on the Port and PinNum (not all pins have the same default config).
 */
GPIO_Status_t GPIO_PinConfigDefault(GPIO_Pin_t* Pin)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)(Pin->Port) >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint8_t)(Pin->PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   /* Set generic default characteristics */
   Pin->Mode = GPIO_MODE_IN;
   Pin->Speed = GPIO_SPEED_LOW;
   Pin->Pull = GPIO_PULL_NONE;
   Pin->Output = GPIO_OUT_PP;
   Pin->AltFunc = GPIO_AF_0;

   /* Set specific default characteristics, depending on the port and pin number */
   if(Pin->Port == GPIO_PORT_A)
   {
      /* default value for GPIOA MODER register is 0xA8000000 => pins 13, 14, 15 are in Alternate Function mode */
      if(Pin->PinNum == GPIO_PIN_NUM_13 || Pin->PinNum == GPIO_PIN_NUM_14 || Pin->PinNum == GPIO_PIN_NUM_15)
      {
         Pin->Mode = GPIO_MODE_ALTFN;
      }

      /* default value for GPIOA OSPEEDR register is 0x0C000000 => pin 13 has VHIGH speed by default */
      if(Pin->PinNum == GPIO_PIN_NUM_13)
      {
         Pin->Speed = GPIO_SPEED_VHIGH;
      }

      /* default value for GPIOA PUPDR register is 0x64000000 => pin 13 and 15 have PullUp, pin 14 has PullDown */
      if(Pin->PinNum == GPIO_PIN_NUM_13 || Pin->PinNum == GPIO_PIN_NUM_15)
      {
         Pin->Pull = GPIO_PULL_UP;
      }
      else if(Pin->PinNum == GPIO_PIN_NUM_14)
      {
         Pin->Pull = GPIO_PULL_DOWN;
      }
   }
   else if(Pin->Port == GPIO_PORT_B)
   {
      /* default value for GPIOB MODER register is 0x00000280 => pins 3 and 4 are in Alternate Function mode */
      if(Pin->PinNum == GPIO_PIN_NUM_3 || Pin->PinNum == GPIO_PIN_NUM_4)
      {
         Pin->Mode = GPIO_MODE_ALTFN;
      }

      /* default value for GPIOB OSPEEDR register is 0x000000C0 => pin 3 has VHIGH speed by default */
      if(Pin->PinNum == GPIO_PIN_NUM_3)
      {
         Pin->Speed = GPIO_SPEED_VHIGH;
      }

      /* default value for GPIOB PUPDR register is 0x00000100 => pin 4 has PullUp */
      if(Pin->PinNum  == GPIO_PIN_NUM_4)
      {
         Pin->Pull = GPIO_PULL_UP;
      }
   }

   Pin->ConfigFlags |= (GPIO_FLAG_MODE | GPIO_FLAG_SPEED | GPIO_FLAG_PULL | GPIO_FLAG_OUTPUT | GPIO_FLAG_ALTFUNC);

   return GPIO_STATUS_OK;
}


/**
 * @brief   Configures a Pin with the given characteristics. The ones with a "NOT_APPLICABLE" value will be skipped.
 */
GPIO_Status_t GPIO_PinConfig(GPIO_Pin_t* Pin, GPIO_Mode_t Mode, GPIO_Speed_t Speed, GPIO_Pull_t Pull, 
      GPIO_Output_t Output, GPIO_AltFunc_t AltFunc)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   /* Set mode or die */
   if ((uint8_t)Mode >= (uint8_t)GPIO_MODE_INVALID)
   {
      return GPIO_STATUS_INVALID_MODE;
   }
   else if (Mode != GPIO_MODE_NOT_APPLICABLE)
   {
      Pin->Mode = Mode;
      Pin->ConfigFlags |= GPIO_FLAG_MODE;
   }

   /* Set speed or die */
   if ((uint8_t)Speed >= (uint8_t)GPIO_SPEED_INVALID)
   {
      return GPIO_STATUS_INVALID_SPEED;
   }
   else if (Speed != GPIO_SPEED_NOT_APPLICABLE)
   {
      Pin->Speed = Speed;
      Pin->ConfigFlags |= GPIO_FLAG_SPEED;
   }

   /* Set Pull or die */
   if ((uint8_t)Pull >= (uint8_t)GPIO_PULL_INVALID)
   {
      return GPIO_STATUS_INVALID_PULL;
   }
   else if(Pull != GPIO_PULL_NOT_APPLICABLE)
   {
      Pin->Pull = Pull;
      Pin->ConfigFlags |= GPIO_FLAG_PULL;
   }

   /* Set output or die */
   if ((uint8_t)Output >= (uint8_t)GPIO_OUT_INVALID)
   {
      return GPIO_STATUS_INVALID_OUTPUT;
   }
   else if (Output != GPIO_OUT_NOT_APPLICABLE)
   {
      Pin->Output = Output;
      Pin->ConfigFlags |= GPIO_FLAG_OUTPUT;
   }

   /* Set alternate function or die */
   if ((uint8_t)AltFunc >= (uint8_t)GPIO_AF_INVALID)
   {
      return GPIO_STATUS_INVALID_ALTFUNC;
   }
   else if (AltFunc != GPIO_AF_NOT_APPLICABLE)
   {
      Pin->AltFunc = AltFunc;
      Pin->ConfigFlags |= GPIO_FLAG_ALTFUNC;
   }

   return GPIO_STATUS_OK;
}


/**
 * @brief   Applies the Pin configuration by writing to the corresponding registers.
 */
GPIO_Status_t GPIO_PinApplyConfig(GPIO_Pin_t* Pin)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)(Pin->Port) >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((Pin->ConfigFlags & GPIO_FLAG_PORT) == 0)
   {
      return GPIO_STATUS_UNASSIGNED_PORT;
   }

   if ((uint8_t)(Pin->PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if ((Pin->ConfigFlags & GPIO_FLAG_PIN_NUM) == 0)
   {
      return GPIO_STATUS_UNASSIGNED_PIN_NUM;
   }

   if ((uint8_t)(Pin->Mode) >= (uint8_t)GPIO_MODE_NOT_APPLICABLE)
   {
      return GPIO_STATUS_INVALID_MODE;
   }

   if ((uint8_t)(Pin->Speed) >= (uint8_t)GPIO_SPEED_NOT_APPLICABLE)
   {
      return GPIO_STATUS_INVALID_SPEED;
   }

   if ((uint8_t)(Pin->Pull) >= (uint8_t)GPIO_PULL_NOT_APPLICABLE)
   {
      return GPIO_STATUS_INVALID_PULL;
   }

   if ((uint8_t)(Pin->Output) >= (uint8_t)GPIO_OUT_NOT_APPLICABLE)
   {
      return GPIO_STATUS_INVALID_OUTPUT;
   }

   if ((uint8_t)(Pin->AltFunc) >= (uint8_t)GPIO_AF_NOT_APPLICABLE)
   {
      return GPIO_STATUS_INVALID_ALTFUNC;
   }

   if ((Pin->ConfigFlags & GPIO_FLAG_ALL) != GPIO_FLAG_ALL)
   {
      GPIO_Pin_t DefaultPin;
      DefaultPin.Port = Pin->Port;
      DefaultPin.PinNum = Pin->PinNum;
      GPIO_PinConfigDefault(&DefaultPin);
      if ((Pin->ConfigFlags & GPIO_FLAG_MODE) == 0)
      {
         Pin->Mode = DefaultPin.Mode;
      }

      if ((Pin->ConfigFlags & GPIO_FLAG_SPEED) == 0)
      {
         Pin->Speed = DefaultPin.Speed;
      }

      if ((Pin->ConfigFlags & GPIO_FLAG_PULL) == 0)
      {
         Pin->Pull = DefaultPin.Pull;
      }

      if ((Pin->ConfigFlags & GPIO_FLAG_OUTPUT) == 0)
      {
         Pin->Output = DefaultPin.Output;
      }

      if ((Pin->ConfigFlags & GPIO_FLAG_ALTFUNC) == 0)
      {
         Pin->AltFunc = DefaultPin.AltFunc;
      }
   }

   GPIO_PortRegDef_t* PortInst = PortInstances[Pin->Port];
   uint8_t Shift;

   if (Pin->Mode == GPIO_MODE_IN || Pin->Mode == GPIO_MODE_OUT || 
         Pin->Mode == GPIO_MODE_ALTFN || Pin->Mode == GPIO_MODE_ANALOG)
   {
      /* non-interrupt pin mode */
      Shift = (2 * Pin->PinNum);
      PortInst->MODER = (PortInst->MODER & (~(0x03U << Shift))) | (Pin->Mode << Shift);
   }
   else
   {
      /* interrupt pin mode */
      if (Pin->Mode == GPIO_MODE_IT_FT)
      {
         /* falling-edge triggered */
         EXTI->FTSR |= (1U << Pin->PinNum);
         EXTI->RTSR &= ~(1U << Pin->PinNum);
      } 
      else if (Pin->Mode == GPIO_MODE_IT_RT)
      {
         /* rising-edge triggered */
         EXTI->RTSR |= (1U << Pin->PinNum);
         EXTI->FTSR &= ~(1U << Pin->PinNum);
      }
      else if (Pin->Mode == GPIO_MODE_IT_RFT)
      {
         /* rising and falling edge triggered */
         EXTI->FTSR |= (1U << Pin->PinNum);
         EXTI->RTSR |= (1U << Pin->PinNum);
      }

      /* enable clock to SYSCFG */
      SYSCFG_PCLK_EN();

      /* configure GPIO port selection in SYSCFG_EXTICRx */
      if(Pin->PinNum <= 3U)
      {
         Shift = (4 * Pin->PinNum);
         SYSCFG->EXTICR1 = (SYSCFG->EXTICR1 & (~(0xFU << Shift))) | (Pin->Port << Shift);
      }
      else if(Pin->PinNum >= 4U && Pin->PinNum <= 7U)
      {
         Shift = (4 * (Pin->PinNum - 4U));
         SYSCFG->EXTICR2 = (SYSCFG->EXTICR2 & (~(0xFU << Shift))) | (Pin->Port << Shift);
      }
      else if(Pin->PinNum >= 8U && Pin->PinNum <= 11U)
      {
         Shift = (4 * (Pin->PinNum - 8U));
         SYSCFG->EXTICR3 = (SYSCFG->EXTICR3 & (~(0xFU << Shift))) | (Pin->Port << Shift);
      }
      else /* Note: PinNum is <= 15U because it was already compared to GPIO_PIN_NUM_INVALID */
      {
         Shift = (4 * (Pin->PinNum - 12U));
         SYSCFG->EXTICR4 = (SYSCFG->EXTICR4 & (~(0xFU << Shift))) | (Pin->Port << Shift);
      }

      /* enable EXTI interrupt delivery using IMR */
      EXTI->IMR |= (1U << Pin->PinNum);
   }

   /* configure pin output speed and output type if pin is output */
   if(Pin->Mode == GPIO_MODE_OUT)
   {
      Shift = (2 * Pin->PinNum);
      PortInst->OSPEEDR = (PortInst->OSPEEDR & (~(0x03U << Shift))) | (Pin->Speed << Shift);
      PortInst->OTYPER = (PortInst->OTYPER & (~(0x01U << Pin->PinNum))) | (Pin->Output << Pin->PinNum);
   }

   /* configure pull resistor */
   Shift = (2 * Pin->PinNum);
   PortInst->PUPDR = (PortInst->PUPDR & (~(0x03U << Shift))) | (Pin->Pull << Shift);

   /* configure alternate function */
   if(Pin->Mode == GPIO_MODE_ALTFN)
   {
      uint8_t Reg = Pin->PinNum / 8; /* 0 or 1, meaning index of AFR (AFR[0] = AFRL reg, AFR[1] = AFRH reg) */
      uint8_t Pos = Pin->PinNum % 8; /* 0..7, means the position of the 4-bit field that has to be modified */
      Shift = (4 * Pos);
      PortInst->AFR[Reg] = (PortInst->AFR[Reg] & (~(0x0FU << (4 * Pos)))) | (Pin->AltFunc << Shift);
   }

   return GPIO_STATUS_OK;
}

/**
 * @brief   Reads the state of the given Pin
 */
GPIO_Status_t GPIO_PinRead(GPIO_Pin_t* Pin, GPIO_PinState_t* State)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)(Pin->Port) >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint8_t)(Pin->PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   GPIO_PortRegDef_t* PortInst = PortInstances[Pin->Port];

   *State = ((PortInst->IDR & (1U << Pin->PinNum)) == 0 ? GPIO_PIN_STATE_RESET : GPIO_PIN_STATE_SET);

   return GPIO_STATUS_OK;
}


/**
 * @brief   Writes the state to the given Pin
 */
GPIO_Status_t GPIO_PinWrite(GPIO_Pin_t* Pin, GPIO_PinState_t State)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)(Pin->Port) >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint8_t)(Pin->PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if ((uint8_t)State >= (uint8_t)GPIO_PIN_STATE_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_STATE;
   }

   GPIO_PortRegDef_t* PortInst = PortInstances[Pin->Port];

   if(State == GPIO_PIN_STATE_SET)
   {
      PortInst->ODR |= (1U << Pin->PinNum);
   }
   else
   {
      PortInst->ODR &= (~(1U << Pin->PinNum));
   }

   return GPIO_STATUS_OK;
}


/**
 * @brief   Toggles the state to the given Pin
 */
GPIO_Status_t GPIO_PinToggle(GPIO_Pin_t* Pin)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint8_t)(Pin->Port) >= (uint8_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint8_t)(Pin->PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   GPIO_PortRegDef_t* PortInst = PortInstances[Pin->Port];

   PortInst->ODR ^= (1U << Pin->PinNum);

   return GPIO_STATUS_OK;
}


/**
 * @brief   Enable interrupt for the Irq number associated with the given pin number.
 * @note    
 */
GPIO_Status_t GPIO_EnableInterrupt(GPIO_PinNum_t PinNum, uint8_t Priority)
{
   if ((uint8_t)(PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if(Priority >= NVIC_IRQ_INVALID_PRIO)
   {
      return GPIO_STATUS_INVALID_PRIO;
   }

   uint8_t IrqNum = IrqNumbers[(uint8_t)PinNum];
   
   if(IrqNum < 32U)
   {
      *NVIC_ISER0 |= (1U << IrqNum);
   }
   else if(IrqNum < 64U)
   {
      *NVIC_ISER1 |= (1U << (IrqNum - 32U));
   }/* NVIC_ISER2...7 not used, the highest Irq num for GPIO is 40 */

   SysNvic_SetPriority((int8_t)IrqNum, Priority);

   return GPIO_STATUS_OK;
}


/**
 * @brief   Disable interrupt for the Irq number associated with the given pin number.
 * @note    
 */
GPIO_Status_t GPIO_DisableInterrupt(GPIO_PinNum_t PinNum)
{
   if ((uint8_t)(PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   uint8_t IrqNum = IrqNumbers[(uint8_t)PinNum];
   
   if(IrqNum < 32U)
   {
      *NVIC_ICER0 |= (1U << IrqNum);
   }
   else if(IrqNum < 64U)
   {
      *NVIC_ICER1 |= (1U << (IrqNum - 32U));
   }/* NVIC_ICER2...7 not used, the highest Irq num for GPIO is 40 */

   return GPIO_STATUS_OK;
}

/**
 * @brief   Clears the pending bit of the interrupt line associated with the pin number.
 * @note    Interrupt lines for pins are mapped 1:1 with the pin number (0..15).
 * @note    Must be called by the interrupt service routine after the interrupt was triggered.
 */
GPIO_Status_t GPIO_ClearPendingInterrupt(GPIO_PinNum_t PinNum)
{
   if ((uint8_t)(PinNum) >= (uint8_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if(EXTI->PR & (1U << ((uint8_t)PinNum))) /* if interrupt is pending */
   {
      EXTI->PR |= (1U << ((uint8_t)PinNum)); /* clear pending bit by writing 1 to it */
   }

   return GPIO_STATUS_OK;
}
