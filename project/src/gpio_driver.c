/*
 * gpio_driver.c
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

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
 * @brief   Enables the clock for the given GPIO port. This must be called before GPIO_PinApplyConfig.
 */
GPIO_Status_t GPIO_PortClockEnable(GPIO_Port_t Port)
{
   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1ENR |= (1U << Port);

   return GPIO_STATUS_OK;
}

/**
 * @brief   Disables the clock for the given GPIO port
 */
GPIO_Status_t GPIO_PortClockDisable(GPIO_Port_t Port)
{
   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1ENR &= ~(1U << Port);

   return GPIO_STATUS_OK;
}

/**
 * @brief   Resets the corresponding GPIO registers of the given port to default
 */
GPIO_Status_t GPIO_PortReset(GPIO_Port_t Port)
{
   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   RCC->AHB1RSTR |=  (1U << Port);
   RCC->AHB1RSTR &= ~(1U << Port);

   return GPIO_STATUS_OK;
}

/**
 * @brief   Reads the value of the given GPIO Port (16 bits, 1 bit for each pin)
 */
GPIO_Status_t GPIO_PortRead(GPIO_Port_t Port, uint16_t* Val)
{
   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   *Val = (uint16_t)(PortInstances[Port]->IDR); /* The lower 16 bits are relevant from the register */

   return GPIO_STATUS_OK;
}

/**
 * @brief   Writes the given value (16 bits, 1 bit for each pin) to the given GPIO Port
 */
GPIO_Status_t GPIO_PortWrite(GPIO_Port_t Port, uint16_t Val)
{
   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   PortInstances[Port]->ODR = (uint32_t)Val;

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

   if ((uint32_t)Port >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)PinNum >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   Pin->Port = Port;
   Pin->PinNum = PinNum;

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

   if ((uint32_t)(Pin->Port) >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)(Pin->PinNum) >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   /* Set generic default characteristics */
   Pin->Mode = GPIO_MODE_IN;
   Pin->Speed = GPIO_SPEED_LOW;
   Pin->Pull = GPIO_PULL_NONE;
   Pin->OutType = GPIO_OUTTYPE_PUSHPULL;
   Pin->AltFunc = GPIO_ALTFUNC_0;

   /* Set specific default characteristics, depending on the port and pin number */
   if(Pin->Port == GPIO_PORT_A)
   {
      /* default value for GPIOA MODER register is 0xA8000000 => pins 13, 14, 15 are in Alternate Function mode */
      if(Pin->PinNum == GPIO_PIN_NUM_13 || Pin->PinNum == GPIO_PIN_NUM_14 || Pin->PinNum == GPIO_PIN_NUM_15)
      {
         Pin->Mode = GPIO_MODE_ALTFUNC;
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
         Pin->Mode = GPIO_MODE_ALTFUNC;
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

   return GPIO_STATUS_OK;
}


/**
 * @brief   Configures a Pin with the given characteristics. The ones with a "NOT_APPLICABLE" value will be skipped.
 */
GPIO_Status_t GPIO_PinConfig(GPIO_Pin_t* Pin, GPIO_Mode_t Mode, GPIO_Speed_t Speed, GPIO_Pull_t Pull, 
      GPIO_OutType_t OutType, GPIO_AltFunc_t AltFunc)
{
   if (!Pin)
   {
      return GPIO_STATUS_INVALID_PIN;
   }

   if ((uint32_t)Mode >= (uint32_t)GPIO_MODE_INVALID)
   {
      return GPIO_STATUS_INVALID_MODE;
   }

   if ((uint32_t)Speed >= (uint32_t)GPIO_SPEED_INVALID)
   {
      return GPIO_STATUS_INVALID_SPEED;
   }

   if ((uint32_t)Pull >= (uint32_t)GPIO_PULL_INVALID)
   {
      return GPIO_STATUS_INVALID_PULL;
   }

   if ((uint32_t)OutType >= (uint32_t)GPIO_OUTTYPE_INVALID)
   {
      return GPIO_STATUS_INVALID_OUTPUT;
   }

   if ((uint32_t)AltFunc >= (uint32_t)GPIO_ALTFUNC_INVALID)
   {
      return GPIO_STATUS_INVALID_ALTFUNC;
   }

   Pin->Mode = Mode;
   Pin->Speed = Speed;
   Pin->Pull = Pull;
   Pin->OutType = OutType;
   Pin->AltFunc = AltFunc;

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

   if ((uint32_t)(Pin->Port) >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)(Pin->PinNum) >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if ((uint32_t)(Pin->Mode) >= (uint32_t)GPIO_MODE_INVALID)
   {
      return GPIO_STATUS_INVALID_MODE;
   }

   if ((uint32_t)(Pin->Speed) >= (uint32_t)GPIO_SPEED_INVALID)
   {
      return GPIO_STATUS_INVALID_SPEED;
   }

   if ((uint32_t)(Pin->Pull) >= (uint32_t)GPIO_PULL_INVALID)
   {
      return GPIO_STATUS_INVALID_PULL;
   }

   if ((uint32_t)(Pin->OutType) >= (uint32_t)GPIO_OUTTYPE_INVALID)
   {
      return GPIO_STATUS_INVALID_OUTPUT;
   }

   if ((uint32_t)(Pin->AltFunc) >= (uint32_t)GPIO_ALTFUNC_INVALID)
   {
      return GPIO_STATUS_INVALID_ALTFUNC;
   }

   GPIO_PortRegDef_t* PortInst = PortInstances[Pin->Port];

   if (Pin->Mode == GPIO_MODE_IN || Pin->Mode == GPIO_MODE_OUT || 
         Pin->Mode == GPIO_MODE_ALTFUNC || Pin->Mode == GPIO_MODE_ANALOG)
   {
      /* non-interrupt pin mode */
      PortInst->MODER = (PortInst->MODER & (~(0x03U << (2 * Pin->PinNum)))) | (Pin->Mode << (2 * Pin->PinNum));
   }
   else if (Pin->Mode != GPIO_MODE_SKIP)
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
      uint8_t Shift;
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
      if (Pin->Speed != GPIO_SPEED_SKIP)
      {
         PortInst->OSPEEDR = (PortInst->OSPEEDR & (~(0x03U << (2 * Pin->PinNum)))) | (Pin->Speed << (2 * Pin->PinNum));
      }

      if (Pin->OutType != GPIO_OUTTYPE_SKIP)
      {
         PortInst->OTYPER = (PortInst->OTYPER & (~(0x01U << Pin->PinNum))) | (Pin->OutType << Pin->PinNum);
      }
   }
   
   /* configure pull resistor */
   if (Pin->Pull != GPIO_PULL_SKIP)
   {
      PortInst->PUPDR = (PortInst->PUPDR & (~(0x03U << (2 * Pin->PinNum)))) | (Pin->Pull << (2 * Pin->PinNum));
   }

   /* configure alternate function */
   if((Pin->Mode == GPIO_MODE_ALTFUNC) && (Pin->AltFunc != GPIO_ALTFUNC_SKIP))
   {
      uint8_t Reg = Pin->PinNum / 8; /* 0 or 1, meaning index of AFR (AFR[0] = AFRL reg, AFR[1] = AFRH reg) */
      uint8_t Pos = Pin->PinNum % 8; /* 0..7, means the position of the 4-bit field that has to be modified */
      PortInst->AFR[Reg] = (PortInst->AFR[Reg] & (~(0x0FU << (4 * Pos)))) | (Pin->AltFunc << (4 * Pos));
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

   if ((uint32_t)(Pin->Port) >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)(Pin->PinNum) >= (uint32_t)GPIO_PIN_NUM_INVALID)
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

   if ((uint32_t)(Pin->Port) >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)(Pin->PinNum) >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if ((uint32_t)State >= (uint32_t)GPIO_PIN_STATE_INVALID)
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

   if ((uint32_t)(Pin->Port) >= (uint32_t)GPIO_PORT_INVALID)
   {
      return GPIO_STATUS_INVALID_PORT;
   }

   if ((uint32_t)(Pin->PinNum) >= (uint32_t)GPIO_PIN_NUM_INVALID)
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
   if ((uint32_t)PinNum >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if(Priority >= NVIC_IRQ_INVALID_PRIO)
   {
      return GPIO_STATUS_INVALID_PRIO;
   }

   uint8_t IrqNum = IrqNumbers[PinNum];
   
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
   if ((uint32_t)PinNum >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   uint8_t IrqNum = IrqNumbers[PinNum];
   
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
   if ((uint32_t)PinNum >= (uint32_t)GPIO_PIN_NUM_INVALID)
   {
      return GPIO_STATUS_INVALID_PIN_NUM;
   }

   if(EXTI->PR & (1U << PinNum)) /* if interrupt is pending */
   {
      EXTI->PR |= (1U << PinNum); /* clear pending bit by writing 1 to it */
   }

   return GPIO_STATUS_OK;
}
