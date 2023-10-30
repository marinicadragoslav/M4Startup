/*
 * gpio_driver.h
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f407.h"

typedef enum
{
   GPIO_PORT_A,
   GPIO_PORT_B,
   GPIO_PORT_C,
   GPIO_PORT_D,
   GPIO_PORT_E,
   GPIO_PORT_F,
   GPIO_PORT_G,
   GPIO_PORT_H,
   GPIO_PORT_I,
   GPIO_PORT_INVALID
}GPIO_Port_t;

typedef enum
{
   GPIO_PIN_NUM_0,
   GPIO_PIN_NUM_1,
   GPIO_PIN_NUM_2,
   GPIO_PIN_NUM_3,
   GPIO_PIN_NUM_4,
   GPIO_PIN_NUM_5,
   GPIO_PIN_NUM_6,
   GPIO_PIN_NUM_7,
   GPIO_PIN_NUM_8,
   GPIO_PIN_NUM_9,
   GPIO_PIN_NUM_10,
   GPIO_PIN_NUM_11,
   GPIO_PIN_NUM_12,
   GPIO_PIN_NUM_13,
   GPIO_PIN_NUM_14,
   GPIO_PIN_NUM_15,
   GPIO_PIN_NUM_INVALID
}GPIO_PinNum_t;

typedef enum
{
   GPIO_MODE_IN,      /* input */
   GPIO_MODE_OUT,     /* output */
   GPIO_MODE_ALTFUNC, /* alternate function */
   GPIO_MODE_ANALOG,  /* analog in */
   GPIO_MODE_IT_FT,   /* interrupt - falling edge triggered */
   GPIO_MODE_IT_RT,   /* interrupt - rising edge triggered */
   GPIO_MODE_IT_RFT,  /* interrupt - rising and falling edge triggered */
   GPIO_MODE_SKIP,
   GPIO_MODE_INVALID
}GPIO_Mode_t;

typedef enum
{
   GPIO_SPEED_LOW,
   GPIO_SPEED_MEDIUM,
   GPIO_SPEED_HIGH,
   GPIO_SPEED_VHIGH,
   GPIO_SPEED_SKIP,
   GPIO_SPEED_INVALID
}GPIO_Speed_t;

typedef enum
{
   GPIO_PULL_NONE,
   GPIO_PULL_UP,
   GPIO_PULL_DOWN,
   GPIO_PULL_SKIP,
   GPIO_PULL_INVALID
}GPIO_Pull_t;

typedef enum
{
   GPIO_OUTTYPE_PUSHPULL,
   GPIO_OUTTYPE_OPENDRAIN,
   GPIO_OUTTYPE_SKIP,
   GPIO_OUTTYPE_INVALID
}GPIO_OutType_t;

typedef enum
{
   GPIO_ALTFUNC_0,
   GPIO_ALTFUNC_1,
   GPIO_ALTFUNC_2,
   GPIO_ALTFUNC_3,
   GPIO_ALTFUNC_4,
   GPIO_ALTFUNC_5,
   GPIO_ALTFUNC_6,
   GPIO_ALTFUNC_7,
   GPIO_ALTFUNC_8,
   GPIO_ALTFUNC_9,
   GPIO_ALTFUNC_10,
   GPIO_ALTFUNC_11,
   GPIO_ALTFUNC_12,
   GPIO_ALTFUNC_13,
   GPIO_ALTFUNC_14,
   GPIO_ALTFUNC_15,
   GPIO_ALTFUNC_SKIP,
   GPIO_ALTFUNC_INVALID
}GPIO_AltFunc_t;

typedef enum
{
   GPIO_PIN_STATE_RESET = 0,
   GPIO_PIN_STATE_SET = 1,
   GPIO_PIN_STATE_INVALID
}GPIO_PinState_t;

typedef enum
{
   GPIO_STATUS_OK,
   GPIO_STATUS_INVALID_PORT,
   GPIO_STATUS_INVALID_PIN_NUM,
   GPIO_STATUS_INVALID_MODE,
   GPIO_STATUS_INVALID_SPEED,
   GPIO_STATUS_INVALID_PULL,
   GPIO_STATUS_INVALID_OUTPUT,
   GPIO_STATUS_INVALID_ALTFUNC,
   GPIO_STATUS_INVALID_PIN,
   GPIO_STATUS_INVALID_PIN_STATE,
   GPIO_STATUS_INVALID_PRIO
}GPIO_Status_t;

typedef struct
{
   GPIO_Port_t Port;
   GPIO_PinNum_t PinNum;
   GPIO_Mode_t Mode;
   GPIO_Speed_t Speed;
   GPIO_Pull_t Pull;
   GPIO_OutType_t OutType;
   GPIO_AltFunc_t AltFunc;
}GPIO_Pin_t;


GPIO_Status_t GPIO_PortClockEnable(GPIO_Port_t Port);
GPIO_Status_t GPIO_PortClockDisable(GPIO_Port_t Port);
GPIO_Status_t GPIO_PortReset(GPIO_Port_t Port);
GPIO_Status_t GPIO_PortRead(GPIO_Port_t Port, uint16_t* Val);
GPIO_Status_t GPIO_PortWrite(GPIO_Port_t Port, uint16_t Val);

GPIO_Status_t GPIO_PinAssign(GPIO_Pin_t* Pin, GPIO_Port_t Port, GPIO_PinNum_t PinNum);
GPIO_Status_t GPIO_PinConfigDefault(GPIO_Pin_t* Pin);
GPIO_Status_t GPIO_PinConfig(GPIO_Pin_t* Pin, GPIO_Mode_t Mode, GPIO_Speed_t Speed, GPIO_Pull_t Pull, 
                              GPIO_OutType_t OutType, GPIO_AltFunc_t AltFunc);
GPIO_Status_t GPIO_PinApplyConfig(GPIO_Pin_t* Pin);

GPIO_Status_t GPIO_PinRead(GPIO_Pin_t* Pin, GPIO_PinState_t* State);
GPIO_Status_t GPIO_PinWrite(GPIO_Pin_t* Pin, GPIO_PinState_t State);
GPIO_Status_t GPIO_PinToggle(GPIO_Pin_t* Pin);

GPIO_Status_t GPIO_EnableInterrupt(GPIO_PinNum_t PinNum, uint8_t Priority);
GPIO_Status_t GPIO_DisableInterrupt(GPIO_PinNum_t PinNum);
GPIO_Status_t GPIO_ClearPendingInterrupt(GPIO_PinNum_t PinNum);


#endif /* GPIO_DRIVER_H */
