/*
 * system.h
 *
 *  Created on: Oct 28, 2023
 *  Author: marinicadragoslav@gmail.com
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

void SysNvic_SetPriority(int8_t IrqNum, uint8_t Prio);
void SysClock_Set168Mhz(void);
uint32_t SysClock_GetSystemMs(void);


#endif /* SYSTEM_H */