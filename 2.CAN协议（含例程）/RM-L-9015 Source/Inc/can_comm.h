#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32F0xx_hal.h"

#define POSITION_MODE   0
#define VELOCITY_Mode   1

#define CPR             32768

void CanComm_Init(void);

void CANComm_SetMode(uint32_t nodeid, uint8_t mode);
void CANComm_SetPostion(uint32_t nodeid, int position);
void CANComm_SetVelocity(uint32_t nodeid, int velocity);
void CANComm_GetPosition(uint32_t nodeid);
void CANComm_GetVelocity(uint32_t nodeid);
void CANComm_GetCurrent(uint32_t nodeid);

#endif
