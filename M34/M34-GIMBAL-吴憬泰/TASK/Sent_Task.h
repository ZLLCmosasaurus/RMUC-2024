#ifndef __sent_task_H
#define __sent_task_H

#include "stm32f4xx.h"

extern uint8_t can_tx_0x114_data[];

uint8_t canTX_chassis_first(int16_t x, int16_t y, int16_t z, int16_t current_t);
uint8_t canTX_chassis_second(uint8_t mode, uint8_t vision_mode);
uint8_t canTX_gimbal_p(int32_t pitch);
uint8_t canTX_gimbal_p_2(int16_t pitch);
uint8_t canTX_fric(int16_t left, int16_t right, int16_t trigger);
uint8_t canTX_trigger(int16_t trigger);
uint8_t canTX_UPPER_COMPUTER(void);
void canTX_UPPER_COMPUTER_2(void);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
#endif
