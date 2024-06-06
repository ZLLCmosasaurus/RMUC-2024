#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "referee.h"
#include "stdbool.h"

#define SPIN_TO_FOLLOW     1
#define MODE_NO_SWITCH     0

typedef __packed struct
{
    uint8_t game_process;
    uint16_t time;
    uint16_t self_blood;
    uint16_t self_outpost_HP;
    uint8_t armor_id:4;
    uint8_t HP_deduction_reason:4;
}can_tx1_t;

typedef __packed struct
{
    uint16_t self_base_HP;
    uint16_t oppo_outpost_HP;
    uint16_t projectile_allowance_17mm;
    uint8_t invincible_state;
    uint8_t outpost_rfid:1;
    uint8_t color:1;
}can_tx2_t;

#define CAN_3508Motor1_ID     0x201
#define CAN_3508Motor2_ID     0x202
#define CAN_3508Motor3_ID     0x203
#define CAN_3508Motor4_ID     0x204
#define CAN_GIMBAL_Y_ID       0x208
#define GIMBAL_R_ID_1         0x150
#define GIMBAL_R_ID_2         0x152
#define GIMBAL_R_ID_3         0x154
#define GIMBAL_T_ID_1         0x008
#define GIMBAL_T_ID_2         0x010
	
#define LOOP_BACK_ID                    0x003
#define CHASSIS_TO_GIMBAL_ID_2          0x009
uint8_t bsp_can_init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);
void canTX_gimbal1(void);
void canTX_gimbal2(void);
void canTX_gimbal3(void);
void canTX_gimbal4(void);

uint8_t canTX_gimbal_y(int16_t yaw);
#endif

