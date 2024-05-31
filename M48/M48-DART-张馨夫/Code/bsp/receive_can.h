#ifndef _RECEIVE_CAN_H
#define _RECEIVE_CAN_H

#include "can.h"
#include "pid.h"

#define shoot_RX_id 	0x200
#define push_RX_id		0x1FF

#define CAN1_TX_Fric 	0X200

#define CAN1_RX_Fric_1 	0X201
#define CAN1_RX_Fric_2 	0x202
#define CAN1_RX_Fric_3 	0X203
#define CAN1_RX_Fric_4	0X204

#define CAN2_TX_Push_Pitch 	0x200
#define CAN2_RX_Push 		0x201
#define CAN2_RX_Pitch 		0x202

#define CAN2_RX_Yaw 	0x205
#define CAN2_TX_Yaw 	0x1ff


typedef struct
{
	uint8_t start_encoder_flag;
	uint16_t start_encoder;
	uint16_t last_encoder;
	uint16_t actual_encoder;
	int total_encoder;
	int round_cnt;
	int16_t actual_speed;
	int16_t actual_current;

	int16_t target_current;
	int16_t target_speed;
	int target_encoder;

	PidTypeDef vpid;
	PidTypeDef apid;
} MOTOR_t;


extern uint8_t CAN1_Fric_Tx_Data[8];
extern uint8_t CAN2_Push_Pitch_Tx_Data[8];
extern uint8_t CAN2_Yaw_Tx_Data[8];
extern uint8_t CAN2_0x1ff_data[8];

void CAN_filter(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Record_Motor_Callback(MOTOR_t *motor, uint8_t *data);
uint8_t CAN_Send_Msg(CAN_HandleTypeDef *__hcan, uint32_t __StdId, uint8_t *__msg, uint8_t __len);

#endif
