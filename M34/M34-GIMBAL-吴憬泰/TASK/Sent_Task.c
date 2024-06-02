#include "sent_task.h"
#include "can.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "upper_computer.h"
#include "rc_task.h"
#include "shoot_task.h"
#include "vision_task.h"
#include "gimbal_task.h"
/*
0 zc
1 sd
2 xtl
3 wl
*/
uint8_t data_[8] = {0}, state;
uint8_t can_tx_0x114_data[8] = {0};
extern uint16_t set_compare;

uint8_t shoot_type = FIVE_SHOOT;
extern enum {
	CLOSE = 0,
	OPEN
} bullet_state;

extern shoot_status_e shoot_status;

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
	CAN_TxHeaderTypeDef tx_header;
	uint32_t used_mailbox;

	// 检测传参是否正确
	assert_param(hcan != NULL);

	tx_header.StdId = ID;
	tx_header.ExtId = 0;
	tx_header.IDE = 0;
	tx_header.RTR = 0;
	tx_header.DLC = Length;

	return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

// 给底盘发送数据(x,v,z速度以及yaw轴电流)
uint8_t canTX_chassis_first(int16_t x, int16_t y, int16_t z, int16_t current_t)
{
	CAN_TxHeaderTypeDef canFrame;

	uint32_t temp = 0;

	canFrame.IDE = CAN_ID_STD;
	canFrame.StdId = 0x007;
	canFrame.RTR = CAN_RTR_DATA;
	canFrame.DLC = 8;
	canFrame.TransmitGlobalTime = DISABLE;
	data_[0] = x >> 8;
	data_[1] = x & 0xff;
	data_[2] = y >> 8;
	data_[3] = y & 0xff;
	data_[4] = z >> 8;
	data_[5] = z & 0xff;
	data_[6] = current_t >> 8;
	data_[7] = current_t & 0xff;
	state = HAL_CAN_AddTxMessage(&hcan2, &canFrame, data_, &temp);

	return temp;
}

extern uint8_t chassis_power_flag;
extern uint8_t supercap_reboot_flag;

uint8_t canTX_chassis_second(uint8_t mode, uint8_t vision_mode)
{
	uint8_t vision_mode_sent;

	if(vision_mode==VISION_OFF&&assist_vision_mode== ASSIST_VISION_OFF)
	{
		vision_mode_sent = VISION_OFF;
	}
	else if (vision_mode == VISION_ON && assist_vision_mode == ASSIST_VISION_OFF)
	{
		vision_mode_sent = VISION_ON;
	}
	else if(assist_vision_mode== ASSIST_VISION_ON&&vision_mode == VISION_OFF)
	{
		vision_mode_sent = ASSIST_VISION_ON;
	}


	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8] = {0};
	uint32_t temp = 0;

	canFrame.IDE = CAN_ID_STD;
	canFrame.StdId = 0x006;
	canFrame.RTR = CAN_RTR_DATA;
	canFrame.DLC = 8;
	canFrame.TransmitGlobalTime = DISABLE;
	data[0] = mode;
	data[1] = vision_mode_sent;
	data[2] = supercap_reboot_flag;
	data[3] = vision_sent.Control_priority;
	data[4] = bullet_state;
	data[5] = shoot_status;
	data[6] = shoot_type;
	data[7] = control_mode;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	return temp;
}

// pitch电机控制
// uint16_t maxspeed=400;
// uint8_t canTX_gimbal_p(int32_t pitch)
//{
//	CAN_TxHeaderTypeDef canFrame;
//	uint8_t data[8]={0};
//	uint32_t temp=0;
//
//	canFrame.IDE=CAN_ID_STD;
//	canFrame.StdId=0x141;
//	canFrame.RTR=CAN_RTR_DATA;
//	canFrame.DLC=8;
//	canFrame.TransmitGlobalTime=DISABLE;
//	data[0]=0xA8;
//	data[1]=0;
//	data[2]=(uint8_t)(maxspeed);
//	data[3]=(uint8_t)(maxspeed>>8);
//	data[4]=(uint8_t)((pitch));
//	data[5]=(uint8_t)((pitch)>>8);
//	data[6]=(uint8_t)((pitch)>>16);
//	data[7]=(uint8_t)((pitch)>>24);

//	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
//
//	return temp;
//}

uint8_t canTX_gimbal_p_2(int16_t pitch)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8] = {0};
	uint32_t temp = 0;

	canFrame.IDE = CAN_ID_STD;
	canFrame.StdId = 0x141;
	canFrame.RTR = CAN_RTR_DATA;
	canFrame.DLC = 8;
	canFrame.TransmitGlobalTime = DISABLE;
	data[0] = 0xA1;
	data[4] = (uint8_t)((pitch));
	data[5] = (uint8_t)((pitch) >> 8);

	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);

	return temp;
}

// 摩擦轮以及拨弹轮的数据发送
uint8_t canTX_fric(int16_t left, int16_t right, int16_t trigger)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8] = {0};
	uint32_t temp = 0;

	canFrame.IDE = CAN_ID_STD;
	canFrame.StdId = 0x200;
	canFrame.RTR = CAN_RTR_DATA;
	canFrame.DLC = 8;
	canFrame.TransmitGlobalTime = DISABLE;
	data[0] = right >> 8;
	data[1] = right & 0xfff;
	data[2] = left >> 8;
	data[3] = left & 0xfff;
	data[4] = trigger >> 8;
	data[5] = trigger & 0xfff;
	data[6] = 0;
	data[7] = 0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);

	return temp;
}

extern float shoot_speed;
int16_t Sent_dataA; // yaw
int16_t Sent_dataB; // pitch
int16_t Sent_dataC; // bullt_speed
uint8_t data_[8];
uint8_t canTX_UPPER_COMPUTER(void) // 给上位机发送数据
{
	CAN_TxHeaderTypeDef canFrame;

	uint32_t temp = 0;

	//		CAN_angle=(gimbal_y.CAN_actual_angle*360.0f/8192.0f)+imu_can_error_y;
	//	if(CAN_angle>180)
	//		CAN_angle -= 360.0f;
	//	if(CAN_angle<-180)
	//		CAN_angle += 360.0f;
	//
	//	if(fricspeed==FRIC_MIN)shoot_vel=15;
	//	else shoot_vel=30;

	Sent_dataA = (int16_t)(gimbal_y.IMU_actual_angle * 100.0f); // yaw
	Sent_dataB = (int16_t)(gimbal_p.IMU_actual_angle * 100.0f); // pitch
	Sent_dataC = (int16_t)(shoot_speed * 100.0f);				// bullt_speed

	canFrame.IDE = CAN_ID_STD;
	canFrame.StdId = 0x011; // 对于上位机我们的ID是0x011
	canFrame.RTR = CAN_RTR_DATA;
	canFrame.DLC = 8;
	canFrame.TransmitGlobalTime = DISABLE;

	data_[0] = Sent_dataA & 0xff;
	data_[1] = Sent_dataA >> 8;
	data_[2] = Sent_dataB & 0xff;
	data_[3] = Sent_dataB >> 8;
	data_[4] = Sent_dataC & 0xff;
	data_[5] = Sent_dataC >> 8;
	data_[6] = auto_exposure_flag;
	data_[7] = 0;

	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data_, &temp);
	//	if(reboot_flag==1) reboot_flag=0;
	return temp;
}

// 发送模式，重启，射速
uint8_t speed, reboot_old, mode_old, speed_old;
extern UPPER_COMPUTER_VISION_t shoot_vision_mode; // 上位机自瞄模式
void canTX_UPPER_COMPUTER_2(void)				  // 给上位机发送控制模式和射速
{
	static uint8_t count = 0;
	count++;
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8] = {0};
	uint32_t temp = 0;

	if (speed_limit == 15)
		speed = 0;
	else if (speed_limit == 30)
		speed = 2;

	if ((vision_mode != mode_old) || (speed != speed_old) || (reboot_flag != reboot_old) || (count == 10))
	{
		canFrame.IDE = CAN_ID_STD;
		canFrame.StdId = 0x013;
		canFrame.RTR = CAN_RTR_DATA;
		canFrame.DLC = 8;
		canFrame.TransmitGlobalTime = DISABLE;

		data[0] = shoot_vision_mode;
		data[1] = 0; // 已经有硬件重启，所以这位报废
		data[2] = speed;

		HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
		count = 0;
	}
	mode_old = vision_mode;
	speed_old = speed;
	reboot_old = reboot_flag;
}
