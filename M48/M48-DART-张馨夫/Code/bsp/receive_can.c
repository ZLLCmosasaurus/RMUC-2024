#include "receive_can.h"
#include "control_task.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Chariot chariot;
	
uint8_t CAN1_Fric_Tx_Data[8];
uint8_t CAN2_Push_Pitch_Tx_Data[8];
uint8_t CAN2_Yaw_Tx_Data[8];
uint8_t CAN2_0x1ff_data[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef  RXmessage;
	uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RXmessage,rx_data);
	
	if(hcan==&hcan1)
	{
		switch (RXmessage.StdId)
			{
				case CAN1_RX_Fric_1:
				Record_Motor_Callback(&chariot.dart_fric.fric_motor[0],rx_data);
				break;
				case CAN1_RX_Fric_2:
				Record_Motor_Callback(&chariot.dart_fric.fric_motor[1],rx_data);
				break;
				case CAN1_RX_Fric_3:
				Record_Motor_Callback(&chariot.dart_fric.fric_motor[2],rx_data);
				break;
				case CAN1_RX_Fric_4:
				Record_Motor_Callback(&chariot.dart_fric.fric_motor[3],rx_data);
				break;
				default :
				break;
			}
	}
	else if(hcan==&hcan2)
	{
		switch (RXmessage.StdId)
		    {
				case CAN2_RX_Push:
				Record_Motor_Callback(&chariot.dart_push.push_motor,rx_data);
				break;
				case CAN2_RX_Pitch:
				Record_Motor_Callback(&chariot.dart_pitch.pitch_motor,rx_data);
				break;
				case CAN2_RX_Yaw:
				Record_Motor_Callback(&chariot.dart_yaw.yaw_motor,rx_data);
				break;
				default :
				break;
		    }
	}
}

void Record_Motor_Callback(MOTOR_t *motor, uint8_t *data)
{

	motor->last_encoder = motor->actual_encoder;
	motor->actual_encoder = (int16_t)((data[0] << 8) | data[1]);
	motor->actual_speed = (int16_t)((data[2] << 8) | data[3]);
	motor->actual_current = (int16_t)((data[4] << 8) | data[5]);

	//记录初始角度
	if(motor->start_encoder_flag == 0)
	{
		motor->start_encoder = (int16_t)((data[0] << 8) | data[1]);
		motor->start_encoder_flag++;	//只在启动时记录一次初始角度
	}
  else
	{
		//计算总角度
		if(motor->actual_encoder - motor->last_encoder > 4096)
		{
			motor->round_cnt --;
		}
		else if (motor->actual_encoder - motor->last_encoder < -4096)
		{
			motor->round_cnt ++;
		}
		motor->total_encoder = motor->round_cnt * 8192 + motor->actual_encoder;
	}
}


uint8_t CAN_Send_Msg(CAN_HandleTypeDef *__hcan, uint32_t __StdId, uint8_t *__msg, uint8_t __len)
{
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t TxMailbox;
	TxMessage.StdId = __StdId;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = __len;	

	if(HAL_CAN_AddTxMessage(__hcan, &TxMessage, __msg, &TxMailbox) != HAL_OK)
	{
		return 0;
	}
	return 1;
}


void CAN_filter(void)
{
	CAN_FilterTypeDef canFilter;
	canFilter.FilterBank=1;    					//筛选器组1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  //掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE; //开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT;	//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 
	canFilter.SlaveStartFilterBank=14;
	HAL_CAN_ConfigFilter(&hcan1,&canFilter);	//配置过滤器
	/*can2初始化*/
	canFilter.FilterBank=14;    															//筛选器组15
	HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断}
}

