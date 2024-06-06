#include "control_task.h"
#include <string.h>
#include <math.h>

//#define NOMAL
//#define DEBUG

Chariot chariot;
extern uint8_t usart3_RX_buf[USART_RX_BUF_LENGTH];

float Init_Angle = 0 ;

float Dart_parameter[4][4] = {
    // yaw_angle , high_speed ,low_speed ,push_length
    {-1.8, 8125, 7825, push_length_1},//劳5
    {-1.4, 7575, 7275, push_length_2},//劳6
    {-1.8, 7650, 7350, push_length_3},//劳7
    {-1.82, 7530, 7230, push_length_4},//劳8
};
int8_t push_command = 1;
uint8_t ID = 0;
uint8_t Start_Flag = 0;
uint8_t Last_Status = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
#ifdef NOMAL
        Dart_FSM_TIM_Status_PeriodElapsedCallback(&chariot);
#endif
#ifdef DEBUG
        chariot.dart_push.push_command = push_command; // DEBUG push电机开启指令
        chariot.dart_push.target_length = -push_length_4;
#endif
        Fric_Task(&chariot.dart_fric);
        Push_Task(&chariot.dart_push);
        Yaw_Task(&chariot.dart_yaw);
#ifdef NOMAL
        CAN_Send_Msg(&hcan1, CAN1_TX_Fric, CAN1_Fric_Tx_Data, 8);
        CAN_Send_Msg(&hcan2, CAN2_TX_Yaw, CAN2_Yaw_Tx_Data, 8);
#endif
        CAN_Send_Msg(&hcan2, CAN2_TX_Push_Pitch, CAN2_Push_Pitch_Tx_Data, 8);
        Last_Status = dart_client_cmd_t.dart_launch_opening_status;
    }
}

void Dart_FSM_TIM_Status_PeriodElapsedCallback(Chariot *chariot)
{
    Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Time++;
    switch (Dart_FSM.Now_Status_Serial)
    {
    case 0:
        /*code*/
         Set_Chariot_Yaw_Fric_Parameter(chariot, ID, FRIC_CLOSE, push_stop); // 状态一：
        if(Last_Status==1&&dart_client_cmd_t.dart_launch_opening_status==2)
            Start_Flag = 1;
        if (Start_Flag == 1 &&
            dart_client_cmd_t.dart_launch_opening_status==Dart_State_Open)
            Set_Status(1);
        break;
    case 1:
        /*code*/
        Set_Chariot_Yaw_Fric_Parameter(chariot, ID, FRIC_OPEN, push_stop);      // 状态二：预设飞镖各个电机参数并驱动firc和yaw电机
        if (fabsf(chariot->dart_yaw.now_angle - chariot->dart_yaw.target_angle) < 0.005) // 判断yaw轴转到指定位置后开始推飞镖
            
						{
							if((ID == 0 || ID == 2)&&
									(Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Time>=3000))
								Set_Status(2);
							else if((ID == 1|| ID == 3) &&
								    (Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Time>=1000))
								Set_Status(2);
						}
        break;
    case 2:
        /*code*/
        Set_Chariot_Yaw_Fric_Parameter(chariot, ID, FRIC_OPEN, push_start); // 状态三：推飞镖
        if(//(Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Time>=9000)&&
            (fabsf(chariot->dart_push.actual_length - Dart_parameter[ID][3]) < 0.001))
            {
                Set_Status(0);
                ID++;
                if(ID%2==0) Start_Flag = 0;
            }
        break;
    }
}
void Task_Init(Chariot *chariot)
{
    Fric_Init(&chariot->dart_fric); // pid初始化
    Push_Init(&chariot->dart_push);
    Yaw_Init(&chariot->dart_yaw);
    Pitch_Init(&chariot->dart_pitch);
}
/*************************************针对于每一发飞镖设置相应的参数 并 进行处理**************************************************************/
/**************************************function_函数********************************************88*/
void Set_Chariot_Yaw_Fric_Parameter(Chariot *chariot, int8_t DART_ID, int8_t FRIC_STATUS, int8_t PUSH_CMD)
{
    chariot->dart_fric.fric_status = (Fric_status)FRIC_STATUS;
    chariot->dart_push.push_command = PUSH_CMD;
    // 摩擦轮电机模式选择//set :fric_speed :high and low
    if (chariot->dart_fric.fric_status == FRIC_OPEN)
    {
        chariot->dart_fric.fric_motor[0].target_speed = (int16_t)Dart_parameter[DART_ID][1];
        chariot->dart_fric.fric_motor[1].target_speed = -(int16_t)Dart_parameter[DART_ID][2];
        chariot->dart_fric.fric_motor[2].target_speed = (int16_t)Dart_parameter[DART_ID][2];
        chariot->dart_fric.fric_motor[3].target_speed = -(int16_t)Dart_parameter[DART_ID][1];
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            chariot->dart_fric.fric_motor[i].target_speed = 0;
        }
    }
    // set :position :yaw pitch
    chariot->dart_yaw.target_angle = Init_Angle+Dart_parameter[DART_ID][0];
    chariot->dart_push.target_length = Dart_parameter[DART_ID][3];
}


