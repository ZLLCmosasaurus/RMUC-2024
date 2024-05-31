#include "yaw.h"
#include "math.h"
//Debug 初始化角度
// float Set_yaw_angle = 0;

//yaw电机pid参数 角度环 速度环
const float YAW_APID_PID[7] = {8,0,0,16500,2000,0};
const float YAW_VPID_PID[7] = {25,0.05,0,5000,2000,0};

void Yaw_Init(Dart_Yaw_t *yaw)
{
    //初始化yaw电机 速度环角度环
    PID_Init(&yaw->yaw_motor.apid, YAW_APID_PID);
    PID_Init(&yaw->yaw_motor.vpid, YAW_VPID_PID);
}


 void Yaw_Task(Dart_Yaw_t *yaw)
 {
     //计算当前角度
     Calculate_Yaw_Angle(yaw);
     //设定目标角度（测试）
	

     //目标角度转编码器值
     Yaw_Target_Angle_To_Encoder(yaw);
     //角度环
     PID_Calc(&yaw->yaw_motor.apid, yaw->yaw_motor.target_encoder, yaw->yaw_motor.total_encoder);

     //速度环
     yaw->yaw_motor.target_speed = yaw->yaw_motor.apid.out;
     PID_Calc(&yaw->yaw_motor.vpid, yaw->yaw_motor.target_speed, yaw->yaw_motor.actual_speed);

     //发送缓冲区
     CAN2_Yaw_Tx_Data[0] = (int16_t)yaw->yaw_motor.vpid.out >> 8;
     CAN2_Yaw_Tx_Data[1] = (int16_t)yaw->yaw_motor.vpid.out;
 }


void Calculate_Yaw_Angle(Dart_Yaw_t *yaw)
{
    //编码器值除以一圈8191，除以减速比，乘以丝杆系数    value/8191/36*8e-3
    //计算当前角度 单位度
    yaw->now_angle = (float)(yaw->yaw_motor.total_encoder - yaw->yaw_motor.start_encoder) * YAW_ENCODER_TO_ANGLE * YAW_GEAR_RATIO;
}

void Yaw_Target_Angle_To_Encoder(Dart_Yaw_t *yaw)
{
    //计算目标编码器值
    yaw->yaw_motor.target_encoder = (int)(yaw->target_angle / YAW_GEAR_RATIO / YAW_ENCODER_TO_ANGLE) + yaw->yaw_motor.start_encoder;
}

