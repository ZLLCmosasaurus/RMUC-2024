#include "pitch.h"
#include "math.h"

float set_pitch_angle = 29.71;

const float D_1 = 0.0f;

//pitch电机 pid参数 速度环 角度环
const float PITCH_APID_PID[7]={280,0,0,11000,2000,0};
const float PITCH_VPID_PID[7]={15,0.9,0,11000,2000,0};


void Pitch_Init(Dart_Pitch_t *pitch)
{
    //初始化pitch电机 速度环角度环
    PID_Init(&pitch->pitch_motor.apid, PITCH_APID_PID);
    PID_Init(&pitch->pitch_motor.vpid, PITCH_VPID_PID);
    //pitch->target_angle = 24.472;
}

 void Pitch_Task(Dart_Pitch_t *pitch)
 {
    pitch->target_angle = set_pitch_angle;
     //计算当前角度
    Calculate_Pitch_Now_Angle(pitch);
	
     //角度环   
     PID_Calc(&pitch->pitch_motor.apid, pitch->target_angle, pitch->now_angle);
    
     //速度环
     pitch->pitch_motor.target_speed = pitch->pitch_motor.apid.out;
     PID_Calc(&pitch->pitch_motor.vpid, pitch->pitch_motor.target_speed, pitch->pitch_motor.actual_speed);

     //发送缓冲区
     CAN2_Push_Pitch_Tx_Data[2] = (int16_t)pitch->pitch_motor.vpid.out >> 8;
     CAN2_Push_Pitch_Tx_Data[3] = (int16_t)pitch->pitch_motor.vpid.out;
 }



 double length;
 void Calculate_Pitch_Now_Angle(Dart_Pitch_t *pitch)
 {
     double d_1,temp_value;
     double alpha_rad,beta_rad;

     //计算丝杆当前行程 单位m
     length = (double)(pitch->pitch_motor.total_encoder - pitch->pitch_motor.start_encoder) / PITCH_ENCODER_COUNT * PITCH_GEAR_RATIO * PITCH_ROUND_TO_LENGTH;
     pitch->now_length = PITCH_BASE_D_1 - length;

     //计算丝杆和水平面的夹角 单位rad
     beta_rad = atan(PITCH_BASE_HIGHT / pitch->now_length);

     //勾股定理计算d_1长度
     d_1 = (double)sqrt((pitch->now_length * pitch->now_length)+(PITCH_BASE_HIGHT * PITCH_BASE_HIGHT));

     //计算当前角度
     temp_value = (d_1 * d_1 + PITCH_D_3 * PITCH_D_3 - PITCH_D_2 * PITCH_D_2) / (2 * d_1 * PITCH_D_3);
     alpha_rad = acos(temp_value);

     pitch->now_angle = (alpha_rad + beta_rad) * 180.0f / 3.1415926f - PITCH_OFFSET_ANGLE;
 }

void Pitch_Target_Angle_To_Encoder(Dart_Pitch_t *pitch)
{   
    //计算目标行程 单位m
    float a = 1.f;
    float b = -2.0f * PITCH_D_3 * cos((pitch->target_angle + PITCH_OFFSET_ANGLE) * 3.1415926f / 180.0f);
    float c = PITCH_D_3 * PITCH_D_3 - PITCH_D_2 * PITCH_D_2;
    pitch->target_length = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    //计算目标编码器值
    pitch->pitch_motor.target_encoder = (int)(pitch->target_length - PITCH_BASE_D_1) / PITCH_ROUND_TO_LENGTH / PITCH_GEAR_RATIO * PITCH_ENCODER_COUNT + pitch->pitch_motor.start_encoder; 
}







