#ifndef _PITCH_H
#define _PITCH_H

#include "receive_can.h"
#include "pid.h"
#include "math.h"

#define PITCH_OUTPOST_ANGLE        17.0f    //前哨站角度
#define PITCH_BASE_ANGLE           20.0f    //基地角度

#define PITCH_MIN_ANGLE        0.0f
#define PITCH_MINDLE_ANGLE     35.0f
#define PITCH_MAX_ANGLE        45.0f

#define PITCH_BASE_HIGHT    1.e-2f
#define PITCH_BASE_D_1      730.05e-3f        //初始化的时候d_1的长度  
#define PITCH_D_2           370e-3f            
// #define PITCH_D_3           467.325e-3f
#define PITCH_D_3          673.304e-3f

#define PITCH_OFFSET_ANGLE      1.3f        //pitch偏移角度 单位度

#define PITCH_ROUND_TO_LENGTH    8e-3f       //pitch输出轴一圈对应的丝杆行程 单位m

#define PITCH_GEAR_RATIO            (187./3591.)    //3508减速比
#define PITCH_ENCODER_TO_ANGLE     (360./8191.)      //编码器值转角度
#define PITCH_ENCODER_COUNT        8191             //3508一圈编码器值

typedef struct
{
    MOTOR_t pitch_motor;
    float target_length;
    float now_length;

    float target_angle;
    float now_angle;

} Dart_Pitch_t;

void Pitch_Init(Dart_Pitch_t *pitch);
void Pitch_Task(Dart_Pitch_t *pitch);
void Calculate_Pitch_Now_Angle(Dart_Pitch_t *pitch);
void Pitch_Target_Angle_To_Encoder(Dart_Pitch_t *pitch);
#endif
