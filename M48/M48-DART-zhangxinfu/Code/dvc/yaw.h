#ifndef _YAW_H
#define _YAW_H

#include "receive_can.h"
#include "pid.h"

#define YAW_OUTPOST_ANGLE   (YAW_MINDLE_ANGLE+6.5f)     //前哨站角度
#define YAW_BASE_ANGLE      (YAW_MINDLE_ANGLE-7.3f)     //基地角度

#define YAW_MIN_ANGLE        0.0f
#define YAW_MINDLE_ANGLE     -10.0f
#define YAW_MAX_ANGLE        -20.0f

#define   YAW_GEAR_RATIO             (72./1052.)

#define YAW_ENCODER_TO_ANGLE     (360./8191.)   //编码器值转角度

typedef struct
{
    MOTOR_t yaw_motor;

    float target_angle;
    float now_angle;

} Dart_Yaw_t;

void Yaw_Init(Dart_Yaw_t *yaw);
void Yaw_Task(Dart_Yaw_t *yaw);
void Calculate_Yaw_Angle(Dart_Yaw_t *yaw);
void Yaw_Target_Angle_To_Encoder(Dart_Yaw_t *yaw);

#endif
