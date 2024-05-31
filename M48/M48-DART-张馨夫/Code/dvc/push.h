#ifndef _PUSH_CONTRAL_H_
#define _PUSH_CONTRAL_H_

#include "receive_can.h"
#include "pid.h"

#define PUSH_MOTOR_SPEED         8500       //push电机的转速 rpm
#define PUSH_MOTOR_STOP         0           //push电机的停止转速 rpm

#define PUSH_DART_LENGTH        0.18f        //一个飞镖所需行程 单位m
#define LENGTH_D                0.05f        //装满四个飞镖后第一个镖头距离丝杠最高点的距离

#define PUSH_ROUND_TO_LENGTH    8e-3f       //输出轴一圈对应的丝杆行程 单位m

#define PUSH_MOTOR_REDUCTION    36          //2006减速比 1:36
#define PUSH_ENCODER_COUNT      8191        //编码器一圈的值


typedef enum
{
    PUSH_CLOSE = 0,
    PUSH_FORWARD = 1,
    PUSH_BACKWARD = 2,
}Push_status;
typedef enum
{
    PUSH_PHASE_ZERO=0,
    PUSH_PHASE_ONE=1,
    PUSH_PHASE_TWO=2,
}Push_phase;
typedef struct
{   
    Push_status push_status;
    MOTOR_t push_motor; 

    int8_t push_command;
    float target_length;
    float actual_length;
}Dart_Push_t;

void Push_Init(Dart_Push_t *push);
void Calculate_Length(Dart_Push_t *push);
void Push_Task(Dart_Push_t *push);



#endif
