#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "tim.h"
#include "receive_can.h"
#include "push.h"
#include "pitch.h"
#include "yaw.h"
#include "shoot.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "alg_fsm.h"
#define not_launch 0
#define First_launch 1
#define Second_launch 2

#define client_cmd_not_launch_dart 0
#define client_cmd_launch_dart 1

#define push_length_1 0.16
#define push_length_2 0.32
#define push_length_3 0.48
#define push_length_4 0.58
#define push_start 1
#define push_stop 0
#define Dart_one 0
#define Dart_two 1
#define Dart_three 2
#define Dart_four 3

typedef struct
{
    Dart_Fric_t     dart_fric;
    Dart_Push_t     dart_push;
    Dart_Yaw_t      dart_yaw;
    Dart_Pitch_t    dart_pitch;
    
    int8_t dart_launch_order;
    int8_t current_mode;
} Chariot;
extern Chariot chariot;

extern uint8_t ID;
extern uint8_t Start_Flag;

void Task_Init(Chariot* chariot);
// void dart_launch_task(Chariot* chariot);
// void Triggers_dart_emission(Chariot* chariot);
// void Dart_FSM_TIM_Status_PeriodElapsedCallback(Chariot* chariot);
void Set_Chariot_Yaw_Fric_Parameter(Chariot* chariot,int8_t DART_ID,int8_t FRIC_STATUS,int8_t PUSH_STATUS);
// void Triggers_dart_emission(Chariot* chariot)
void Dart_FSM_TIM_Status_PeriodElapsedCallback(Chariot *chariot);
#endif
