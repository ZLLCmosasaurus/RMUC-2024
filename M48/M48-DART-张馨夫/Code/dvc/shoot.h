#ifndef _SHOOT_CONTRAL_H_
#define _SHOOT_CONTRAL_H_

#include "receive_can.h"
#include "pid.h"

#define FRIC_MOTOR_SPEED  7000     //rpm
#define FRIC_MOTOR_STOP   0     //rpm

typedef enum
{
    FRIC_CLOSE = 0,
    FRIC_OPEN = 1,
}Fric_status;

typedef struct
{
    Fric_status fric_status;
    MOTOR_t fric_motor[4];
    
    int16_t FRIC_Speed_high;
    int16_t FRIC_Speed_low;
    int8_t shoot_command;
}Dart_Fric_t;

void Fric_Init(Dart_Fric_t* fric);
void Fric_Task(Dart_Fric_t* fric);


#endif
