#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include "referee.h"
#include <math.h>

#define CHASSIS_NORMAL     0
#define CHASSIS_FOLLOW     1
#define CHASSIS_SPIN       2
#define CHASSIS_OPPO_SPIN  3
#define CHASSIS_NO_FORCE   4
#define SENTRY_CONTROL_NORMAL 		 5
#define SENTRY_CONTROL_NORMAL_SPIN   6
#define SENTRY_CONTROL_FOLLOW_SPIN   7

//typedef enum
//{
//  CHASSIS_NORMAL     0
//  CHASSIS_FOLLOW   	 1
//  CHASSIS_SPIN       2
//  CHASSIS_NO_FORCE   3
//  SENTRY_CONTROL 		 4
//}

typedef enum
{
	CHASSIS = 1,
	FOLLOW =2,
	NUL=0,
}switch_flag_t;

#define xrun_cnt   402.3f    //	402.3																					reduction_ratio 19.203f			5Ã×								500/£¨3.14*7.6£©*19.203
#define yrun_cnt		402.3f

#define CLOCKWISE          1
#define ANTICLOCKWISE      2

#define GIMBAL_HEAD_ANGLE   92.0f  //245   //161  342

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f 
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
//#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
//#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

#define K_M      0.3f
				


typedef struct
{
	float vx_set;
	float vy_set;
	float wz_set;
	uint32_t chassis_mode;
	uint32_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
	uint16_t last_gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	int16_t real_vx;
	int16_t real_vy;
}REAl_CHASSIS_SPEED_t;

typedef enum
{
	NO_STEP=0,
	X_STEP,
	Y_STEP,	
	XY_STEP,
}STEPSTAR;

extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern uint8_t fly_flag;
extern int8_t xrun_flag;
extern int8_t yrun_flag;

void chassis_control(void);
void chassis_move(void);
void Chassis_PID_Init(void);
#endif

