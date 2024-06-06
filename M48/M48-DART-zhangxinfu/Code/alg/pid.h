#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "math.h"

#define Integral_max         500
#define IntegralSeparation   20


typedef struct
{
	uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    float max_out;  
    float max_iout; 
	float deadband;
	float intergral_range;
	
    float ref;
    float fdb;
	float lastfdb;

    float out;
    float Pout;
    float Iout;
	float last_Iout;
    float Dout;
    float Dbuf[3];  
    float error[3];
}PidTypeDef;


void PID_Init(PidTypeDef *pid,  const float PID[5]);
float PID_Calc(PidTypeDef *pid, float ref, float set);
void PID_clear(PidTypeDef *pid);

#endif
