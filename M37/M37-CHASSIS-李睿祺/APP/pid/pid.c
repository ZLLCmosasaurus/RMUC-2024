#include "pid.h"
#include <struct_typedef.h>

PidTypeDef p_pid;
PidTypeDef b_pid;

MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
PidTypeDef Chassis_speed_x,Chassis_speed_y,Chassis_speed_z;

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//pid参数初始化
void PID_Init(PidTypeDef *pid,  const float PID[7])
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->max_out  = PID[3];
    pid->max_iout = PID[4]; 
	pid->deadband = PID[5];
	pid->i_split  = PID[6];
    pid->Dbuf[0]  = pid->Dbuf[1]          = pid->Dbuf[2]  = 0.0f;
    pid->error[0] = pid->error[1]         = pid->error[2] = 0.0f;
    pid->Pout     = pid->Iout = pid->Dout = pid->out      = 0.0f;
}

//pid计算
float PID_Calc(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->ref - pid->fdb;
	
	if(pid->error[0] > -pid->deadband && pid->error[0] < pid->deadband) //死区
	pid->error[0]=0;

	pid->Pout = pid->Kp * pid->error[0];
	pid->last_Iout=pid->Iout;
	
	//减少比例输出时，i的累加
	if(pid->Pout >= pid->max_out)
		pid->Pout = pid->max_out;
	if(pid->Pout <= -pid->max_out)
		pid->Pout = -pid->max_out;

	pid->Iout += pid->Ki * pid->error[0];

    if(pid->max_iout!=0)
        LimitMax(pid->Iout, pid->max_iout);
		
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	//	0.02
	if(pid->fdb < pid->i_split && pid->fdb > -pid->i_split) //死区执行积分分离
	{
		pid->out = pid->Pout + pid->Dout;
		pid->Iout=0;
	}
	
	else
		pid->out = pid->Pout + pid->Iout + pid->Dout;
	if(pid->max_out!=0)
        LimitMax(pid->out, pid->max_out);
	
    return pid->out;
}

//pid复位
void PID_clear(PidTypeDef *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0]  = pid->Dbuf[1]  = pid->Dbuf[2]  = 0.0f;
    pid->out      = pid->Pout     = pid->Iout     = pid->Dout = 0.0f;
    pid->fdb      = pid->ref      = 0.0f;
}

/* 自定义PID运算函数 */

