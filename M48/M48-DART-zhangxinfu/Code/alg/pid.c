#include "pid.h"

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
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = PID[3];
    pid->max_iout = PID[4]; 
	pid->deadband= PID[5];
	pid->intergral_range=PID[6];
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

//pid计算 暂时不用积分分离
uint8_t intergral_index=1;
float PID_Calc(PidTypeDef *pid, float ref,float fdb)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
		
	if(pid->error[0] > -pid->deadband && pid->error[0] < pid->deadband) //死区
		pid->error[0]=0;
	
	//暂时不用积分分离
	// if (fabs(pid->error[0])>pid->intergral_range)
	// 	intergral_index=0;
	// else
	// 	intergral_index=1;

	pid->Pout = pid->Kp * pid->error[0];
	pid->last_Iout=pid->Iout;
	
	//减少比例输出时，i的累加
	if(pid->Pout>=pid->max_out)
		pid->Pout=pid->max_out;
	if(pid->Pout<=-pid->max_out)
		pid->Pout=-pid->max_out;

	pid->Iout += pid->Ki * pid->error[0]*intergral_index;
	LimitMax(pid->Iout, pid->max_iout);
		
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];

	pid->out = pid->Pout + pid->Iout + pid->Dout;
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
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->ref = 0.0f;
		pid->intergral_range=0;
}



