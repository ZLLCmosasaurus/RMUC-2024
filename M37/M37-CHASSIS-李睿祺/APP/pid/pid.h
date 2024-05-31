#ifndef __pid_H
#define __pid_H
#include "main.h"
#include "usart.h"
#include <string.h>

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
	float deadband; //死区
	float i_split;  //积分分离区限
	
    float ref;      //目标值
    float fdb;      //实际值
	float lastfdb;  //上一实际值

    float out;      //实际输出
    float Pout;     //P输出
    float Iout;     //I输出
	float last_Iout;//上一I输出
    float Dout;     //D输出
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
}PidTypeDef;

typedef struct{
	
	float start_angle;			//电机初始角度值
	
	int start_angle_x;      //记录电机开始执行x_run命令时，相应电机角度
	int start_angle_y;      //记录电机开始执行y_run命令时，相应电机角度
	
	int start_angle_flag;	  //记录电机初始角度值的flag
	int stop_angle;				  //发送停止命令时候的角度值
	float target_angle;
	
	float actual_angle;		  //当前真实角度值
	float last_angle;			  //上一次返回的角度值
	float switch_mode_angle;//记录模式转换角度值
	int round_cnt;				  //相对开机时转过的圈数
	int total_angle;			  //总共转过的计数
	
	float actual_speed;		  //电机真实速度,rpm
	int target_speed;			  //电机目标速度,rpm  转/min
	int last_speed;         //电机上一次回传的速度值
	int actual_current;		  //电机真实电流
	int target_current;		  //电机目标电流
	//int temp;							//电机温度（2006电机不支持，3508支持）
    PidTypeDef a_pid;
	PidTypeDef v_pid;

	uint8_t spin_dirt;
}MOTOR_t;

extern PidTypeDef p_pid;
extern PidTypeDef b_pid;
extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
extern PidTypeDef Chassis_speed_x,Chassis_speed_y,Chassis_speed_z;

extern void PID_Init(PidTypeDef *pid,  const float PID[5]);
extern float PID_Calc(PidTypeDef *pid);
extern void PID_clear(PidTypeDef *pid);

#endif
