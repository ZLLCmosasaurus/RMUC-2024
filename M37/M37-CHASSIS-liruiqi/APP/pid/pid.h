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
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������
	float deadband; //����
	float i_split;  //���ַ�������
	
    float ref;      //Ŀ��ֵ
    float fdb;      //ʵ��ֵ
	float lastfdb;  //��һʵ��ֵ

    float out;      //ʵ�����
    float Pout;     //P���
    float Iout;     //I���
	float last_Iout;//��һI���
    float Dout;     //D���
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
}PidTypeDef;

typedef struct{
	
	float start_angle;			//�����ʼ�Ƕ�ֵ
	
	int start_angle_x;      //��¼�����ʼִ��x_run����ʱ����Ӧ����Ƕ�
	int start_angle_y;      //��¼�����ʼִ��y_run����ʱ����Ӧ����Ƕ�
	
	int start_angle_flag;	  //��¼�����ʼ�Ƕ�ֵ��flag
	int stop_angle;				  //����ֹͣ����ʱ��ĽǶ�ֵ
	float target_angle;
	
	float actual_angle;		  //��ǰ��ʵ�Ƕ�ֵ
	float last_angle;			  //��һ�η��صĽǶ�ֵ
	float switch_mode_angle;//��¼ģʽת���Ƕ�ֵ
	int round_cnt;				  //��Կ���ʱת����Ȧ��
	int total_angle;			  //�ܹ�ת���ļ���
	
	float actual_speed;		  //�����ʵ�ٶ�,rpm
	int target_speed;			  //���Ŀ���ٶ�,rpm  ת/min
	int last_speed;         //�����һ�λش����ٶ�ֵ
	int actual_current;		  //�����ʵ����
	int target_current;		  //���Ŀ�����
	//int temp;							//����¶ȣ�2006�����֧�֣�3508֧�֣�
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
