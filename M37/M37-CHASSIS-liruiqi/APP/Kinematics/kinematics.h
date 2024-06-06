#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "chassis_move.h"
#include <stdlib.h>
#define wheel_diameter  15.100000f			    //����ֱ��(cm)
#define half_width      15.500000f		        //���̰��(cm)
#define half_length     16.000000f	            //���̰볤(cm)
#define chassis_radius  22.280000f              //���̰뾶(cm)(������������ļ�ľ���)

#define PI 	    3.141593f
#define RPM2RAD 0.104720f						//ת��ת���ٶ�		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.403171f						//ת��ת���ٶ�		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 2.48033f						//���ٶ�תת��
#define M2006_REDUCTION_RATIO 36.000000f		//��������ٱ�
#define M3508_REDUCTION_RATIO 19.203209f		//��������ٱ�
#define GM6020_ENCODER_ANGLE  8192.0f


#define MAX_MOTOR_SPEED             14336		//������ת�٣��궨�巽���޸�   ��Χ0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED       120.817f    //�������ƽ���ٶȣ���λcm/s   
#define MAX_BASE_ROTATIONAL_SPEED   7.260570f   //���������ת�ٶȣ���λrad/s    
#define NORMAL_LINEAR_SPEED         70.0f
#define NORMAL_ROTATIONAL_SPEED     0.5f
				
typedef struct
{
	float linear_vel;			                //���ٶ�
	float rpm;						            //ת��Ȧÿ����
}Speed_t;

typedef struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//���̼������ĵ���/���ٶ�
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //���ٶ�rpm
}Velocities_t;

typedef struct
{
	float target_angular;
	float actual_angular;
  float target_angle;
	float actual_angle;
}Application_t;

typedef struct
{
	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	Wheel_t wheel4;
	
	Velocities_t target_velocities;		//Ŀ�����ٶ�
	Velocities_t actual_velocities; 	//ʵ�����ٶ�
}Kinematics_t;

extern Kinematics_t Kinematics;
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
int find_max(void);

#endif
