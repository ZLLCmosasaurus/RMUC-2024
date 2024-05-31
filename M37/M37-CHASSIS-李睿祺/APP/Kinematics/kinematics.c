#include "kinematics.h"
Kinematics_t Kinematics;

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Get_Base_Velocities(void);
int find_max(void);

//static float ang_t = 3.5;
//static uint8_t ang_f = 0;
//static uint8_t ang_s = 1;
//static float x_t = 0;
//static float y_t = 0;

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
    float vz = 0;
	
	Chassis_speed_x.ref = linear_x;
	Chassis_speed_y.ref = linear_y;
	Chassis_speed_z.ref = angular_z;

	Chassis_speed_x.fdb = Kinematics.actual_velocities.linear_x;
	Chassis_speed_y.fdb = Kinematics.actual_velocities.linear_y;
	Chassis_speed_z.fdb = Kinematics.actual_velocities.angular_z;

	PID_Calc(&Chassis_speed_x);
	PID_Calc(&Chassis_speed_y);
	PID_Calc(&Chassis_speed_z);
    
	Kinematics.wheel1.target_speed.linear_vel = -(linear_x + Chassis_speed_x.out) - (linear_y + Chassis_speed_y.out) + (vz + angular_z + Chassis_speed_z.out)*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel =  (linear_x + Chassis_speed_x.out) - (linear_y + Chassis_speed_y.out) - (vz + angular_z + Chassis_speed_z.out)*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel =  (linear_x + Chassis_speed_x.out) + (linear_y + Chassis_speed_y.out) + (vz + angular_z + Chassis_speed_z.out)*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = -(linear_x + Chassis_speed_x.out) + (linear_y + Chassis_speed_y.out) - (vz + angular_z + Chassis_speed_z.out)*(half_width+half_length);

//  Kinematics.wheel1.target_speed.linear_vel = -linear_x - linear_y + angular_z*(half_width+half_length);
//	Kinematics.wheel2.target_speed.linear_vel =  linear_x - linear_y - angular_z*(half_width+half_length);
//	Kinematics.wheel3.target_speed.linear_vel =  linear_x + linear_y + angular_z*(half_width+half_length);
//	Kinematics.wheel4.target_speed.linear_vel = -linear_x + linear_y - angular_z*(half_width+half_length);
	//线速度 cm/s  转转度  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	chassis_motor1.target_speed = - (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor2.target_speed =   (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor3.target_speed = - (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	chassis_motor4.target_speed =  	(int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);

}

//正运动学公式
//通过轮胎的实际转速计算底盘几何中心的三轴速度
void Get_Base_Velocities(void)
{
	//根据电机转速测算轮子转速
	Kinematics.wheel1.actual_speed.rpm =  (float)chassis_motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm = -(float)chassis_motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =  (float)chassis_motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = -(float)chassis_motor4.actual_speed / M3508_REDUCTION_RATIO;
	//轮子转速转换为轮子线速度
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//轮子线速度转换为底盘中心三轴的速度
	Kinematics.actual_velocities.linear_x  = (Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
                - Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = (Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel - Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
    Kinematics.actual_velocities.angular_z = (Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*chassis_radius);
}

int find_max(void)
{
  int temp=0;
  
  temp=abs(chassis_motor1.target_speed);
  if(abs(chassis_motor2.target_speed)>temp)
    temp=abs(chassis_motor2.target_speed);
  if(abs(chassis_motor3.target_speed)>temp)
    temp=abs(chassis_motor3.target_speed);
  if(abs(chassis_motor4.target_speed)>temp)
    temp=abs(chassis_motor4.target_speed);
  return temp;
}


