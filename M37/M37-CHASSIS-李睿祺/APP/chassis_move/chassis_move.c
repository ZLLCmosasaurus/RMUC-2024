#include "chassis_move.h"
#include "fuzzy_pid.h"
#include "bsp_can.h"
#include "referee.h"
#include <math.h>

switch_flag_t switch_flag;

static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
//static float chassis_buffer_loop(uint16_t buffer);
static float Get_chassis_theta(void);

CHASSIS_CONTROL_ORDER_t chassis_control_order;

static float avx,avy;
static float last_vx,last_vy;

//�µĹ��������õ��Ĳ���   Rhn
static float abs_f(float a);
static void Power_limit(void);
static float Power_out_1  = 0;
static float Power_out_2  = 0;
static float Power_out_3  = 0;
static float Power_out_4  = 0;
static float Power_out    = 0;              ///Power_out֮��
//static float Power_out_square_sum = 0;      // Power_out��ƽ����
static float Power_in_1   = 0;
static float Power_in_2   = 0;
static float Power_in_3   = 0;
static float Power_in_4   = 0;
static float Power_in     = 0;              //Power_in֮��
static float Target_current_square_sum = 0; //Ŀ�������ƽ����
static float Actual_speed_square_sum   = 0; //ʵ�ʽ��ٶȵ�ƽ����
static float K_limit      = 0;                     //Ŀ�������Сϵ��


static float POWER_K1     = 2.8;
static float POWER_K2     = 0.01;
static float offset       = 0;
static float reduction_ratio = 19.203f;     //���ӵ�����ٱ�
extern uint16_t supercap_volt;              //���Ӱ뾶

/* PID(kp, ki, kd, max_out, max_iout, deadband, i_split) */
float c_center_apid[] = {0.035,0.0, 0.0, 1   , 0  , 0.5, 0};
float c_center_vpid[] = {2.80, 0.1, 0.1, 5000, 500, 0  , 0};

float c_motor1_vpid[] = {5.0, 0.35, 0.0,  16000,  1000, 0,  0};
float c_motor2_vpid[] = {5.0, 0.35, 0.0,  16000,  1000, 0,  0};
float c_motor3_vpid[] = {5.0, 0.35, 0.0,  16000,  1000, 0,  0};
float c_motor4_vpid[] = {5.0, 0.35, 0.0,  16000,  1000, 0,  0};

float power_pid[]     = {4.6, 1.8,  0,    2000,   180,  0,  12};
float buffer_pid[]    = {5.1, 1.2,  0,    1000,   100,  0,  0};

float PID_x[]={0.01,0.0,0.5,1,0,0,0};
float PID_y[]={0.01,0.0,0.5,1,0,0,0};
float PID_z[]={0.01,0.0,0.5,1,0,0,0};

float nrk=0;

void Chassis_PID_Init(void)
{
  PID_Init(&chassis_center.a_pid, c_center_apid);
  PID_Init(&chassis_center.v_pid, c_center_vpid);
  PID_Init(&chassis_motor1.v_pid, c_motor1_vpid);
  PID_Init(&chassis_motor2.v_pid, c_motor2_vpid);
  PID_Init(&chassis_motor3.v_pid, c_motor3_vpid);
  PID_Init(&chassis_motor4.v_pid, c_motor4_vpid);
  PID_Init(&p_pid, power_pid);
  PID_Init(&b_pid, buffer_pid);
  PID_Init(&Chassis_speed_x, PID_x);
  PID_Init(&Chassis_speed_y, PID_y);
  PID_Init(&Chassis_speed_z, PID_z);
}

static void Chassis_Pid_Calc(void)
{
                                       //??????
    PID_Calc(&chassis_motor1.v_pid);
	PID_Calc(&chassis_motor2.v_pid);
	PID_Calc(&chassis_motor3.v_pid);
	PID_Calc(&chassis_motor4.v_pid);
	chassis_motor1.target_current=chassis_motor1.v_pid.out;
	chassis_motor2.target_current=chassis_motor2.v_pid.out;
	chassis_motor3.target_current=chassis_motor3.v_pid.out;
	chassis_motor4.target_current=chassis_motor4.v_pid.out;
}

#define MAX_D_SPEED_X       40.0f    // ��� x �����ٶȱ仯��
#define MAX_D_SPEED_Y       40.0f    // ��� y �����ٶȱ仯��
#define MAX_D_SPEED_X_STOP  60.0f   // ��� x ����ֹͣ�ٶȱ仯��
#define MAX_D_SPEED_Y_STOP  60.0f   // ��� y ����ֹͣ�ٶȱ仯��

/**
 * @brief   �����ٶ��Ż���ʹ�ٶȱ仯��ƽ��
 * @param[in]  vx_desired: ������ x �����ٶ�
 * @param[in]  vy_desired: ������ y �����ٶ�
 * @param[out] vx_optimized: �Ż���� x �����ٶ�
 * @param[out] vy_optimized: �Ż���� y �����ٶ�
 */

static void Speed_optimize(float vx_desired, float vy_desired, float *vx_optimized, float *vy_optimized)
{
    // �ٶ��Ż�
    if (vx_desired != 0.0f || vy_desired != 0.0f)
    {
        if (fabsf(vx_desired - last_vx) > MAX_D_SPEED_X)
        {
            *vx_optimized = last_vx + (vx_desired > last_vx ? MAX_D_SPEED_X : -MAX_D_SPEED_X);
        }
        else
        {
            *vx_optimized = vx_desired;
        }

        if (fabsf(vy_desired - last_vy) > MAX_D_SPEED_Y)
        {
            *vy_optimized = last_vy + (vy_desired > last_vy ? MAX_D_SPEED_Y : -MAX_D_SPEED_Y);
        }
        else
        {
            *vy_optimized = vy_desired;
        }

        last_vx = *vx_optimized;
        last_vy = *vy_optimized;
    }
    else
    {
        // ֹͣ�ٶ��Ż�
        if (last_vx > 0.0f)
        {
            last_vx -= MAX_D_SPEED_X_STOP;
            last_vx = (last_vx < 0.0f) ? 0.0f : last_vx;
        }
        else if (last_vx < 0.0f)
        {
            last_vx += MAX_D_SPEED_X_STOP;
            last_vx = (last_vx > 0.0f) ? 0.0f : last_vx;
        }

        if (last_vy > 0.0f)
        {
            last_vy -= MAX_D_SPEED_Y_STOP;
            last_vy = (last_vy < 0.0f) ? 0.0f : last_vy;
        }
        else if (last_vy < 0.0f)
        {
            last_vy += MAX_D_SPEED_Y_STOP;
            last_vy = (last_vy > 0.0f) ? 0.0f : last_vy;
        }

        *vx_optimized = last_vx;
        *vy_optimized = last_vy;
    }
}
/**
  * @breif         �����˶�����
  * @param[in]     none 
	* @param[out]    none
  * @retval        none     
  */
void chassis_move(void)
{
	//�Ż��ٶ�
	Speed_optimize(chassis_control_order.vx_set,chassis_control_order.vy_set,&chassis_control_order.vx_set,&chassis_control_order.vy_set);
	//ģʽѡ��
	chassis_move_mode();
	//pid����
	Chassis_Pid_Calc();
	//��������
	Power_limit();
	
	//���͵���
	can_send_chassis_current();
}

/**
  * @breif         ��ȡ��̨�����֮��ļн�
  * @param[in]     none
	* @param[out]    ��̨�����֮��ļн�(������)
  * @retval        none     
  */
static float Get_chassis_theta(void)
{
	float temp,temp2,angle;
	if(chassis_center.actual_angle < GIMBAL_HEAD_ANGLE)
		temp = chassis_center.actual_angle + 360.0f;
	else temp = chassis_center.actual_angle;
	temp2 = temp - GIMBAL_HEAD_ANGLE;	
	angle = temp2 / 360.0f * 2 * PI;
	return angle;
}
float theta; 

void chassis_spin(float *vx,float *vy) 
{
	theta=Get_chassis_theta(); 
	if(chassis_control_order.chassis_mode==CHASSIS_SPIN && ((int)(chassis_control_order.vx_set)!=0 || (int)(chassis_control_order.vy_set)!=0))
		theta += 0.2f;
	
	*vx = (float)(avy*sin(theta) + avx*cos(theta)); 
	*vy = (float)(avy*cos(theta) - avx*sin(theta));   
}
  

/**
  * @breif         ���̹�������
  * @param[in]     none 
	* @param[out]    ������ƺ���ĸ��������ֵ
  * @retval        none     
  */
//float current_scale,BUFFER_MAX=60.0f,POWER_TOTAL_CURxRENT_LIMIT=9000.0f;
//float power_scale,buffer_scale;
uint16_t power,max_power,buffer;

static float abs_f(float a)
 {
	if(a<0)a=-1.0f*a;
	return a;
 }

static void Power_limit(void) //�µĹ�������
{
    power     = referee_conventional.power_heat_data.chassis_power;
    buffer    = referee_conventional.power_heat_data.buffer_energy;
    max_power = referee_conventional.robot_status.chassis_power_limit;
	if(max_power == 0)
		max_power=120; //����ϵͳ��ȡ��������ʱ�Լ���ֵ

	Power_out_1 = abs_f(((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor1.target_current*20.0f/16384.0f));
	Power_out_2 = abs_f(((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor2.target_current*20.0f/16384.0f));
	Power_out_3 = abs_f(((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor3.target_current*20.0f/16384.0f));
	Power_out_4 = abs_f(((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*(K_M*chassis_motor4.target_current*20.0f/16384.0f));
	Power_out   = Power_out_1 + Power_out_2 + Power_out_3 + Power_out_4;
//	Power_out_square_sum = Power_out_1 * Power_out_1 + Power_out_2 * Power_out_2 + Power_out_3 * Power_out_3 + Power_out_4 * Power_out_4;
	Power_in_1  = Power_out_1 + POWER_K1 * (K_M*chassis_motor1.target_current*20.0f/16384.0f)*(K_M*chassis_motor1.target_current*20.0f/16384.0f)+POWER_K2*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
	Power_in_2  = Power_out_2 + POWER_K1 * (K_M*chassis_motor2.target_current*20.0f/16384.0f)*(K_M*chassis_motor2.target_current*20.0f/16384.0f)+POWER_K2*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
	Power_in_3  = Power_out_3 + POWER_K1 * (K_M*chassis_motor3.target_current*20.0f/16384.0f)*(K_M*chassis_motor3.target_current*20.0f/16384.0f)+POWER_K2*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
	Power_in_4  = Power_out_4 + POWER_K1 * (K_M*chassis_motor4.target_current*20.0f/16384.0f)*(K_M*chassis_motor4.target_current*20.0f/16384.0f)+POWER_K2*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
	Target_current_square_sum = (K_M*chassis_motor1.target_current*20.0f/16384.0f)*(K_M*chassis_motor1.target_current*20.0f/16384.0f)+(K_M*chassis_motor2.target_current*20.0f/16384.0f)*(K_M*chassis_motor2.target_current*20.0f/16384.0f)+(K_M*chassis_motor3.target_current*20.0f/16384.0f)*(K_M*chassis_motor3.target_current*20.0f/16384.0f)+(K_M*chassis_motor4.target_current*20.0f/16384.0f)*(K_M*chassis_motor4.target_current*20.0f/16384.0f);
	Actual_speed_square_sum   = ((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor1.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)+((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor2.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)+((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor3.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)+((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f)*((chassis_motor4.actual_speed/reduction_ratio)*2.0f*3.1415927f/60.0f);
	Power_in    = Power_in_1 + Power_in_2 + Power_in_3 + Power_in_4;
	
	if(max_power<50)offset = 10;
	else if(max_power>=50  && max_power<60)   offset=10;
	else if(max_power>=60  && max_power<70)   offset=10;
	else if(max_power>=70  && max_power<80)   offset=10;
	else if(max_power>=80  && max_power<100)  offset=13;
	else if(max_power>=80  && max_power<100)  offset=13;
	else if(max_power>=100 && max_power<120)  offset=15;
	else if(max_power>=120)offset=20;
	
if(Power_in>max_power && supercap_volt<=15) //?????????????????????????????
	{
		if((Power_out*Power_out-4.0f*POWER_K1*(Target_current_square_sum)*(POWER_K2*(Actual_speed_square_sum)-max_power+offset))>=0)
		{
			K_limit=(-1.0f*Power_out+sqrt(Power_out*Power_out-4.0f*POWER_K1*(Target_current_square_sum)*(POWER_K2*(Actual_speed_square_sum)-max_power+offset)))/(2.0f*POWER_K1*Target_current_square_sum);
			if(K_limit>1)K_limit=1;
		}
		else 
		{
			K_limit=0;
		}
	}
	else 
	{
		K_limit=1.0f;
	}
		chassis_motor1.target_current*=K_limit;
		chassis_motor2.target_current*=K_limit;
		chassis_motor3.target_current*=K_limit;
		chassis_motor4.target_current*=K_limit;
}
/**
  * @breif         ���̹��ʻ�����
  * @param[in]     target_power���趨��Ŀ��ֵ
	* @param[in]     target_power�����ص���ʵֵ  
	* @param[in]     last_power����һ�η��ص���ʵֵ
	* @param[out]    �ĸ�������������
  * @retval        none     
  */


//static float chassis_buffer_loop(uint16_t buffer)
//{
//	float temp;
//	b_pid.ref=50;
//	b_pid.fdb=buffer;
//	PID_Calc(&b_pid);
//	temp=1.07-((float)b_pid.out/1000.0f);
//	temp*=0.85;
//	return temp;
//}

/**
  * @breif         ���̷��º�������ֹ����º󻺳���������
  * @param[in]     buffer�����̻�������
	* @param[out]    �ĸ�������������
  * @retval        none     
  */
//static void chassis_fly(uint16_t buffer)
//{
//	if(buffer<20)
//	{
//		chassis_motor1.v_pid.out*=0.5f;
//		chassis_motor2.v_pid.out*=0.5f;
//		chassis_motor3.v_pid.out*=0.5f;
//		chassis_motor4.v_pid.out*=0.5f;
//	}
//}

/**
  * @breif         �˶�ѧ�ֽ⣬���������ĵ��ٶ�ת��Ϊ�ĸ����ӵ��ٶ�
  * @param[in]     speed_x��x�����ٶ�
	* @param[in]     speed_y��y�����ٶ�
	* @param[in]     speed_r����ת�ٶ�
	* @param[out]    �ĸ������Ŀ���ٶ�
  * @retval        none     
  */
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	int max;
		//?????????????
	BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
	
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis_motor1.target_speed=(int)(chassis_motor1.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor2.target_speed=(int)(chassis_motor2.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor3.target_speed=(int)(chassis_motor3.target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis_motor4.target_speed=(int)(chassis_motor4.target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
	chassis_motor1.v_pid.ref = chassis_motor1.target_speed;		                         
	chassis_motor2.v_pid.ref = chassis_motor2.target_speed;                             
	chassis_motor3.v_pid.ref = chassis_motor3.target_speed;                             
	chassis_motor4.v_pid.ref = chassis_motor4.target_speed;
}	

/**
  * @breif         ����ģʽ��ͨ���ǶȻ���Ŀ��Ƕ�ת��ΪĿ���ٶ�
  * @param[in]     none
  * @param[out]    ������ת�ٶ�
  * @retval        none     
  */
static float chassis_follow(void)
{
  float angle_err;
	//��̨ǹ�ڶ�Ӧ�ĽǶ�ֵ
  chassis_center.a_pid.ref = GIMBAL_HEAD_ANGLE;
  chassis_center.a_pid.fdb = chassis_center.actual_angle;

  angle_err = chassis_center.a_pid.ref - chassis_center.a_pid.fdb;
  while(angle_err > 180.0f) angle_err -= 360.0f;
  while(angle_err <- 180.0f) angle_err += 360.0f;
  chassis_center.a_pid.ref = chassis_center.a_pid.fdb + angle_err;

  chassis_center.v_pid.ref = PID_Calc(&chassis_center.a_pid);;
  return PID_Calc(&chassis_center.a_pid);
}

static STEPSTAR step_flag;
static float k_vx,k_vy,b_vx,b_vy; //�ֱ����K��B һ�κ���
static int step_times_x=0, step_times_y=0; //ʱ��
static float TIME_LIMIT=270; //б�µ�ʱ��  
static int STEP_VALUE=50; //��ֵ����step_value����б��

//б�º���״̬�ж�
void step_flag_judge(float vx,float vy,float last_vx,float last_vy)
{
	if(step_flag==NO_STEP)
	{
		if(fabs(vx - last_vx) > STEP_VALUE && fabs(vx) > 100) step_flag = X_STEP;
	    else if(fabs(vy - last_vy) > STEP_VALUE && fabs(vy) > 90) step_flag = Y_STEP;
		return;
	}
	if(step_flag == X_STEP)
	{
		if(step_times_x > TIME_LIMIT)
	    {
		    step_times_x = 0;
		    step_flag = NO_STEP;
			return;
	    }
		 if(fabs(vx) <= 1.0f) 
	     {
		    step_flag = NO_STEP;
		    step_times_x = 0;
			return;
	     }
		if(fabs(vy - last_vy) > STEP_VALUE && fabs(vy) > 90) 
		{
			step_flag = XY_STEP;
		}
		return;
	}
	
	if(step_flag==Y_STEP)
	{
		if(step_times_y>TIME_LIMIT)
	    {
		    step_times_y=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(fabs(vy)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			return;
	     }
		if(fabs(vx-last_vx)>STEP_VALUE&&fabs(vx)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==XY_STEP)
	{
		if(step_times_y>TIME_LIMIT &&step_times_x>TIME_LIMIT)
	    {
		    step_times_y=0;
        step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		if(step_times_x>TIME_LIMIT)
	    {
		    step_times_x=0;
		    step_flag=Y_STEP;
			return;
	    }
		if(step_times_y>TIME_LIMIT)
	    {
		    step_times_y=0;
		    step_flag=X_STEP;
			return;
	    }
		 if(fabs(vy)<=1.0f&&fabs(vx)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			 step_times_x=0;
			 return;
	     }
		 if(fabs(vy)<=1.0f) 
	     {
		    step_flag=X_STEP;
		    step_times_y=0;
	     }
		 if(fabs(vx)<=1.0f) 
	     {
		    step_flag=Y_STEP;
		    step_times_x=0;
	     }
		return;
	}
}

void step_star(float *vx,float *vy,float last_vx,float last_vy)
{
	step_flag_judge(*vx,*vy,last_vx,last_vy);

	if(step_flag==NO_STEP)  return;

	
	if(step_flag==X_STEP)
	{
		step_times_x++;
		if(step_times_x<=1)
		{
			k_vx=(*vx-last_vx)/TIME_LIMIT;
			b_vx=last_vx;
		}
		
		*vx=(float)(k_vx*(float)step_times_x)+b_vx;
	}
	if(step_flag==Y_STEP)
	{
		step_times_y++;
		if(step_times_y<=Y_STEP)
		{
			k_vy=(*vy-last_vy)/TIME_LIMIT;
			b_vy=last_vy;
		}
		
		*vy=k_vy*(float)step_times_y+b_vy;
	}
	if(step_flag==XY_STEP)
	{
		step_times_y++;
		if(step_times_y<=Y_STEP)
		{
			k_vy=(*vy-last_vy)/TIME_LIMIT;
			b_vy=last_vy;
		}
		step_times_x++;
		if(step_times_x<=1)
		{
			k_vx=(*vx-last_vx)/TIME_LIMIT;
			b_vx=last_vx;
		}
		
		*vx=(float)(k_vx*(float)step_times_x)+b_vx;
		
		*vy=k_vy*(float)step_times_y+b_vy;
	}
}

// �ܻ�С����
static void Shot_Spin_Judge(float* w)
{
    static uint32_t time = 0;
    float current_HP,maximum_HP;
    current_HP = referee_conventional.robot_status.current_HP;
    maximum_HP = referee_conventional.robot_status.maximum_HP;
    if(maximum_HP == 0 || current_HP == 0)  // ����ģʽ
    {
        return;
    }

    if(current_HP <= maximum_HP/2.0f)        // Ѫ������->����С����
    {
        *w = -2.0;
    }
    else
    {
        if(referee_conventional.hurt_data.armor_id != 0 && 
            (referee_conventional.hurt_data.HP_deduction_reason == 0 || referee_conventional.hurt_data.HP_deduction_reason == 5))   // �ܻ�
        {
            memset(&referee_conventional.hurt_data,0,sizeof(referee_conventional.hurt_data));
            *w = -2.0;
            time = HAL_GetTick();               // ����ʱ��
        }
        else                                // δ�ܻ�(С����->���� ���ڻ���ʱ��)
        {
            if(HAL_GetTick() - time >= 10000)
                return;
            else
                *w = -2.0;
        }
    }
    return;
}

float vx,vy,wz;
/**
  * @breif         ѡ������˶�ģʽ
  * @param[in]     none
	* @param[out]  ��������������ٶ�
  * @retval        none     
  */
static void chassis_move_mode(void)
{
	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
	
	avx=vx;
	avy=vy;
//	step_star(&avx,&avy,last_vx,last_vy);

	if(step_flag != NO_STEP)
	{
		vx = avx;
        vy = avy;
	}

	switch(chassis_control_order.chassis_mode)
	{
		case CHASSIS_NORMAL:  //��ʼ��
		break;

		case CHASSIS_NO_FORCE://���� 
			vx = vy = wz = 0;
		break;

		case CHASSIS_FOLLOW://�涯
		{
			chassis_spin(&vx,&vy);
			wz = -1.0f * chassis_follow();
		}
		break;

		case CHASSIS_SPIN://С����
		{
			chassis_spin(&vx,&vy);
			wz = -2.0;
		}
		break;

    case CHASSIS_OPPO_SPIN://����С����
		{
			chassis_spin(&vx,&vy);
			wz = 2.0;
		}
		break;

		case SENTRY_CONTROL_NORMAL:     //���涯
		{
			chassis_spin(&vx,&vy);
		}
		break;

        case SENTRY_CONTROL_FOLLOW_SPIN://��λ��(�涯+�ܻ���)
		{
			chassis_spin(&vx,&vy);
            wz = -1.0f * chassis_follow();
            Shot_Spin_Judge(&wz);
		}
		break;

        case SENTRY_CONTROL_NORMAL_SPIN:  //��λ��(���涯+�ܻ���)
		{
			chassis_spin(&vx,&vy);
            Shot_Spin_Judge(&wz);
		}
		break;
		default:break;
	}
	last_vx=(float)chassis_control_order.vx_set;
	last_vy=(float)chassis_control_order.vy_set;
	chassis_speed_control(vx,vy,wz);
}

/**
  * @breif         �����ĸ�����ĵ���
  * @param[in]     none
	* @param[out]    �ĸ�����ĵ���ֵ
  * @retval        none     
  */
static void can_send_chassis_current(void)
{
	static uint8_t cdata[8];
	if(chassis_control_order.chassis_mode == CHASSIS_NO_FORCE){
		chassis_motor1.target_current = 0;
		chassis_motor2.target_current = 0;
		chassis_motor3.target_current = 0;
		chassis_motor4.target_current = 0;
	}
	cdata[0]=(chassis_motor1.target_current)>>8;
	cdata[1]=(chassis_motor1.target_current)&0xFF;
	cdata[2]=(chassis_motor2.target_current)>>8;
	cdata[3]=(chassis_motor2.target_current)&0xFF;
	cdata[4]=(chassis_motor3.target_current)>>8;
	cdata[5]=(chassis_motor3.target_current)&0xFF;
	cdata[6]=(chassis_motor4.target_current)>>8;
	cdata[7]=(chassis_motor4.target_current)&0xFF;
	
	Can_Tx_Message(&hcan1,cdata);
}

/**
  * @breif         ���Ƶ��תһ��Ȧ��,����Ŀ�ĵغ��л�������ģʽ
  * @param[in]     none
	* @param[out]    �������
  * @retval        none     
  */
int8_t xrun_flag;
int8_t yrun_flag;

void chassis_control(void)
{
//x������5��Ϊ����
	if(xrun_flag==1)
	{
		if(chassis_motor1.total_angle<chassis_motor1.start_angle_x+8192*xrun_cnt) 					
		{
			chassis_control_order.vx_set=100;
			chassis_control_order.vy_set=0;
		}
	}
	else if(yrun_flag==1)
	{
		if(chassis_motor2.total_angle<chassis_motor2.start_angle_y+8192*yrun_cnt)
		{
			chassis_control_order.vx_set=0;
			chassis_control_order.vy_set=100;
		}
	}
}

