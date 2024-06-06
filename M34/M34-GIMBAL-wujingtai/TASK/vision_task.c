#include "vision_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "chassis_task.h"
#include <math.h>
#include "USB_Commucation.h"
#include "usbd_cdc_if.h"
#include "RC_task.h"
#include "sent_task.h"

VISION_t vision_mode = VISION_OFF;
VISION_t assist_vision_mode = ASSIST_VISION_OFF;

VISION_GET_t vision_sent;
UPPER_COMPUTER_VISION_t shoot_vision_mode = VISION_NOMAL; // ��λ������ģʽ

float chassis_angle = 0;				// �����ٶ��������̨�ο�ϵ�ļн�
float vision_x = 0, vision_y = 0;		// �����ٶ�����̨����ϵ�µ��ٶ�
float vision_yaw = 0, vision_pitch = 0; // ����λ��
int vision_cnt = 0;
extern uint8_t traget_exit_flag;
extern uint8_t auto_exposure_flag;
extern uint8_t reboot_flag;

extern Pack_tx_t pack;
extern Pack_rx_t pack_rx;
int vision_delay(int vt)
{
	vision_cnt++;
	if (vision_cnt > vt)
	{
		vision_cnt = 0;
		return 1;
	}
	else
		return 0;
}
uint8_t vision_check_flag = 0; // ���ڷ�ֹ�������Ȩ�л�ʱ��������
float last_yaw_target_angle = 0, last_pitch_target_angle = 0;
float yaw_deadline_angle = 0.01f, pitch_deadline_angle = 0.01; // ����

void Vision_Task(void)
{
	auto_aim_fsm_task(&vision_sent.fsm, &vision_sent);

	if (vision_mode == VISION_OFF && assist_vision_mode == ASSIST_VISION_OFF) // ����ģʽ��
	{
		
		if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
		{
			gimbal_y.target_speed = ((rc_sent.yaw.target_angle) / 5);
		}
		if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
		{
			gimbal_y.add_angle = (8192.0f / 360.0f) * (rc_sent.yaw.target_angle);
		}
		if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
		{
			gimbal_y.add_angle = rc_sent.yaw.target_angle;
		}

		//		gimbal_y.add_angle=rc_sent.yaw.target_angle;
		gimbal_p.add_angle = rc_sent.pitch.target_angle;
		//		gimbal_y.target_speed=2*(rc_sent.yaw.target_angle);
		//		gimbal_y.target_speed=((rc_sent.yaw.target_angle)/4.5);
		//		gimbal_y.target_angle=Ren;
	}

	else if (vision_mode == VISION_ON || assist_vision_mode == ASSIST_VISION_ON) // ����ģʽ��
	{

		if (vision_sent.Control_priority == 1) // �Ӿ������ٿ�Ȩ����λ��
		{
			vision_sent.Control_priority = 0;
			// pack_rx.UP_flag=0;
			vision_check_flag = 1;
			// yaw
			// if (gimbal_y.auto_aim_angle - last_yaw_target_angle > yaw_deadline_angle || gimbal_y.auto_aim_angle - last_yaw_target_angle < -yaw_deadline_angle) // ����
			{
				// gimbal_y.add_angle=vision_sent.yaw.target_angle;
				// last_yaw_target_angle=vision_sent.yaw.target_angle;
				// gimbal_y.add_angle=(vision_sent.yaw.target_angle-imu_can_error_y);
				// last_yaw_target_angle=(vision_sent.yaw.target_angle-imu_can_error_y);
				// if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
				// {
				// 	gimbal_y.target_angle = (8192.0f / 360.0f) * (gimbal_y.auto_aim_angle - imu_can_error_y);
				// 	last_yaw_target_angle = (8192.0f / 360.0f) * (gimbal_y.auto_aim_angle - imu_can_error_y);
				// }
				// if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
				{
					gimbal_y.target_angle = gimbal_y.auto_aim_angle;
					last_yaw_target_angle = gimbal_y.auto_aim_angle;
				}
			}
			// pitch
			// if (gimbal_p.auto_aim_angle - last_pitch_target_angle > pitch_deadline_angle || gimbal_p.auto_aim_angle - last_pitch_target_angle < -pitch_deadline_angle)
			{
				gimbal_p.target_angle = gimbal_p.auto_aim_angle;
				last_pitch_target_angle = gimbal_p.auto_aim_angle;
			}
		}
		else // �Ӿ��������ǲٿ�Ȩ����λ��
		{
			vision_check_flag = 0;
			if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
			{
				//				gimbal_y.target_speed=((rc_sent.yaw.target_angle)/5);
			}
			if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
			{
				gimbal_y.add_angle = (8192.0f / 360.0f) * (rc_sent.yaw.target_angle);
			}
			if (gimbal_y.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
			{
				gimbal_y.add_angle = rc_sent.yaw.target_angle;
			}
			gimbal_p.add_angle = rc_sent.pitch.target_angle; // ǿ��Ϊimu����ģʽ
		}
	}
}
float vision_actual_angle_cal()
{
	chassis_angle = gimbal_y.CAN_actual_angle - 343;
	if (chassis_angle < 0)
		chassis_angle += 360;
	return 0;
}

// ����״̬ 1.��ͨ���� 2.��糵 3.С�糵
void auto_aim_fsm_task(Struct_FSM *fsm, VISION_GET_t *ptr)
{
	const uint16_t time = 200;//60*3+20
	fsm->Status[fsm->Now_Status_Serial].Time++;
	switch (Get_Now_Status_Serial(fsm))
	{
	// ��ͨ����
	case 0:
		ptr->status = NORMAL_AIM;
		pack.point_num = 4;
		if(ptr->windwill_press_flag==1)
		{
			ptr->windwill_press_flag=0;
			if(ptr->remain_time>time)
			{
				// ��ͨ����->С�糵
				Set_FSM_Status(fsm, 1);
			}
			else
			{
				// ��ͨ����->��糵
				Set_FSM_Status(fsm, 2);
			}
		}
		if (control_mode == KEY_OFF && switch_is_down(rc_ctrl.rc.s[1])) // ����
		{
			if (ptr->last_s1 != RC_SW_DOWN) // ����
			{
				Set_FSM_Status(fsm, 1);
			}
		}
		ptr->last_status = NORMAL_AIM;
		break;
	// С�糵 ����200s
	case 1:
		ptr->status = SMALL_WINDWILL_AIM;
		pack.point_num = 5;
		pack.is_large_buff = 0;
		if(ptr->amor_press_flag==1)
		{
			ptr->amor_press_flag=0;
			// С�糵->��ͨ����
			Set_FSM_Status(fsm, 0);
		}
		if (control_mode == KEY_OFF && switch_is_down(rc_ctrl.rc.s[1]))
		{
			if (ptr->last_s1 != RC_SW_DOWN)
			{
			Set_FSM_Status(fsm, 0);
			}
		}
		ptr->last_status = SMALL_WINDWILL_AIM;
		break;
	// ��糵 С��200s
	case 2:
		ptr->status = BIG_WINDWILL_AIM;
		pack.is_large_buff = 1;
		pack.point_num = 5;
		if(ptr->amor_press_flag==1)
		{
			ptr->amor_press_flag=0;
			// ��糵->��ͨ����
			Set_FSM_Status(fsm, 0);
		}
		ptr->last_status = BIG_WINDWILL_AIM;
		break;
	}
	ptr->last_s1 = rc_ctrl.rc.s[1];

	can_tx_0x114_data[0] = ptr->status;
}

// double vision_actual_speed,vision_actual_angle;
// void vision_getSpeed()
//{
//	vision_actual_speed=(double)(chassis_speed_x*chassis_speed_x+chassis_speed_y*chassis_speed_y);
//	vision_actual_speed=sqrt(vision_actual_speed);
//	vision_x=chassis_speed_x*acos(chassis_angle)-chassis_speed_y*asin(chassis_angle);
//	vision_y=chassis_speed_x*asin(chassis_angle)+chassis_speed_y*acos(chassis_angle);
//	vision_actual_angle_cal();
//	vision_actual_angle=chassis_angle;
// }
