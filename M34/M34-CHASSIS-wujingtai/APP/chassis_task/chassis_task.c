#include "chassis_task.h"
#include "supercap.h"
#include "MY_anonymous.h"
#include "referee_UI.h"
#include "buzzer.h"

/**
 * @breif         ����������������ʵ�����Ƕ�ʱ���жϣ�1ms����һ��
 * @param[in]     none
 * @param[out]    none
 * @retval        none
 */
int IMU_cnt = 0, start_flag = 0, calibrate_start_flag;

int MS_Count = 0;
int S_Count = 0;
// uint8_t offline_check_(void);
void my_offline_check(void);
void Time_Service_Task(void);
void CHASSIS_TASK()
{
	static int time_count = 0;

	if (IMU_cnt > 10)
		start_flag = 1;

	time_count++;

	// if(time_count%2==0)
	// {
	// 	buzzer_taskScheduler(&buzzer);
	// }
	// INS_task();
	if (supercap_object.last_power_mode == BATTRY)
	{
		//supercap_object.diff_cnt++;
	}
	
	
		supercap_error_FSM(&supercap_object.error_fsm, &supercap_object);
	
	my_offline_check();
	Time_Service_Task();
	if (time_count % 500 == 0) // 0.5s
	{
		send_game_time();
	}
	if (time_count % 13 == 0 && start_flag == 1)
	{
		// У׼imu
		// if (calibrate_start_flag == 1)
		// 	calibrate_task();
		// IMU_read();
	}
	if (time_count % 5 == 0)
	{
		supercap_can_tx();
	}

	if (time_count % 7 == 0)
	{
		// Զ��ң�أ�����ʱʹ��
		//		remote_control();
		//		//�����˶�
		chassis_move();
		canTX_gimbal_y(chassis_center.target_current); // ֱ�ӷ�����̨�������ĵ�����ֵ
	}
	// chassis_control_order.chassis_mode=0;
	if (time_count % 10 == 0)
	{
		Get_Base_Velocities();
		send_gimbal_data_2();
		canTX_chassis_imu();
		// ��ȡ����ϵͳ
		referee_unpack_fifo_data();
		// ����̨��������
		send_gimbal_data();
		// ����
	}
	if (time_count % 100 == 0)
	{
#if defined(Game_On)

		{

			if (supercap_object.diff_flag == 1)
			{
				supercap_object.diff_cnt = 0;
			}
			if (supercap_object.error_status == CAP_OK || supercap_object.diff_flag == 1)
			{
				supercap_object.diff_flag = 0;
				set_power_mode(&supercap_object, supercap_object.change_flag);
			}
		}
#endif
	}
	if (time_count >= 1000) // ���������־    1s
	{
		time_count = 1;
		// ��������
		//		supercap();
		if (start_flag == 0)
			IMU_cnt++;
	}
	// ��������
	// supercap(S_Count, MS_Count);
}
void Time_Service_Task(void)
{
	MS_Count++;
	if (MS_Count > 999)
	{
		MS_Count = 0;
		S_Count++;
	}
}

void my_offline_check(void)
{ // �ϰ����߼��
	if (offline_check(&chassis_control_order.offline_check, 100) == OFFLINE)
	{
		chassis_control_order.chassis_mode = CHASSIS_NO_FORCE;
		// buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
	}
	// ������߼��
	if (offline_check(&chassis_motor1.offline_check, 100) == OFFLINE || offline_check(&chassis_motor2.offline_check, 100) == OFFLINE || offline_check(&chassis_motor3.offline_check, 100) == OFFLINE || offline_check(&chassis_motor4.offline_check, 100) == OFFLINE || offline_check(&chassis_center.offline_check, 100) == OFFLINE)
	{
		// buzzer_setTask(&buzzer, BUZZER_DEVICE_OFFLINE_PRIORITY);
	}
	// �������߼��
	if (offline_check(&supercap_object.offline_check, 200) == OFFLINE)
	{
		// set_power_mode(&supercap_object, BATTRY);
	}
	else
	{
		// set_power_mode(&supercap_object, SUPER_CAP);
	}
}

// uint8_t offline_check_(void)
// {

// 	if (chassis_control_order.can2_online_cnt == chassis_control_order.can2_last_online_cnt)
// 	{
// 		chassis_control_order.can2_offline_cnt++;
// 	}
// 	else
// 	{
// 		chassis_control_order.can2_offline_cnt = 0;
// 	}
// 	if (chassis_control_order.can2_offline_cnt > 100)
// 	{
// 		chassis_control_order.can2_online_cnt = 0;
// 		chassis_control_order.can2_last_online_cnt = 0;
// 		chassis_control_order.chassis_mode = CHASSIS_NO_FORCE;

// 		return 1;
// 	}

// 	chassis_control_order.can2_last_online_cnt = chassis_control_order.can2_online_cnt;
// 	return 0;
// }
