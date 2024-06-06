#include "chassis_task.h"
#include "supercap.h"
#include "kinematics.h"
#include "referee_UI.h"
#include "referee_cmd.h"

void Referee_Send2Screen_Tx(void);

/**
  * @breif         ����������������ʵ�����Ƕ�ʱ���жϣ�1ms����һ��
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
void CHASSIS_TASK()
{
	static int time_count=1;
	time_count++;

	//Զ��ң�أ�����ʱʹ��
	//remote_control();
	//chassis_control_order.chassis_mode=0;
	if(time_count%10==0)
	{
		Get_Base_Velocities();
		
		//��ȡ����ϵͳ
		referee_unpack_fifo_data();
		//����̨��������
		canTX_gimbal1();
		//��������
	//	supercap();
	}
  if(time_count%6==0)
	{
		canTX_gimbal2();
	}
	if(time_count%7==0)
	{
		//�����˶�
		chassis_move();
		if(chassis_control_order.chassis_mode == CHASSIS_NO_FORCE)
			chassis_center.target_current = 0;
		canTX_gimbal_y(chassis_center.target_current); //ֱ�ӷ�����̨�������ĵ�����ֵ
	}
  if(time_count%8==0)
	{
		canTX_gimbal3();
	}

    if(time_count%8==7)
	{
		canTX_gimbal4();
	}

//	 if(time_count%500==0)
//	 {
//      Referee_Send2Screen_Tx();
//	 }
	
	if(time_count%40==1)
	{
     Referee_Send2Radar_Tx();
	}

	if(time_count>=1000)			//���������־    1s
    time_count=1;
}
