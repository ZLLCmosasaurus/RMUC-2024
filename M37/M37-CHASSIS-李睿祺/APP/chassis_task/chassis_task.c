#include "chassis_task.h"
#include "supercap.h"
#include "kinematics.h"
#include "referee_UI.h"
#include "referee_cmd.h"

void Referee_Send2Screen_Tx(void);

/**
  * @breif         底盘所有任务函数，实质上是定时器中断，1ms进入一次
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
void CHASSIS_TASK()
{
	static int time_count=1;
	time_count++;

	//远程遥控，调试时使用
	//remote_control();
	//chassis_control_order.chassis_mode=0;
	if(time_count%10==0)
	{
		Get_Base_Velocities();
		
		//读取裁判系统
		referee_unpack_fifo_data();
		//向云台发送数据
		canTX_gimbal1();
		//超级电容
	//	supercap();
	}
  if(time_count%6==0)
	{
		canTX_gimbal2();
	}
	if(time_count%7==0)
	{
		//底盘运动
		chassis_move();
		if(chassis_control_order.chassis_mode == CHASSIS_NO_FORCE)
			chassis_center.target_current = 0;
		canTX_gimbal_y(chassis_center.target_current); //直接发送云台传过来的电流赋值
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

	if(time_count>=1000)			//清除计数标志    1s
    time_count=1;
}
