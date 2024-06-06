#include "bsp_can.h"
#include "referee.h"
#include "chassis_move.h"

static int16_t* read_motor_data(uint8_t *rxdata);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);

/**
  * @breif         can通信初始化
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;

	canFilter.FilterBank=1;    																//筛选器组1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;												//can2筛选组起始编号
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//配置过滤器
	
	canFilter.FilterBank=15;    															//筛选器组15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//配置过滤器
	
	/*离开初始模式*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*开中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 接收fifo 0不为空中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 接收fifo 0不为空中断
	return status;
}

/**
  * @breif         can发送函数
  * @param[in]     hcan：can的句柄结构体
	* @param[in]     mdata：需要发送的数组
	* @param[out]    none
  * @retval        status：发送的状态    
  */
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
{
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	if(hcan==&hcan1)
	{
		CAN_TxHeaderStruct.StdId=0x200;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	else if(hcan==&hcan2)
	{
		CAN_TxHeaderStruct.StdId=0x006;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
	return status;
}

uint8_t canTX_gimbal_y(int16_t yaw)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[6]=yaw>>8;
	data[7]=yaw&0xff;
	data[4]=0;
	data[5]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);
	
	return temp;
}

/**
  * @breif         can接收中断函数
  * @param[in]     hcan：can的句柄结构体
	* @param[out]    can1接收四个电机返回的电流值，can2接收上板传来的数据
  * @retval        none   
  */

int YAW_SPEED;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
		switch(CAN_RxHeaderStruct.StdId)
		{
			case CAN_3508Motor1_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			break;
			case CAN_3508Motor2_ID:
				get_motor_data(&chassis_motor2,angle,speed,current);
			break;
			case CAN_3508Motor3_ID:
				get_motor_data(&chassis_motor3,angle,speed,current);
			break;
			case CAN_3508Motor4_ID:
				get_motor_data(&chassis_motor4,angle,speed,current);
			break;
			case CAN_GIMBAL_Y_ID:
				record_chassis_callback(&chassis_center,angle,current);
				YAW_SPEED=gdata[1];
			break;
			case LOOP_BACK_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			default:break;
		}
	}
	else if(hcan==&hcan2)
	{
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
			chassis_control_order.chassis_mode=CHASSIS_NO_FORCE;
		gdata = read_motor_data(rxdata);
		if(CAN_RxHeaderStruct.StdId == GIMBAL_R_ID_1)
		{
            chassis_control_order.vx_set  = -gdata[1];
            chassis_control_order.vy_set  = -gdata[0]; 
            chassis_control_order.wz_set  = gdata[2]/100;
            chassis_center.target_current = gdata[3];
		}
		if(CAN_RxHeaderStruct.StdId == GIMBAL_R_ID_2)	
		{
			chassis_control_order.chassis_mode = rxdata[0];//mode
			chassis_control_order.last_chassis_mode = chassis_control_order.chassis_mode; 
		}
	}
}

/**
  * @breif         can整合电机数据函数
  * @param[in]     rxdata：电机返回的数组
	* @param[out]    整合好的数组
  * @retval        adata：整合好的数组  
  */
static int16_t adata[4];
static int16_t* read_motor_data(uint8_t *rxdata)
{
	adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
	adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
	adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
	adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	return adata;
}

/**
  * @breif         can解析电机数据函数
  * @param[in]     motor：电机参数结构体
  * @param[in]     angle：电机角度值
  * @param[in]     speed：电机速度值
  * @param[in]     current：电机电流值
	* @param[out]    将输入的值赋值给电机的结构体成员
  * @retval        none   
  */
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;

	motor->actual_angle = angle;
	motor->a_pid.fdb = motor->actual_angle;

	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->v_pid.fdb = motor->actual_speed;

	motor->last_speed = speed;
	motor->actual_current = current;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt--;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;// - motor->start_angle;
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/8191.0f;
	motor->actual_angle*= 360.0f;
	motor->actual_current = current;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = (float)angle/8191.0f*360.0f;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
}

//------------3.21----------------//

/**
  * @breif         can发送上板函数
  * @param[in]     can_tx, can_tx_id
	* @param[out]    none
  * @retval        none   
  */

static can_tx1_t can_tx1;
static can_tx2_t can_tx2;
void Can_Data_Solve(void);

void canTX_gimbal1(void)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=GIMBAL_T_ID_1;       //0x008
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
    Can_Data_Solve();
    memcpy(data, &can_tx1, sizeof(can_tx1));
	
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_gimbal2(void)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=GIMBAL_T_ID_2;       //0x010
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
    
    Can_Data_Solve();
    memcpy(data, &can_tx2, sizeof(can_tx2));

	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_gimbal3(void)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=0x012;               //0x012
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
    data[0] = referee_conventional.robot_status.shooter_barrel_cooling_value >> 8;
    data[1] = referee_conventional.robot_status.shooter_barrel_cooling_value & 0xFF;
    data[2] = referee_conventional.robot_status.shooter_barrel_heat_limit >> 8;
    data[3] = referee_conventional.robot_status.shooter_barrel_heat_limit & 0xFF;
    data[4] = referee_conventional.power_heat_data.shooter_17mm_1_barrel_heat >> 8;
    data[5] = referee_conventional.power_heat_data.shooter_17mm_1_barrel_heat & 0xFF;
    data[6] = referee_conventional.power_heat_data.shooter_17mm_2_barrel_heat >> 8;
    data[7] = referee_conventional.power_heat_data.shooter_17mm_2_barrel_heat & 0xFF;

	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}
void canTX_gimbal4(void)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=0x013;               //0x012
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
    memcpy(data,&referee_conventional.shoot_data,sizeof(referee_conventional.shoot_data));

	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}
void Detect_invincibility_status(bool* _status);
void Can_Data_Solve(void)
{
    bool color;
    if(referee_conventional.robot_status.robot_id == RED_SENTRY)
        color = 0;
    else if(referee_conventional.robot_status.robot_id == BLUE_SENTRY)
        color = 1;
    
    memset(&can_tx1,0,sizeof(can_tx1));
    memset(&can_tx2,0,sizeof(can_tx2));

    can_tx1.game_process        = referee_conventional.game_status.game_progress;
    can_tx1.time                = referee_conventional.game_status.stage_remain_time;
    can_tx1.self_blood          = referee_conventional.robot_status.current_HP;
    can_tx1.armor_id            = referee_conventional.hurt_data.armor_id;
    can_tx1.HP_deduction_reason = referee_conventional.hurt_data.HP_deduction_reason;

    if(color)
        can_tx1.self_outpost_HP      = referee_conventional.game_robot_HP.blue_outpost_HP;
    else
        can_tx1.self_outpost_HP      = referee_conventional.game_robot_HP.red_outpost_HP;

    can_tx2.projectile_allowance_17mm = referee_conventional.projectile_allowance.projectile_allowance_17mm;
    can_tx2.color               = color;
    can_tx2.outpost_rfid        = referee_conventional.rfid_status.rfid_status & 0x00010000 >> 12;

    if(color)
    {
        can_tx2.oppo_outpost_HP    = referee_conventional.game_robot_HP.red_outpost_HP;
        can_tx2.self_base_HP = referee_conventional.game_robot_HP.blue_base_HP;
    }
    else
    {
        can_tx2.oppo_outpost_HP    = referee_conventional.game_robot_HP.blue_outpost_HP;
        can_tx2.self_base_HP = referee_conventional.game_robot_HP.red_base_HP;
    }

	Detect_invincibility_status(&can_tx2.invincible_state);
}

