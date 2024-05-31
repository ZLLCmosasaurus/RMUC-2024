#include "referee_cmd.h"
static robot_interaction_data_t robot_interaction_data;
static uint8_t seq = 0;

static void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i=i;
	
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	append_CRC8_check_sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_CRC16_check_sum(tx_buff,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
    else seq++;
        
	/*****数据上传*****/
	__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_TC);
	HAL_UART_Transmit(&huart6, tx_buff,frame_length , 100);
	while (__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC) == RESET); //等待之前的字符发送完成
}

// 无敌状态检测(否：0，是：1)
void Detect_invincibility_status(uint8_t* _status)
{
	static Robot_Status_t item[6];
	if(referee_conventional.robot_status.maximum_HP == 0)
		return;

	if(referee_conventional.robot_status.robot_id == RED_SENTRY)
	{
		item[0].HP = referee_conventional.game_robot_HP.blue_1_robot_HP;
		item[1].HP = referee_conventional.game_robot_HP.blue_2_robot_HP;
		item[2].HP = referee_conventional.game_robot_HP.blue_3_robot_HP;
		item[3].HP = referee_conventional.game_robot_HP.blue_4_robot_HP;
		item[4].HP = referee_conventional.game_robot_HP.blue_5_robot_HP;
		item[5].HP = referee_conventional.game_robot_HP.blue_7_robot_HP;
	}
	else if(referee_conventional.robot_status.robot_id == BLUE_SENTRY)
	{
		item[0].HP = referee_conventional.game_robot_HP.red_1_robot_HP;
		item[1].HP = referee_conventional.game_robot_HP.red_2_robot_HP;
		item[2].HP = referee_conventional.game_robot_HP.red_3_robot_HP;
		item[3].HP = referee_conventional.game_robot_HP.red_4_robot_HP;
		item[4].HP = referee_conventional.game_robot_HP.red_5_robot_HP;
		item[5].HP = referee_conventional.game_robot_HP.red_7_robot_HP;
	}
	for(uint8_t i = 0; i<6; i++)
	{
		if(item[i].Invincible_State == 1 && HAL_GetTick() - item[i].Invincible_Time >= 4000)
		{
			item[i].Invincible_State = 0;
		}

		if(item[i].p_HP == 0 && item[i].HP > 0)
		{
			item[i].Invincible_State = 1;
			item[i].Invincible_Time = HAL_GetTick();
		}

		item[i].p_HP = item[i].HP;
		*_status |= item[i].Invincible_State << i;
	}
    if((referee_conventional.robot_status.robot_id == RED_SENTRY && referee_conventional.game_robot_HP.blue_outpost_HP != 0)
        || (referee_conventional.robot_status.robot_id == BLUE_SENTRY && referee_conventional.game_robot_HP.red_outpost_HP != 0))
    {
        *_status |= 0x01 << 5;
    }
}

// 结构体初始化
static void Data2Radar_Init(void)
{
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data));
}

static void Radar_Cmd_F(uint8_t* radar_cmd)
{
    float x,y;
    x = map_interaction.map_command.target_position_x;
    y = map_interaction.map_command.target_position_y;
    if(y >= 5.8 && y <= 9.2)
    {
        if((x>=0 && x<=3.5) || (x>=24.5 && x<=28))
            *radar_cmd = 1;
        else
            *radar_cmd = 0;
    }
    else
        *radar_cmd = 0;
    if(*radar_cmd == 1)
        memset(&map_interaction.map_command, 0, sizeof(map_interaction.map_command));
}   

// 发送信息主函数(哨兵->雷达)
void Referee_Send2Radar_Tx(void)
{
  Data2Radar_Init();
  static uint8_t radar_cmd = 0;
  robot_interaction_data.data_cmd_id = RADAR_TX_ID;
  if(referee_conventional.robot_status.robot_id == RED_SENTRY)
  {
    robot_interaction_data.sender_id = RED_SENTRY;
    robot_interaction_data.receiver_id = RED_RADAR;
  }
  else if(referee_conventional.robot_status.robot_id == BLUE_SENTRY)
  {
    robot_interaction_data.sender_id = BLUE_SENTRY;
    robot_interaction_data.receiver_id = BLUE_RADAR;
  }
  Radar_Cmd_F(&radar_cmd);
  memcpy(&robot_interaction_data.user_data, &radar_cmd, sizeof(radar_cmd));
  referee_data_pack_handle(SOF, ROBOT_INTER_ID, (uint8_t*)&robot_interaction_data, sizeof(robot_interaction_data));
}

void Referee_Send2Screen_Tx(void)
{
    memset(&referee_conventional.custom_info, 0, sizeof(referee_conventional.custom_info));
    if(referee_conventional.robot_status.robot_id == RED_SENTRY)
    {
    referee_conventional.custom_info.sender_id = RED_SENTRY;
    referee_conventional.custom_info.receiver_id = 0x0106;
    }
    else if(referee_conventional.robot_status.robot_id == BLUE_SENTRY)
    {
    referee_conventional.custom_info.sender_id = BLUE_SENTRY;
    referee_conventional.custom_info.receiver_id = 0x016A;
    }
    memcpy(referee_conventional.custom_info.user_data,"Radar Ready",sizeof("Radar Ready"));
}