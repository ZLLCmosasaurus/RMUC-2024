#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
	RED_HERO        = 1,
	RED_ENGINEER    = 2,
	RED_STANDARD_1  = 3,
	RED_STANDARD_2  = 4,
	RED_STANDARD_3  = 5,
	RED_AERIAL      = 6,
	RED_SENTRY      = 7,
	RED_DART        = 8,
	RED_RADAR       = 9,
	RED_OUTPOST     = 10,
	RED_BASE        = 11,
	BLUE_HERO       = 101,
	BLUE_ENGINEER   = 102,
	BLUE_STANDARD_1 = 103,
	BLUE_STANDARD_2 = 104,
	BLUE_STANDARD_3 = 105,
	BLUE_AERIAL     = 106,
	BLUE_SENTRY     = 107,
	BLUE_DART       = 108,
	BLUE_RADAR      = 109,
	BLUE_OUTPOST    = 110,
	BLUE_BASE       = 111,
} robot_id_t;

typedef enum
{
	PROGRESS_UNSTART        = 0,
	PROGRESS_PREPARE        = 1,
	PROGRESS_SELFCHECK      = 2,
	PROGRESS_5sCOUNTDOWN    = 3,
	PROGRESS_BATTLE         = 4,
	PROGRESS_CALCULATING    = 5,
} game_progress_t;

/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct // 0x0001 比赛状态数据
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct // 0x0002 比赛结果数据
{
	uint8_t winner;
}game_result_t;

typedef __packed struct // 0x0003 机器人血量数据
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct // 0x0101 场地事件数据
{
	uint32_t event_type;
}event_data_t;

typedef __packed struct // 0x0102 补给站动作标识
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
}ext_supply_projectile_action_t;

typedef __packed struct // 0x0104 裁判警告信息
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
}referee_warning_t;

typedef __packed struct // 0x0105 飞镖发射口倒计时
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
}dart_info_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct // 0x0201 比赛机器人状态
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef __packed struct // 0x0202 实时功率热量数据
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float 	 chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef __packed struct // 0x0203 位置与朝向
{
	float x;
	float y;
	float angle;
}robot_pos_t;

typedef __packed struct	// 0x204 机器人增益
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
}buff_t;

typedef __packed struct // 0x0205 空中机器人能量状态
{
	uint8_t airforce_status;
	uint8_t time_remain;
}air_support_data_t;

typedef __packed struct // 0x0206 伤害状态
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
}hurt_data_t;

typedef __packed struct // 0x0207 实时射击信息
{
	uint8_t bullet_type;        //子弹类型
	uint8_t shooter_number;     //发射机构ID
	uint8_t launching_frequency;//射频
	float initial_speed;        //弹速
}shoot_data_t;

typedef __packed struct // 0x0208 子弹剩余发射数
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef __packed struct // 0x0209 机器人RFID状态
{
	uint32_t rfid_status;
}rfid_status_t;

typedef __packed struct // 0x020A 飞镖机器人客户端指令数据
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

typedef __packed struct	// 0x020B 己方机器人位置
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
}ground_robot_position_t;

typedef __packed struct // 0x020C 对方机器人被标记进度
{
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
}radar_mark_data_t;

typedef __packed struct	// 0x020D 哨兵自主决策信息同步
{
	uint32_t sentry_info;
} sentry_info_t;

typedef __packed struct // 0x020E 雷达双倍易伤状态
{
 uint8_t radar_info;
} radar_info_t;

/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct // 0x0301 机器人交互数据
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[1];
}robot_interaction_data_t;

/* 0x010X (子内容) ------------------------------------------------------------*/
typedef __packed struct // 0x0100 选手端删除图层
{
	uint8_t delete_type;
	uint8_t layer;
}interaction_layer_delete_t;

typedef __packed struct // 0x0101 选手端绘制一个图形
{ 
	uint8_t figure_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t figure_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t details_a:9;
	uint32_t details_b:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t details_c:10; 
	uint32_t details_d:11; 
	uint32_t details_e:11; 
}interaction_figure_t;

typedef __packed struct // 0x0102 选手端绘制两个图形
{
  interaction_figure_t interaction_figure[2];
}interaction_figure_2_t;

typedef __packed struct // 0x0103 选手端绘制五个图形
{
	interaction_figure_t interaction_figure[5];
}interaction_figure_3_t;

typedef __packed struct // 0x0104 选手端绘制七个图形
{
	interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

/* 0x011X (子内容) ------------------------------------------------------------*/
typedef __packed struct // 字符图形子结构体 <0>
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    interaction_figure_t interaction_figure;
}graphic_data_struct_t;

typedef __packed struct // 0x0110 选手端绘制字符图形
{
	graphic_data_struct_t graphic_data_struct;     // <0>
	uint8_t data[30];
}ext_client_custom_character_t;

/* 0x012X (子内容) ------------------------------------------------------------*/
typedef __packed struct // 0x0120 哨兵自主决策指令
{
	uint32_t sentry_cmd; 
}sentry_cmd_t;

typedef __packed struct // 0x0121 雷达自主决策指令
{
	uint8_t radar_cmd;
}radar_cmd_t;

/* 0x030X 小地图交互数据 -------------------------------------------------------*/
typedef __packed struct // 0x0303 小地图下发信息标识
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
}map_command_t;

typedef __packed struct // 0x0305 小地图接收信息标识
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
}map_robot_data_t;

typedef __packed struct // 0x0307 路径坐标数据发送
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
}map_data_t;

typedef __packed struct // 0x0308 自定义消息发送
{ 
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
}custom_info_t;

/* 0x030X 图传链路数据 ---------------------------------------------------------*/
typedef __packed struct // 0x0302 自定义控制器与机器人交互
{
	uint8_t data[30];
}custom_robot_data_t;

typedef __packed struct // 0x0304 键鼠遥控数据
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}remote_control_t;

/* 0x030X 非链路数据 -----------------------------------------------------------*/
typedef __packed struct // 0x0306 自定义控制器模拟键鼠
{
	uint16_t key_value;
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4;
	uint16_t reserved;
}custom_client_data_t;

/* 裁判系统 总结构体 */
typedef __packed struct				// 发送结构体
{
	interaction_layer_delete_t 	    interaction_layer_delete;
    graphic_data_struct_t           graphic_data_struct;
	ext_client_custom_character_t 	ext_client_custom_character;
	sentry_cmd_t 					sentry_cmd;
	radar_cmd_t 					radar_cmd;
}robot_interaction_sub_t;

typedef __packed struct				// 常规链路结构体
{
	game_status_t 					game_status;
	game_result_t 					game_result;
	game_robot_HP_t 				game_robot_HP;
	event_data_t                    event_data;
	ext_supply_projectile_action_t  ext_supply_projectile_action;
	referee_warning_t 				referee_warning;
	dart_info_t 					dart_info;
	robot_status_t 					robot_status;
	power_heat_data_t 				power_heat_data;
	robot_pos_t 					robot_pos;
	buff_t 							buff;
	air_support_data_t 		        air_support_data;
	hurt_data_t 					hurt_data;
	shoot_data_t 					shoot_data;
	projectile_allowance_t 			projectile_allowance;
	rfid_status_t 					rfid_status;
	dart_client_cmd_t 				dart_client_cmd;
	ground_robot_position_t 		ground_robot_position;
	radar_mark_data_t 				radar_mark_data;
	sentry_info_t 					sentry_info;
	radar_info_t 					radar_info;
    custom_info_t                   custom_info;
	robot_interaction_data_t 		robot_interaction_data;
}referee_conventional_t;

typedef __packed struct				// 小地图交互结构体
{
	map_command_t 					map_command;
	map_robot_data_t 				map_robot_data;
	map_data_t 						map_data;
	custom_info_t 					custom_info;
}map_interaction_t;

typedef __packed struct				// 图传链路数据结构体
{
	custom_robot_data_t 			custom_robot_data;
	remote_control_t 				remote_control;
}image_transmission_t;

extern referee_conventional_t     referee_conventional; // 常规链路
extern map_interaction_t          map_interaction;      // 小地图交互
extern image_transmission_t       image_transmission;   // 图传链路
extern custom_client_data_t       custom_client_data;   // 非链路
extern image_transmission_t       image_transmission;

extern void referee_unpack_fifo_data(void);
extern void referee_data_init(void);
#endif
