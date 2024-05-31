#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
#include "protocal.h"
#include <string.h>

/* 飞镖信息 */
#define Dart_State_Open     0 //飞镖闸门开启
#define Dart_State_Close    1 //飞镖闸门关闭
#define Dart_State_Changing 2 //正在开启或者关闭中
#define Dart_Target_Outpost 0 //飞镖目标为前哨站
#define Dart_Target_Base    1 //飞镖目标为基地

typedef __packed struct  //0x0105 飞镖发射口倒计时
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} ext_dart_remaining_time_t;



typedef __packed struct  //0x020A 飞镖机器人客户端指令数据
{
    uint8_t  dart_launch_opening_status;//当前飞镖发射站的状态：1：关闭\\\ 2：正在开启或者关闭中 \\\0：已经开启
    uint8_t  reserved;
    uint16_t target_change_time;//切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
} ext_dart_client_cmd_t;

/*飞镖信息结构体-------------------------------------------------------------------------------*/
typedef struct 
{
    /* data */
    uint8_t dart_shoot_remaining_time;//飞镖发射口倒计时
    uint8_t dart_launch_open_status ;//当前飞镖发射站的状态：1：关闭\\\ 2：正在开启或者关闭中 \\\0：已经开启
    uint16_t dart_change_time;//切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t dart_latest_launch_cmd_time;//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
}DART_RX_Information;

extern void init_referee_struct_data(void);

extern frame_header_struct_t referee_receive_header;
extern ext_dart_remaining_time_t dart_remaining_time_t;//飞镖发射剩余时间
extern ext_dart_client_cmd_t dart_client_cmd_t;//飞镖发射站状态，及相应比赛剩余时间
#endif

