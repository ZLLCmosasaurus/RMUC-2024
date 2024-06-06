#include "referee.h"


frame_header_struct_t referee_receive_header;

ext_dart_remaining_time_t dart_remaining_time_t;//飞镖发射剩余时间
ext_dart_client_cmd_t dart_client_cmd_t;//飞镖发射站状态，及相应比赛剩余时间


void init_referee_struct_data(void)
{
    memset(&dart_remaining_time_t,0,sizeof(ext_dart_remaining_time_t));
    memset(&dart_client_cmd_t,0,sizeof(ext_dart_client_cmd_t));
    dart_client_cmd_t.dart_launch_opening_status = Dart_State_Close;     //飞镖闸门关闭
}





