#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "bsp_referee.h"

referee_conventional_t  referee_conventional; // 常规链路
map_interaction_t       map_interaction;      // 小地图交互
custom_client_data_t    custom_client_data;   // 非链路
image_transmission_t    image_transmission;   // 图传链路

static unpack_data_t    referee_unpack_obj;
static frame_header_struct_t referee_receive_header;
static frame_header_struct_t referee_send_header;

extern void referee_data_solve(uint8_t *frame);

void Referee_Send2Screen_Tx(void);

void referee_data_init()
{
	memset(&referee_conventional,   0, sizeof(referee_conventional_t));
    memset(&map_interaction,        0, sizeof(map_interaction_t));
    memset(&custom_client_data,     0, sizeof(custom_client_data_t));
    memset(&image_transmission,     0, sizeof(image_transmission));
    memset(&referee_unpack_obj,     0, sizeof(unpack_data_t));
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header,    0, sizeof(frame_header_struct_t));
}

void referee_unpack_fifo_data(void)
{
  int temp;
  char byte_global = 0;
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;

  unpack_data_t *p_obj = &referee_unpack_obj;
  temp = fifo_s_used(&referee_fifo);

  while (fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    byte_global = byte;
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
}

void referee_data_solve(uint8_t *frame)
{
  uint16_t cmd_id = 0;
  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id)
  {
    case 0x0001:
    {
      memcpy(&referee_conventional.game_status,                   frame + index, sizeof(game_status_t));
    }
    break;
    case 0x0002:
    {
      memcpy(&referee_conventional.game_result,                   frame + index, sizeof(game_result_t));
    }
    break;
    case 0x0003:
    {
      memcpy(&referee_conventional.game_robot_HP,                 frame + index, sizeof(game_robot_HP_t));
    }
    break;
    case 0x0101:
    {
      memcpy(&referee_conventional.event_data,                    frame + index, sizeof(event_data_t));
    }
    break;
    case 0x0102:
    {
      memcpy(&referee_conventional.ext_supply_projectile_action,  frame + index, sizeof(ext_supply_projectile_action_t));
    }
    break;
    case 0x0104:
    {
      memcpy(&referee_conventional.referee_warning,               frame + index, sizeof(referee_warning_t));
    }
    break;
    case 0x0105:
    {
      memcpy(&referee_conventional.dart_info,                     frame + index, sizeof(dart_info_t));
    }
    break;
    case 0x0201:
    {
      memcpy(&referee_conventional.robot_status,                  frame + index, sizeof(robot_status_t));
    }
    break;
    case 0x0202:
    {
      memcpy(&referee_conventional.power_heat_data,               frame + index, sizeof(power_heat_data_t));
    }
    break;
    case 0x0203:
    {
      memcpy(&referee_conventional.robot_pos,                     frame + index, sizeof(robot_pos_t));
    }
    break;
    case 0x0204:
    {
      memcpy(&referee_conventional.buff,                          frame + index, sizeof(buff_t));
    }
    break;
    case 0x0205:
    {
      memcpy(&referee_conventional.air_support_data,              frame + index, sizeof(air_support_data_t));
    }
    break;
    case 0x0206:
    {
      memcpy(&referee_conventional.hurt_data,                     frame + index, sizeof(hurt_data_t));
    }
    break;
    case 0x0207:
    {
      memcpy(&referee_conventional.shoot_data,                    frame + index, sizeof(shoot_data_t));
    }
    break;
    case 0x0208:
    {
      memcpy(&referee_conventional.projectile_allowance,          frame + index, sizeof(projectile_allowance_t));
    }
    break;
    case 0x0209:
    {
      memcpy(&referee_conventional.rfid_status,                   frame + index, sizeof(rfid_status_t));
    }
    break;
    case 0x020A:
    {
      memcpy(&referee_conventional.dart_client_cmd,               frame + index, sizeof(dart_client_cmd_t));
    }
    break;
    case 0x020B:
    {
      memcpy(&referee_conventional.ground_robot_position,         frame + index, sizeof(ground_robot_position_t));
    }
    break;
    case 0x020C:
    {
      memcpy(&referee_conventional.radar_mark_data,               frame + index, sizeof(radar_mark_data_t));
    }
    break;
    case 0x020D:
    {
      memcpy(&referee_conventional.sentry_info,                   frame + index, sizeof(sentry_info_t));
    }
    break;
    case 0x020E:
    {
      memcpy(&referee_conventional.radar_info,                    frame + index, sizeof(radar_info_t));
    }
    break;
    case 0x0301:
    {
        memcpy(&referee_conventional.robot_interaction_data,        frame + index, sizeof(robot_interaction_data_t));
//        if(referee_conventional.robot_interaction_data.data_cmd_id == 0x0201)
//        {
//            if(referee_conventional.robot_interaction_data.user_data[0] == 1)
//            {
//                Referee_Send2Screen_Tx();
//            }
//        }
    }
    break;
    case 0x0302:
    {
      memcpy(&image_transmission.custom_robot_data,               frame + index, sizeof(custom_robot_data_t));
    }
    break;
    case 0x0303:
    {
      memcpy(&map_interaction.map_command,                        frame + index, sizeof(map_command_t));
    }
    break;
    case 0x0304:
    {
      memcpy(&image_transmission.remote_control,                  frame + index, sizeof(remote_control_t));
    }
    break;
    case 0x0305:
    {
      memcpy(&map_interaction.map_robot_data,                     frame + index, sizeof(map_robot_data_t));
    }
    break;
    case 0x0306:
    {
      memcpy(&custom_client_data,                                 frame + index, sizeof(custom_client_data));
    }
    break;
    case 0x0307:
    {
      memcpy(&map_interaction.map_data,                           frame + index, sizeof(map_data_t));
    }
    break;
    case 0x0308:
    {
      memcpy(&map_interaction.custom_info,                        frame + index, sizeof(custom_info_t));
    }
    break;
    default:
    {
      break;
    }
  }
}



