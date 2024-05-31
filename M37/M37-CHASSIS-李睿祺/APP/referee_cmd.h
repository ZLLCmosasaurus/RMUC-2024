#ifndef __REFEREE_CMD_H__
#define __REFEREE_CMD_H__

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "referee.h"
#include "bsp_referee.h"

typedef enum
{
  HERO = 0,
  ENGINEER,
  STANDARD_3,
  STANDARD_4,
  STANDARD_5,
}Enemy_Robot_Id;

typedef __packed struct
{
  uint16_t HP;
  uint16_t p_HP;
  uint32_t Invincible_Time;
  bool Invincible_State;
}Robot_Status_t;

#define MAX_SIZE         128    //�ϴ��������ĳ���
#define frameheader_len  5       //֡ͷ����
#define cmd_len          2       //�����볤��
#define crc_len          2       //CRC16У��

#define SOF              0xA5
#define RADAR_TX_ID      0x200
#define ROBOT_INTER_ID   0x0301

void Referee_Send2Radar_Tx(void);
void Detect_invincibility_status(uint8_t* _status);

#endif
