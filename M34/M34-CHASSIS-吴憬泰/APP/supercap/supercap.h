#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "usart.h"
#include "bsp_can.h"
#include "offline_check.h"
// extern uint16_t supercap_volt;
// extern uint16_t supercap_per;
// void supercap(void);
// void get_supercap_data(void);

#define Low_Voltage_Mode 0
#define Medium_Voltage_Mode 1
#define High_Voltage_Mode 2

#define Supercap_Connected 1
#define Supercap_Disconnected 0

#define Game_On
//#define Game_Off

typedef enum
{
    SUPER_CAP,
    BATTRY,
} enum_power_mode;

typedef enum
{
    CAP_OK,
    CAP_ERROR,
    CAP_ERROR_HANDLE,
} enum_error_status;

typedef struct
{
    float max_power;
    float actual_power;
    uint16_t power_buffer;
    float remain_power_buffer;

    float voltage;
    float percent;

    uint8_t change_flag;
    uint8_t diff_flag;
    uint32_t diff_cnt;
    uint8_t rx_buff[8];
    uint8_t tx_buff[8];

    enum_power_mode power_mode;
    enum_power_mode last_power_mode;
    offline_check_handle offline_check;
    Struct_FSM error_fsm;
    enum_error_status error_status;
} supercap_handle;

/***********************************************************
 *@Brief	���ⲿ���õı���
 ***********************************************************/
extern supercap_handle supercap_object;
extern float supercap_volt;
extern float supercap_per;
extern int Supercap_Connection_Status;
/***********************************************************
 *@Brief	���ⲿ���õĺ���
 ***********************************************************/
void supercap(int S_Cnt, int MS_Cnt);

void supercap_can_tx(void);
void set_power_mode(supercap_handle *ptr, enum_power_mode mode);
enum_error_status supercap_error_FSM(Struct_FSM *fsm, supercap_handle *ptr);
#endif
