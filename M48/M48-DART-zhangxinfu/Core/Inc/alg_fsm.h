/**
 * @file alg_fsm.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 有限自动机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef ALG_FSM_H
#define ALG_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported macros -----------------------------------------------------------*/

#define STATUS_MAX (10)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 状态所处的阶段
 *
 */
typedef enum 
{
    Status_Stage_DISABLE = 0,
    Status_Stage_ENABLE,
}Enum_Status_Stage;

/**
 * @brief 状态结构体
 *
 */
typedef struct 
{
    Enum_Status_Stage Status_Stage;
    uint32_t Time;
}Struct_Status;


typedef struct  
{
    Struct_Status Status[STATUS_MAX];
    uint8_t Status_Number;
    // FSM当前状态
    uint8_t Now_Status_Serial;

}Class_FSM;

/* Exported variables --------------------------------------------------------*/
extern Class_FSM Dart_FSM;
/* Exported function declarations --------------------------------------------*/

void Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial);
uint8_t Get_Now_Status_Serial(void);
void Set_Status(uint8_t Next_Status_serial);
// void Dart_FSM_TIM_Status_PeriodElapsedCallback(void);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
