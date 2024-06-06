/**
 * @file alg_fsm.cpp
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief 有限自动机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
Class_FSM Dart_FSM;
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 状态机初始化
 *
 * @param __Status_Number 状态数量
 * @param __Now_Status_Serial 当前指定状态机初始编号
 */
void Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial)
{
    Dart_FSM.Status_Number = __Status_Number;

    Dart_FSM.Now_Status_Serial = __Now_Status_Serial;

    //所有状态全刷0
    for (int i = 0; i < Dart_FSM.Status_Number; i++)
    {
        Dart_FSM.Status[i].Status_Stage = Status_Stage_DISABLE;
        Dart_FSM.Status[i].Time = 0;
    }

    //使能初始状态
    Dart_FSM.Status[__Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}

/**
 * @brief 获取FSM当前状态
 *
 * @return uint8_t FSM当前状态
 */
uint8_t Get_Now_Status_Serial(void)
{
    return (Dart_FSM.Now_Status_Serial);
}

/**
 * @brief 设置状态改变
 *
 * @param Next_Status_serial 下一个状态
 */
void Set_Status(uint8_t __Next_Status_serial)
{
    //失能当前状态, 计时器清零
    Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Status_Stage = Status_Stage_DISABLE;
    Dart_FSM.Status[Dart_FSM.Now_Status_Serial].Time = 0;

    //转到下一个状态
    Dart_FSM.Status[__Next_Status_serial].Status_Stage = Status_Stage_ENABLE;
    Dart_FSM.Now_Status_Serial = __Next_Status_serial;
}

/**
 * @brief 重新加载状态机定时器
 *
 */


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
