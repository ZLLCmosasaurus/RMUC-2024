/**
 * @file drv_math.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_MATH_H
#define DRV_MATH_H

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include "limits.h"
#include "arm_math.h"
#include "stdint.h"
#include "float.h"

/* Exported macros -----------------------------------------------------------*/

//RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
//rad/s换算到RPM
#define RADPS_TO_RPM (60.0f / (2.0f * PI))
//deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)
//rad换算到deg
#define RAD_TO_DEG (180.0f / PI)
//摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)
//重力加速度
//合肥9.7933
#define GRAVITY_ACCELERATE (9.7933f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Math_Endian_Reverse_16(void *Address);
void Math_Endian_Reverse_16(void *Source, void *Destination);
void Math_Endian_Reverse_32(void *Address);
void Math_Endian_Reverse_32(void *Source, void *Destination);

uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

float Math_Sinc(float x);

int32_t Math_Float_To_Int(const float &x, const float &Float_Min, const float &Float_Max, const int32_t &Int_Min, const int32_t &Int_Max);
float Math_Int_To_Float(const int32_t &x, const int32_t &Int_Min, const int32_t &Int_Max, const float &Float_Min, const float &Float_Max);

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
void Math_Constrain(Type &x, const Type &Min, const Type &Max)
{
    if (x < Min)
    {
        x = Min;
    }
    else if (x > Max)
    {
        x = Max;
    }
}

/**
 * @brief 判断是否在阈值函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
bool Math_Judge_Threshold(const Type &x, const Type &Min, const Type &Max)
{
    if ((x < Min)||(x > Max))
    {
        return false;
    }
    else 
    {
        return true;
    }
}

/**
 * @brief 求绝对值
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
