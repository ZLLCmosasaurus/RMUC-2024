#include "shoot.h"
#include "control_task.h"
//debug控制转动与停止
// int shoot_command = 0;

//摩擦轮速度环pid  注意是4行且每行有7个参数
//const float FRIC_VPID_PID[4][7]={{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0}};
float FRIC_VPID_PID[4][7]={{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0},{8.0,0.08,0.3,16500,2000,0}};
void Fric_Init(Dart_Fric_t* fric)
{   
    //摩擦轮电机初始化
    for(int i = 0; i < 4; i++)
    {
        PID_Init(&fric->fric_motor[i].vpid, FRIC_VPID_PID[i]);
    }
}

 
/**
 * @brief 摩擦轮任务函数
 * 
 * @param fric 摩擦轮结构体指针
 */
void Fric_Task(Dart_Fric_t* fric)
{
    //摩擦轮电机速度环计算+can发送
    for(int i = 0; i < 4; i++)
    {    
        PID_Calc(&fric->fric_motor[i].vpid,fric->fric_motor[i].target_speed, fric->fric_motor[i].actual_speed);
        CAN1_Fric_Tx_Data[i*2] = (int16_t)fric->fric_motor[i].vpid.out >> 8;
        CAN1_Fric_Tx_Data[i*2+1] = (int16_t)fric->fric_motor[i].vpid.out;
    }
}



