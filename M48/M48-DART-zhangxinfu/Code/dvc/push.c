#include "push.h"
#include "math.h"

//push电机速度环pid
const float PUSH_VPID_PID[7]={15,0.01,20,11000,2000,0};

void Push_Init(Dart_Push_t *push)
{
    //push电机初始化 速度环
    PID_Init(&push->push_motor.vpid,PUSH_VPID_PID);
}
void Calculate_Length(Dart_Push_t *push)
{
    //编码器值除以一圈8191，除以减速比，乘以丝杆系数    value/8191/36*8e-3
    //计算当前行程 单位m
    push->actual_length = (float)(push->push_motor.total_encoder - push->push_motor.start_encoder) / PUSH_ENCODER_COUNT / PUSH_MOTOR_REDUCTION * PUSH_ROUND_TO_LENGTH ;
}
void Push_Task(Dart_Push_t *push)
{
                    //计算push行程
                    Calculate_Length(push);
                    //push电机的状态判断 偏差大于0.5mm就开始推 push行程死区
                    if(push->push_command == 1)
                    {
                            if(fabs(push->target_length - push->actual_length) > 0.001f) 
                        {
                            if(push->actual_length < push->target_length)
                            {
                                push->push_status= PUSH_FORWARD;
                                push->push_motor.target_speed = PUSH_MOTOR_SPEED;
                            }
                            else if(push->actual_length > push->target_length)
                            {
                                push->push_status = PUSH_BACKWARD;
                                push->push_motor.target_speed = -PUSH_MOTOR_SPEED;
                            }
                        }
                        else
                        {
                                push->push_status = PUSH_CLOSE;
                                push->push_motor.target_speed = PUSH_MOTOR_STOP;    
                        }
                    }
                    else 
                    {
                                push->push_status = PUSH_CLOSE;
                                push->push_motor.target_speed = PUSH_MOTOR_STOP;
                    }
                    
    //push电机速度环计算
    PID_Calc(&push->push_motor.vpid, push->push_motor.target_speed, push->push_motor.actual_speed);

    //can发送
    CAN2_Push_Pitch_Tx_Data[0] = (int16_t)push->push_motor.vpid.out >> 8;
    CAN2_Push_Pitch_Tx_Data[1] = (int16_t)push->push_motor.vpid.out;
}





