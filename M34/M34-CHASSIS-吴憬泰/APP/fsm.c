#include "fsm.h"

void FSM_Init(Struct_FSM *FSM, uint8_t Status_Num, uint8_t Now_Status_Serial)
{
    FSM->Status_Num = Status_Num;
    FSM->Now_Status_Serial = 0;
    // ����״̬ȫˢ0
    for (uint8_t i = 0; i < Status_Num; i++)
    {
        FSM->Status[i].Status_Stage = Status_Stage_DISABLE;
        FSM->Status[i].Time = 0;
    }
    // ʹ�ܳ�ʼ״̬
    FSM->Status[Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}

void Set_FSM_Status(Struct_FSM *FSM, uint8_t Next_Status_serial)
{
    // ʧ�ܵ�ǰ״̬, ��ʱ������
    FSM->Status[FSM->Now_Status_Serial].Status_Stage = Status_Stage_DISABLE;
    FSM->Status[FSM->Now_Status_Serial].Time = 0;
    // ʹ����һ��״̬
    FSM->Status[Next_Status_serial].Status_Stage = Status_Stage_ENABLE;
    FSM->Now_Status_Serial = Next_Status_serial;
}

inline uint8_t Get_Now_Status_Serial(Struct_FSM *FSM)
{
    return FSM->Now_Status_Serial;
}

// ����״̬
typedef enum
{
    STATE_A,
    STATE_B,
    STATE_C,
    // ��Ӹ���״̬...
    STATE_NUM
} State_example;

void FSM_example(Struct_FSM *FSM)
{

    FSM_Init(FSM, 3, 0);

    // task loop ��������Ϊ1ms
    FSM->Status[FSM->Now_Status_Serial].Time++;
    // �Լ���һ��״̬ת��ͼ��Ȼ�����״̬ת��ͼд״̬��
    // ����״̬ת��ͼд״̬��
    switch (Get_Now_Status_Serial(FSM))
    {
    case STATE_A:
        // ����״̬A
        // �������ת��������ת��������״̬
        // Set_FSM_Status(&FSM, STATE_B)
        break;
    case STATE_B:
        // ����״̬B
        // �������ת��������ת��������״̬
        // Set_FSM_Status(&FSM, STATE_C)
        break;
    case STATE_C:
        // ����״̬C
        // �������ת��������ת��������״̬
        // Set_FSM_Status(&FSM, STATE_A)
        break;
        // ��Ӹ���״̬...
    }
}