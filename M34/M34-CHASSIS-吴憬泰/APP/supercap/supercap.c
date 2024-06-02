#include "supercap.h"
#include "referee.h"
#include "usart.h"

/***********************************************************
 *@Brief	���ⲿ���õı���
 ***********************************************************/
int Supercap_Connection_Status = Supercap_Disconnected;
int Supercap_UART_RX_KeepAlive_Flag = 0;
int Capacitor_State = 0;				 // ����ʣ������״̬��0Ϊ�ͣ�1Ϊ��
float supercap_volt = 0;				 // �������ݵ�ѹ
int Power_Mode = 0, Last_Power_Mode = 0; // ����ָʾ���̼���ģʽ
float supercap_per = 0;					 // �������ݵ����ٷֱ�

supercap_handle supercap_object;

/***********************************************************
 *@Brief	����supercap.c�ļ���ʹ�õı���
 ***********************************************************/
int Global_Time = 0;

/***********************************************************
 *@Brief	����supercap.c��������˳��
 ***********************************************************/
void Supercap_Trans_RefereeData(void);
void Supercap_Keep_Alive(void);
void PowerMode_Judgement(void);

/***********************************************************
 * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
 * @param[in]     none
 * @param[out]    ����ͬ���ʵ��ַ�
 * @retval        none
 ***********************************************************/

extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
extern uint8_t supercap_reboot_flag;
void supercap(int S_Cnt, int MS_Cnt)
{
	Global_Time = S_Cnt * 1000 + MS_Cnt; // ����ȫ��ʱ��
	if (Global_Time % 200 == 0)
		Supercap_Keep_Alive();
	if (Global_Time % 100 == 0)
	{
		Supercap_Trans_RefereeData();
		if (supercap_reboot_flag == 1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // �Ͽ�����̵���
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		//		PowerMode_Judgement();
	}
}
/***********************************************************
 *@fuction	:Supercap_Keep_Alive
 *@brief		:ȷ�ϳ�������״̬
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
extern void UART_TX_Supercap_Connection_Check(int Global_Time);
int Keep_Alive_Time_Cnt;
void Supercap_Keep_Alive(void)
{
	Keep_Alive_Time_Cnt++;							// ��ʱ
	UART_TX_Supercap_Connection_Check(Global_Time); // ���Ͳ�ѯ����
	// ��bsp_uart.c�Ĵ����ж��У����յ���ѯ������Keep_Alive_Time_Cnt������
	if (Keep_Alive_Time_Cnt > 10) // 1.2sû���յ����練����
		Supercap_Connection_Status = Supercap_Disconnected;
	else
		Supercap_Connection_Status = Supercap_Connected;
}

/***********************************************************
 *@fuction	:Supercap_Trans_RefereeData
 *@brief		:�򳬵緢�Ͳ���ϵͳ����
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
void Supercap_Trans_RefereeData(void)
{
	// �򳬵緢�͹�����Ϣ
	uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
	fp32 UART_TX_Power = 0;
	get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max); // ��ȡ����ϵͳ����
	UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);								   // �򳬵緢�͹���������������̵�ǰ����
}

/***********************************************************
 *@fuction	:PowerMode_Judgement
 *@brief		:���ݳ���״̬���Ƶ��̹���ģʽ
 *@param		:
 *@return	:void
 *@author	:DGYin
 *@date		:2023-05-29
 ***********************************************************/
void PowerMode_Judgement(void)
{
	// ���̹���ģʽ�жϣ��ֳ����Ƿ������������
	if (Supercap_Connection_Status == Supercap_Connected)
	{
		if (supercap_volt > 200)
			Power_Mode = High_Voltage_Mode;
		if (supercap_volt <= 150)
			Power_Mode = Low_Voltage_Mode;
		if (Last_Power_Mode == Low_Voltage_Mode && supercap_volt < 180)
			Power_Mode = Low_Voltage_Mode;
	}
	else
	{
		Power_Mode = Medium_Voltage_Mode;
	}
	// ��¼�ϴι���ģʽ
	Last_Power_Mode = Power_Mode;
}

void set_power_mode(supercap_handle *ptr, enum_power_mode mode)
{
	ptr->power_mode = mode;
	if (mode == SUPER_CAP)
	{
		HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_RESET);
	}
	if (mode == BATTRY)
	{
		HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_SET);
		ptr->percent = 0.0f;
		ptr->voltage = 0.0f;
	}
}

void set_battry_mode(supercap_handle *ptr)
{
	HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_SET);
	ptr->percent = 0.0f;
	ptr->voltage = 0.0f;
}

void set_cap_mode(supercap_handle *ptr)
{
	HAL_GPIO_WritePin(Supercap_GPIO_Port, Supercap_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Battry_GPIO_Port, Battry_Pin, GPIO_PIN_RESET);
}

// ����״̬ 0 ���� 1 ���ƴ��� 2 ���� 3 ������
enum_error_status supercap_error_FSM(Struct_FSM *fsm, supercap_handle *ptr)
{
	const float power_threshold = 1.6f;
	const float power_threshold_low = 0.8f;
	if (ptr->power_mode == SUPER_CAP)
	{
		fsm->Status[fsm->Now_Status_Serial].Time++;
		switch (Get_Now_Status_Serial(fsm))
		{
		// ����״̬
		case 0:
			ptr->error_status = CAP_OK;
			if (ptr->actual_power <= power_threshold && ptr->actual_power >= 0.8f)
			{
				// ����״̬->���ƴ���״̬
				Set_FSM_Status(fsm, 1);
			}
			break;
		// ���ƴ���״̬
		case 1:
			if (ptr->actual_power <= power_threshold_low || ptr->actual_power >= power_threshold)
			{
				// ���ƴ���״̬->����״̬
				Set_FSM_Status(fsm, 0);
			}
			else if (fsm->Status[fsm->Now_Status_Serial].Time >= 1000)
			{
				// ���ƴ���״̬->����״̬
				Set_FSM_Status(fsm, 2);
			}
			break;
		// ����״̬
		case 2:
			ptr->error_status = CAP_ERROR;
			if (ptr->actual_power <= power_threshold_low || ptr->actual_power >= power_threshold)
			{
				// ����״̬->����״̬
				Set_FSM_Status(fsm, 0);
			}
			else
			{
				// ����״̬->������״̬
				Set_FSM_Status(fsm, 3);
			}
			break;
			// ������
		case 3:

			ptr->error_status = CAP_ERROR_HANDLE;
			if (fsm->Status[fsm->Now_Status_Serial].Time < 1000)
			{
				set_battry_mode(ptr);
			}
			else
			{
				set_cap_mode(ptr);
			}
			if (ptr->actual_power <= power_threshold_low || ptr->actual_power >= power_threshold)
			{
				// ������״̬->����״̬
				Set_FSM_Status(fsm, 0);
			}

			break;
		}
		return ptr->error_status;
	}
}

void supercap_can_tx(void)
{
	supercap_object.actual_power = power_heat_data_t.chassis_power;
	supercap_object.max_power = (float)robot_state.chassis_power_limit;

	supercap_object.power_buffer = power_heat_data_t.chassis_power_buffer;
	supercap_object.remain_power_buffer = (float)power_heat_data_t.chassis_power_buffer;

	supercap_object.max_power = supercap_object.max_power + supercap_object.remain_power_buffer - 15.0f;
	if (supercap_object.remain_power_buffer - 10.f < 0)
	{
		supercap_object.max_power = (float)robot_state.chassis_power_limit;
	}
	if (supercap_object.max_power == 0)
	{
		supercap_object.max_power = 45;
	}
	memcpy(supercap_object.tx_buff, &supercap_object.max_power, 4);

	memcpy(supercap_object.tx_buff + 4, &supercap_object.actual_power, 4);

	CAN_Send_Data(&hcan2, SUPER_CAP_TX_ID, supercap_object.tx_buff, 8);
}
