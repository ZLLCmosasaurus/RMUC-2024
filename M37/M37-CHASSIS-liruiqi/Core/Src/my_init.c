#include "my_init.h"
void all_init(void)
{
  Chassis_PID_Init();
	referee_usart_fifo_init();
	remote_control_init();
	bsp_can_init();
	HAL_TIM_Base_Start_IT(&htim3);
	referee_data_init();
}

