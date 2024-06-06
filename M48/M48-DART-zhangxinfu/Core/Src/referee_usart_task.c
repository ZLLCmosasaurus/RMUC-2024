#include "referee_usart_task.h"

uint8_t usart1_RX_buf[USART_RX_BUF_LENGTH];
// fifo_s_t referee_fifo;
// uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];

extern ext_dart_remaining_time_t dart_remaining_time_t;//飞镖发射剩余时间
extern ext_dart_client_cmd_t dart_client_cmd_t;//飞镖发射站状态，及相应比赛剩余时间
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
 int flag_uart1 = 0;
uint16_t buffer_index = 0;
uint16_t cmd_id,data_length;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{    
    if (huart->Instance == USART1)
    {
			flag_uart1 = 2;
    uint16_t CMD_ID = 0;
    buffer_index = 0; 
    uint16_t buffer_index_max =  USART_RX_BUF_LENGTH- 1;
    while(buffer_index<buffer_index_max)
    {
          //通过校验和帧头
          if ((usart1_RX_buf[buffer_index]==0xA5) && 
              (verify_CRC8_check_sum(&usart1_RX_buf[buffer_index],5)==1))
          {
              //数据处理过程
              cmd_id=(usart1_RX_buf[buffer_index+6])&0xff;
              cmd_id=(cmd_id<<8)|usart1_RX_buf[buffer_index+5];  
              data_length=usart1_RX_buf[buffer_index+2]&0xff;
              data_length=(data_length<<8)|usart1_RX_buf[buffer_index+1];
              CMD_ID = cmd_id;
              switch (CMD_ID)
              {
                  case 0x0105://0x0105 飞镖发射口倒计时
                  {
                      if((buffer_index_max-buffer_index>=(sizeof(ext_dart_remaining_time_t)+7))&&
                      verify_CRC16_check_sum(&usart1_RX_buf[buffer_index],sizeof(ext_dart_remaining_time_t)+7)==1)
                      {
                          memcpy(&dart_remaining_time_t, &usart1_RX_buf[buffer_index+7], sizeof(ext_dart_remaining_time_t));
                          buffer_index+=sizeof(ext_dart_remaining_time_t)+7;
                      }
                  }
                  case 0x020A://0x0105 飞镖发射口倒计时
                  {
                      if((buffer_index_max-buffer_index>=(sizeof(ext_dart_client_cmd_t)+7))&&
                      verify_CRC16_check_sum(&usart1_RX_buf[buffer_index],sizeof(ext_dart_client_cmd_t)+7)==1)
                      {
                          memcpy(&dart_client_cmd_t, &usart1_RX_buf[buffer_index+7], sizeof(ext_dart_client_cmd_t));
                          buffer_index+=sizeof(ext_dart_client_cmd_t)+7;
                      }
                  }
              }
          }
          buffer_index++;
    } 
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_RX_buf, USART_RX_BUF_LENGTH);
    }
}




