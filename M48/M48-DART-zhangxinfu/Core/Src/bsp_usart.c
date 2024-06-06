#include "bsp_usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
/*
总的来说，这段代码是用于初始化 USART6，
并配置了 USART6 的 DMA 接收通道以及空闲中断
，实现了 USART6 的数据接收功能。
*/
int flag_init = 0;
void USART1_RX_Init(uint8_t *rxdata_buf ,uint16_t dma_buf_amount)
{
    //enable the DMA transfer for the receiver request
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
     __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    //清除了 DMA 接收通道的传输完成标志位，以确保 DMA 在配置完成后可以正常工作
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_FLAG_GL3);
    //dma系统外设寄存器地址-->usart3数据寄存器地址
    hdma_usart1_rx.Instance->CPAR = (uint32_t)&(USART1->DR);  
    //设置缓冲区存放数据
    hdma_usart1_rx.Instance->CMAR = (uint32_t)(rxdata_buf);
    //设置传输数据长度
    hdma_usart1_rx.Instance->CNDTR = dma_buf_amount;
    //enable DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
	  //在 DMA 模式下接收大量数据
	    HAL_UART_Receive_DMA(&huart1,rxdata_buf,dma_buf_amount);
	
	flag_init = 100;
}


