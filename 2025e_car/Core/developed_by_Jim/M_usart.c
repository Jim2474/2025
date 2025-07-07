#include "M_usart.h"
#include "usart.h"
#include "jy61p.h"

uint8_t USART3_RxData;
uint8_t USART3_DMA_Buffer[JY61P_PACKET_SIZE*3]; // JY61P DMA接收缓冲区
uint8_t USART4_RxData[200];

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

void Uart_Init(void)
{
	// 初始化UART4 DMA接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	
	// 初始化USART3 DMA接收陀螺仪数据
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_DMA_Buffer, sizeof(USART3_DMA_Buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	
	// 保留中断方式接收作为备用
	// HAL_UART_Receive_IT(&huart3, &USART3_RxData, 1);
}


// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	if (huart == &huart3)
// 	{
// 		// HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
// 		// //HAL_UART_Transmit_IT(&huart3,&USART3_RxData,1);
// 		// get_jy61p(USART3_RxData);
// 		// HAL_UART_Receive_IT(&huart3, &USART3_RxData, 1);
// 	}
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart4)
	{
		//HAL_UART_Transmit_DMA(&huart4, USART4_RxData, Size);
		
        // 解析数据
        sscanf((char*)USART4_RxData,"!,%f,%f,%f,%f,%f#", &left_wheel_pid.kp, &left_wheel_pid.ki, &left_wheel_pid.kd,&g_left_target_speed,&g_right_target_speed);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	}
	else if (huart == &huart3)
	{
		// 处理陀螺仪DMA数据
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1); // 指示灯切换状态
		get_jy61p_dma(USART3_DMA_Buffer, Size);
		
		// 重新启动DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_DMA_Buffer, sizeof(USART3_DMA_Buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

// int fputc(int ch, FILE *f)
// {
// 	// 采用轮询方式发送1字节数据，超时时间设置为无限等待
// 	HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
// 	return ch;
// }

// 实现FireWater协议
void send_firewater_data(void)
{
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_time < 50) return; // 20Hz发送频率
    last_time = now;
    
    float data[4] = {left_wheel_speed, right_wheel_speed, IMU_data.YawZ, currentPosition.x};
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F}; // FireWater协议的结束标志
    
    // 使用DMA一次性发送
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)data, sizeof(data));
    while(uart_tx_busy); // 等待DMA传输完成
    HAL_UART_Transmit_DMA(&huart4, tail, sizeof(tail));
}