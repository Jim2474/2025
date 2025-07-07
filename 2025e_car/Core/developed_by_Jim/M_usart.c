#include "M_usart.h"
#include "usart.h"
#include "jy61p.h"

uint8_t USART3_RxData;
uint8_t USART3_DMA_Buffer[JY61P_PACKET_SIZE*3]; // JY61P DMA接收缓冲区
uint8_t USART4_RxData[200];

// 串口发送相关变量
uint8_t uart_tx_buffer[128]; // 发送缓冲区
volatile uint8_t uart_tx_busy = 0; // 发送忙标志

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
	
	// 初始化发送标志
	uart_tx_busy = 0;
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

/**
 * @brief DMA发送完成回调函数
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart4)
    {
        // 标记发送完成
        uart_tx_busy = 0;
    }
}

int fputc(int ch, FILE *f)
{
	// 采用轮询方式发送1字节数据，超时时间设置为无限等待
	HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

// /**
//  * @brief 发送数据到VOFA+ (FireWater协议)
//  */
// void send_firewater_data(void)
// {
//     static uint32_t last_time = 0;
//     uint32_t now = HAL_GetTick();
//     if (now - last_time < 50) return; // 20Hz发送频率
//     last_time = now;
    
//     // 如果上一次发送未完成，则跳过本次发送
//     if (uart_tx_busy)
//         return;
    
//     // 准备数据
//     float data[4] = {left_wheel_speed, right_wheel_speed, IMU_data.YawZ, left_wheel_pid.out};
    
//     // FireWater协议的结束标志 0x00 0x00 0x80 0x7F
//     uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    
//     // 将数据拷贝到发送缓冲区
//     memcpy(uart_tx_buffer, data, sizeof(data));
//     memcpy(uart_tx_buffer + sizeof(data), tail, sizeof(tail));
    
//     // 标记发送忙
//     uart_tx_busy = 1;
    
//     // 使用DMA发送数据
//     HAL_UART_Transmit_DMA(&huart4, uart_tx_buffer, sizeof(data) + sizeof(tail));
// }