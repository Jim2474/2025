#include "M_usart.h"
#include "usart.h"
uint8_t USART3_RxData;

uint8_t USART4_RxData[200];


extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

void Uart_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
	

	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	
	HAL_UART_Receive_IT(&huart3, &USART3_RxData, 1);
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
		HAL_UART_Transmit_IT(&huart3,&USART3_RxData,1);
		get_jy61p(USART3_RxData);
		HAL_UART_Receive_IT(&huart3, &USART3_RxData, 1);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart4)
	{
		//HAL_UART_Transmit_DMA(&huart4, USART4_RxData, Size);
		
        // 解析数据
        sscanf((char*)USART4_RxData,"!,%f,%f,%f,#", &left_wheel_pid.kp, &left_wheel_pid.ki, &left_wheel_pid.kd);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	}
	
}
int fputc(int ch, FILE *f)
{
	// 采用轮询方式发送1字节数据，超时时间设置为无限等待
	HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}