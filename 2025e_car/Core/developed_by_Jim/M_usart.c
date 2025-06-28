#include "M_usart.h"
#include "usart.h"
uint8_t USART3_RxData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		// HAL_UART_Transmit_IT(&huart3,&USART3_RxData,1);
		get_jy61p(USART3_RxData);
		HAL_UART_Receive_IT(&huart3, &USART3_RxData, 1);
	}
}