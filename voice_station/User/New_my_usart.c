#include "New_my_usart.h"
uint8_t USART1_RxData[200];
uint8_t USART2_RxData[200];

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

drone_data_t drone_data ; // 初始化无人机数据

char send_buffer1[50];
void New_Myusart_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_RxData, sizeof(USART1_RxData));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)//飞机和单片机
	{ 
		
		if (strstr((char*)USART1_RxData, "@") != NULL)
		{
			// 解析code数据
			sscanf((char*)USART1_RxData,"code,%f,%f,@", &drone_data.drone_x, &drone_data.drone_y);
		}

		if (strstr((char*)USART1_RxData, "#") != NULL)
		{
			// 解析box数据
			sscanf((char*)USART1_RxData,"box,%d,%d,#", &box_pos, &box_num);
			
			// 将box_pos(1-26)转换为A1-A6, B1-B6, C1-C6, D1-D6格式
			char position_str[4] = {0};

			if (box_pos >= 1 && box_pos <= 26) 
			{
				char row = 'A' + (box_pos - 1) / 6;  // 计算行字母 A,B,C,D
				int col = ((box_pos - 1) % 6) + 1;   // 计算列数字 1-6
				sprintf(position_str, "%c%d", row, col);
			}

			sprintf(send_buffer1, "main.t3.txt=\"%s\"\xFF\xFF\xFF", position_str);

			// 等待第一次传输完成
			//while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);

			// 发送state信息 (box_pos直接对应state序号1-26)
			// if (box_pos >= 1 && box_pos <= 26)
			//  {
			// 	sprintf(send_buffer2, "state%d=%d\xFF\xFF\xFF", box_pos, box_num);
			// } 
			
			// HAL_UART_Transmit_DMA(&huart1, (uint8_t*)send_buffer2, strlen(send_buffer2));
		}

        		// 重新启动DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_RxData, sizeof(USART1_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}
