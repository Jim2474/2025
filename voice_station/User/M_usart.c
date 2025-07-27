#include "M_usart.h"
#include "usart.h"
#include "jy61p.h"

uint8_t USART3_DMA_Buffer[JY61P_PACKET_SIZE*3]; // JY61P DMA接收缓冲区
uint8_t USART1_RxData[200];
uint8_t USART3_RxData;
uint8_t USART4_RxData[200];
uint8_t USART5_RxData[200];

// 串口发送相关变量
uint8_t uart_tx_buffer[128]; // 发送缓冲区
volatile uint8_t uart_tx_busy = 0; // 发送忙标志

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

drone_data_t drone_data ; // 初始化无人机数据
uint8_t new_data_flag = 0;  // 新数据标志位（全局变量）

void Uart_Init(void)
{
	// 初始化UART4 DMA接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
		
	// 初始化UART5 DMA接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, USART5_RxData, sizeof(USART5_RxData));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	

	// 初始化USART3 DMA接收陀螺仪数据
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_DMA_Buffer, sizeof(USART3_DMA_Buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	
	// 初始化USART3 DMA接收陀螺仪数据
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_RxData, sizeof(USART1_RxData));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
     
	drone_data.drone_x = 0.0f;
	drone_data.drone_y = 0.0f;	
	drone_data.fire_id = 0; // 初始化无人机数据

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
	if (huart == &huart1)//飞机和单片机
	{
		static uint8_t fire_data_received = 0;  // 标志：是否已接收过火源数据

		// 接收中断只设置标志位，不做解析
		new_data_flag = 1;

		// 只在第一次接收到有效火源数据时赋值
		if (!fire_data_received)
		 {
			float temp_fire_x, temp_fire_y;
			uint8_t temp_fire_id;

			// 尝试解析火源数据
			if (sscanf((char*)USART1_RxData,"fire,%f,%f,%d,@", &temp_fire_x, &temp_fire_y, &temp_fire_id) == 3) {
				// 验证火源ID是否有效
				if (temp_fire_id >= 1 && temp_fire_id <= 6)
				{
					// 只在第一次接收到有效数据时赋值
					fire_x_f = temp_fire_x;
					fire_y_f = temp_fire_y;
					drone_data.fire_id = temp_fire_id;
					fire_data_received = 1;  // 标记已接收过数据
				}
			}
		}

		// 重新启动DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1_RxData, sizeof(USART1_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}


	if (huart == &huart3)//陀螺仪
	{
		// 处理陀螺仪DMA数据
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1); // 指示灯切换状态
		get_jy61p_dma(USART3_DMA_Buffer, Size);
		
		// 重新启动DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_DMA_Buffer, sizeof(USART3_DMA_Buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
	else if (huart == &huart4)//蓝牙通信
	{
		//HAL_UART_Transmit_DMA(&huart4, USART4_RxData, Size);
		//在这里写火源坐标解析 调用 Mission_StartByFireId 解析任务 
        // 解析数据
        sscanf((char*)USART4_RxData,"!,%f,%f,%f,%f,%f,#", &left_wheel_pid.kp, &left_wheel_pid.ki, &left_wheel_pid.kd,&g_left_target_speed,&g_right_target_speed);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, USART4_RxData, sizeof(USART4_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	}
	else if (huart == &huart5)//视觉通信
	{
		//OLED_ShowNum(10,6,11,2,16,0);

        sscanf((char*)USART5_RxData,"!,%f,%f,%d,#", &vision_data.error_x, &vision_data.error_y, &vision_data.target_detected);
		// 处理maixcam数据
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, USART5_RxData, sizeof(USART5_RxData));
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
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

/**
 * @brief 处理飞机数据（在主循环中每500ms调用一次）
 */
void Process_Drone_Data(void)
{
	static uint32_t last_process_time = 0;
	uint32_t current_time = HAL_GetTick();

	// 每500ms处理一次，且有新数据时才处理
	if ((current_time - last_process_time) >= 500 && new_data_flag) {
		// 验证数据包完整性（简单检查是否包含@结束符）
		if (strstr((char*)USART1_RxData, "@") != NULL) {
			// 解析code数据
			sscanf((char*)USART1_RxData,"code,%f,%f,@", &drone_data.drone_x, &drone_data.drone_y);
		}

		new_data_flag = 0;  // 清除标志位
		last_process_time = current_time;
	}
}