// #include "main.h"
// #include <stdio.h>
// #include "board.h"
// // 数据包清空用变量
// uint8_t buffer_index;
// // 数据接收计数器
// uint8_t Rx_Cnt = 0;
// // 数据包
// char jy61p_Buffer[44];
// // 各项数据
// // float AX, AY, AZ, WX, WY, WZ, RollX, PitchY, YawZ;

// // 定义结构体变量储存数据 10/2 jim
// IMU_data_Struct IMU_data;

// extern uint8_t USART3_RxData;
// const int YAW_WINDOW_SIZE = 3; // 使用const

// float yaw_buffer[YAW_WINDOW_SIZE] = {0}; // 存储姿态角Yaw的滑动窗口
// int yaw_buffer_index = 0;				 // 窗口索引

// // 计算滑动平均
// float calculate_average(float *buffer, int size)
// {
// 	float sum = 0;
// 	for (int i = 0; i < size; i++)
// 	{
// 		sum += buffer[i];
// 	}
// 	return sum / size;
// }

// void get_jy61p(uint8_t RxData)
// {
// 	// 完整数据包为44位
// 	// 将串口数据存入到数据包jy61p_Buffer，每接收一次Rx_Cnt计数一次
// 	jy61p_Buffer[Rx_Cnt] = RxData;
// 	Rx_Cnt++;
// 	// 接收完第二个数据后，校验帧头（0x55和0x51）
// 	if (Rx_Cnt == 2 && (jy61p_Buffer[0] != 0X55 || jy61p_Buffer[1] != 0X51))
// 	{
// 		for (buffer_index = 0; buffer_index < 44; buffer_index++)
// 		{
// 			jy61p_Buffer[buffer_index] = 0x00;
// 		}
// 		Rx_Cnt = 43;

// 	}

// 	// 确认帧头正确后，如果接收完44位数据
// 	if (Rx_Cnt == 44)
// 	{
// 		// 解析姿态角数据
// 		if (jy61p_Buffer[23] == 0x53)
// 		{
// 			// 解析 YawZ 并更新滑动窗口
// 			// float new_yaw = (float)(((jy61p_Buffer[29] << 8) | jy61p_Buffer[28]) / 32768.0 * 180);
// 			// if (new_yaw > 180)
// 			// 	new_yaw = -new_yaw;
// 			// // 实际计算新的滤波值
// 			// yaw_buffer[yaw_buffer_index] = new_yaw; // 直接赋值
// 			// // 更新索引
// 			// yaw_buffer_index = (yaw_buffer_index + 1) % YAW_WINDOW_SIZE;
// 			// float N_yaw = calculate_average(yaw_buffer, YAW_WINDOW_SIZE);

// 			// if (N_yaw < 0)
// 			// 	N_yaw = N_yaw + 360;
// 			// 计算滑动平均
// 			IMU_data.YawZ = (float)(((jy61p_Buffer[29] << 8) | jy61p_Buffer[28]) / 32768.0 * 180);
// 		}
// 		// 解析完毕，清空数据包和计数器
// 	}
// 	if (Rx_Cnt >= 44)
// 	{
// 		for (buffer_index = 0; buffer_index < 44; buffer_index++)
// 		{
// 			jy61p_Buffer[buffer_index] = 0x00;
// 		}
// 		Rx_Cnt = 0;
// 	}
// }
#include "jy61p.h"
// 数据包清空用变量
uint8_t buffer_index;
// 数据接收计数器
uint8_t Rx_Cnt = 0;
// 数据包
char jy61p_Buffer[11];

// DMA接收缓冲区
uint8_t jy61p_DMA_Buffer[JY61P_PACKET_SIZE * 3]; // 3倍数据包大小，确保能捕获完整帧

// 定义结构体变量储存数据 10/2 jim
IMU_data_Struct IMU_data;
extern uint8_t USART3_RxData;
int yaw_buffer_index = 0; // 窗口索引

/**
 * @brief 初始化JY61P DMA接收
 */
void jy61p_init_dma(void)
{
	// 在M_usart.c中实现DMA初始化
}

/**
 * @brief 处理JY61P的DMA接收数据
 * @param buffer DMA接收缓冲区
 * @param size 接收到的数据大小
 */
void get_jy61p_dma(uint8_t *buffer, uint16_t size)
{
	// 查找数据包帧头
	for (uint16_t i = 0; i < size - JY61P_PACKET_SIZE + 1; i++)
	{
		// 检查帧头 (0x55, 0x53)
		if (buffer[i] == 0x55 && buffer[i + 1] == 0x53)
		{
			// 确认有完整的数据包
			if (i + JY61P_PACKET_SIZE <= size)
			{
				// 获取原始角度值（0-360度）
				float raw_yaw = (float)(((buffer[i + 7] << 8) | buffer[i + 6]) / 32768.0 * 180);

				// 将角度转换为-180到180度范围
				if (raw_yaw > 180.0f)
				{
					raw_yaw -= 360.0f;
				}

				// 更新IMU数据
				IMU_data.YawZ = raw_yaw;

				// 找到有效帧后可以退出循环，或继续查找更多帧
				// 如果要处理所有帧，这里不退出
				// break;
			}
		}
	}
}

void get_jy61p(uint8_t RxData)
{
	// 完整数据包为44位
	// 将串口数据存入到数据包jy61p_Buffer，每接收一次Rx_Cnt计数一次
	jy61p_Buffer[Rx_Cnt] = RxData;
	Rx_Cnt++;
	// 接收完第二个数据后，校验帧头（0x55和0x51）
	if (Rx_Cnt == 2 && (jy61p_Buffer[0] != 0X55 && jy61p_Buffer[1] != 0X53))
	{
		for (buffer_index = 0; buffer_index < 11; buffer_index++)
		{
			jy61p_Buffer[buffer_index] = 0x00;
		}
		Rx_Cnt = 0;
	}

	// 确认帧头正确后，如果接收完44位数据
	if (Rx_Cnt == 11)
	{
		// 解析姿态角数据
		if (jy61p_Buffer[1] == 0x53)
		{
			// 获取原始角度值（0-360度）
			float raw_yaw = (float)(((jy61p_Buffer[7] << 8) | jy61p_Buffer[6]) / 32768.0 * 180);

			// 将角度转换为-180到180度范围
			if (raw_yaw > 180.0f)
			{
				raw_yaw -= 360.0f;
			}

			IMU_data.YawZ = raw_yaw;
		}
		// 解析完毕，清空数据包和计数器
	}
	if (Rx_Cnt >= 11)
	{
		for (buffer_index = 0; buffer_index < 11; buffer_index++)
		{
			jy61p_Buffer[buffer_index] = 0x00;
		}
		Rx_Cnt = 0;
	}
}
