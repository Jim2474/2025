#ifndef __JY61P_H
#define __JY61P_H

#include "board.h"
#include "main.h"                 // Device header
#include <stdio.h>

// 定义JY61P数据包大小
#define JY61P_PACKET_SIZE 11

 //IMU数据 结构体
typedef struct 
{
	float AX; 
	float AY;
	float AZ;
	float WX; 
	float WY; 
	float WZ; 
	float RollX; 
	float PitchY; 
	float YawZ;

}IMU_data_Struct;
extern IMU_data_Struct IMU_data;
// 数据包清空用变量

//函数声明
void get_jy61p(uint8_t RxData);
void jy61p_init_dma(void);
void get_jy61p_dma(uint8_t *buffer, uint16_t size);

#endif


















// #ifndef __JY61P_H
// #define __JY61P_H

// #include "board.h"
// #include "main.h"                 // Device header
// #include <stdio.h>

//  //IMU数据 结构体
// typedef struct 
// {
// 	float AX; 
// 	float AY;
// 	float AZ;
// 	float WX; 
// 	float WY; 
// 	float WZ; 
// 	float RollX; 
// 	float PitchY; 
// 	float YawZ;

// }IMU_data_Struct;
// extern IMU_data_Struct IMU_data;
// // 数据包清空用变量

// //函数声明
// void get_jy61p(uint8_t RxData);

// #endif

















