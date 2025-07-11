#ifndef __BOARD_H
#define __BOARD_H



#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdint.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"
#include "M_pid.h"


//用户函数
#include "motor_pwm.h"
#include "Mytimer.h"
#include "Encoder.h"
#include "M_pid.h"
#include "jy61p.h"
#include "M_usart.h"
#include "M_navy.h"
#include "oled.h"
#include "string.h"
#include "Mission.h"
#include "M_servo.h"

//资源分配表：
/*

左轮编码器：TIM3  PA6 PA7
右轮编码器：TIM5  PA0 PA1 
左轮PWM:TIM4 CH3 PD14 
右轮PWM:TIM4 CH4  PD15
舵机X PWM: TIM2 CH1 PA5
舵机Y PWM: TIM2 CH2 PB3

TIM1：任务调度1ms定时
TIM2：双舵机
TIM3：左轮编码器
TIM4：电机双通道PWM输出
TIM5：右轮编码器
TIM6：
TIM7：
TIM8：

USART1：
USART2：
USART3：串口陀螺仪 波特率：115200 DMA接收
USART4：蓝牙模块 用于调试 DMA转运
USART5：摄像头数据解析
USART6：

    
*/


#endif