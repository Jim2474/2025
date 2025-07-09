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

//资源分配表：
/*

左轮编码器：PA6 PA7
右轮编码器：PD12 PD13
左轮正转 TIM1_CH1  PE9
左轮反转 TIM1_CH2  PE11
右轮正转 TIM1_CH3  PE13
右轮反转 TIM1_CH4  PE14

TIM1：四通道PWM输出
TIM2：1ms定时
TIM3：左轮编码器
TIM4：右轮编码器
TIM5：双通道PWM输出
TIM6：
TIM7：
TIM8：

USART1：
USART2：
USART3：串口陀螺仪 波特率：115200
USART4：蓝牙模块 用于调试 DMA转运
USART5：
USART6：

    
*/


#endif