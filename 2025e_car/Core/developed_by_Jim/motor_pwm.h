#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

/*

左轮编码器：PA6 PA7
右轮编码器：PD12 PD13
左轮正转 TIM1_CH1  PE9
左轮反转 TIM1_CH2  PE11
右轮正转 TIM1_CH3  PE13
右轮反转 TIM1_CH4  PE14
*/
#include "board.h"
#include "tim.h"

// 设置指定通道占空比（0~1050）
void Motor_PWM_SetDuty(uint32_t channel, uint16_t duty);
// 启动所有PWM通道
void Motor_PWM_StartAll(void);
// 停止所有PWM通道
void Motor_PWM_StopAll(void);
// duty范围-1050~1050，正为正转，负为反转
void Motor_PWM_SetLeft(int16_t duty);
void Motor_PWM_SetRight(int16_t duty);

// 输出左右轮速度
void Motor_PWM_Output(int16_t left, int16_t right);

#endif
