#ifndef __MYTIMER_H
#define __MYTIMER_H

#include "board.h"

void Mytimer_Init(void);
void TIM2_Task_1000Hz(void);
void TIM2_Task_100Hz(void);

// 非阻塞延时函数
uint8_t delay_ms_nb(uint32_t delay_ms);
uint8_t delay_ms_nb_id(uint32_t delay_ms, uint8_t id);
uint32_t get_system_tick_ms(void);

#endif