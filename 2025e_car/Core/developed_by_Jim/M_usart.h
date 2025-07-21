#ifndef __M_USART_H
#define __M_USART_H

#include "board.h"
extern uint8_t USART3_RxData;

void Uart_Init(void);
void send_firewater_data(void);
void Process_Drone_Data(void);  // 处理飞机数据函数
typedef struct
{
    float drone_x;
    float drone_y;
    uint8_t fire_id; 

}drone_data_t;
extern drone_data_t drone_data;



#endif