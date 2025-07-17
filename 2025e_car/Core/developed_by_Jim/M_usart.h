#ifndef __M_USART_H
#define __M_USART_H

#include "board.h"
extern uint8_t USART3_RxData;
void Uart_Init(void);
void send_firewater_data(void);
typedef struct
{
    float drone_x;
    float drone_y;
    uint8_t fire_id; // 0-360度

}drone_data_t;
extern drone_data_t drone_data;



#endif