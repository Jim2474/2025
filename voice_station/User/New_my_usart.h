#ifndef __NEW_MY_USART_H
#define __NEW_MY_USART_H
#include "headfile.h"

void New_Myusart_Init(void);

typedef struct
{
    float drone_x;
    float drone_y;
    uint8_t fire_id; 

}drone_data_t;
extern drone_data_t drone_data;


#endif

