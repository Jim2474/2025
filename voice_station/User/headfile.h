#ifndef __HEADFILE_H
#define __HEADFILE_H
#include "stm32f1xx_hal.h"
#include "main.h"
#include "string.h"

#include "usart.h"
#include "Myusart.h"
#include "New_my_usart.h"
#include "tjc_usart_hmi.h"
#include "stm32f1xx_hal_uart.h"
#include "stdint.h"
#include "stdio.h"
#include "M_timer.h"
#include "key.h"

  extern uint8_t box_pos;
extern uint8_t box_num;
extern  char send_buffer2[50];
extern char send_buffer1[50];
extern int8_t Key_KeyNumber;


#endif

