#ifndef __TJCUSARTHMI_H__
#define __TJCUSARTHMI_H__

#include <stdio.h>
#include "board.h"
#include "M_usart.h"  // 包含drone_data_t类型定义
/**
	打印到屏幕串口
*/



#define TJC_UART huart2
#define TJC_UART_INS USART2
extern UART_HandleTypeDef huart2;

void set_image_aph(uint8_t pic_id, uint8_t aph_val);

void display_coordinates(const char *txt_id, int x, int y, int z);

void draw_line_on_screen(int x1, int y1, int x2, int y2, int color);

void draw_fly(int x1, int y1);

void tjc_send_string(char *str);
void tjc_send_txt(char* objname, char* attribute, char* txt);
void tjc_send_val(char* objname, char* attribute, int val);
void tjc_send_nstring(char* str, unsigned char str_length);
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);

// 飞机轨迹显示函数
void update_drone_display(float drone_x, float drone_y);
static void redraw_all_tracks(void);
void update_drone_display_with_fire(float drone_x, float drone_y, uint8_t fire_id);
void clear_drone_track(void);
void drone_display_task(void);
void redraw_tracks_after_control_update(void);


#define RINGBUFFER_LEN	(500)     //定义最大接收字节数 500

#define usize getRingBufferLength()
#define code_c() initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x) read1ByteFromRingBuffer(x)

extern uint8_t RxBuffer[1];
extern uint32_t msTicks;


#endif
