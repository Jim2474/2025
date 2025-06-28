#ifndef __M_PID_H
#define __M_PID_H

#include "board.h"

// PID参数宏定义，方便调节
#define PID_KP_DEFAULT  1.0f
#define PID_KI_DEFAULT  0.0f
#define PID_KD_DEFAULT  0.0f
#define PID_OUT_MAX     1000.0f
#define PID_OUT_MIN     -1000.0f

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float set;
    float actual;
    float err;
    float err_last;
    float err_sum;
    float out;
    float out_max;
    float out_min;
} PID_TypeDef;

// 初始化PID结构体
void pid_init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min);
// 计算PID输出
float pid_calc(PID_TypeDef *pid, float set, float actual);

#endif