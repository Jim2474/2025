#ifndef __ENCODER_H
#define __ENCODER_H

#include "board.h"
/*

Total Pulses = 500 * 28 * 4 =56000 转一圈 56000脉冲

*/
extern volatile float left_wheel_rpm;
extern volatile float right_wheel_rpm;
extern  float left_wheel_speed;
extern  float right_wheel_speed;

    #define WHEEL_D 6.0f

void encoder_count();

void Calculate_speed(float left_wheel_rpm, float right_wheel_rpm);

#endif


