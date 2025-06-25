#ifndef __ENCODER_H
#define __ENCODER_H

#include "board.h"
/*

Total Pulses = 500 * 28 * 4 =56000 转一圈 56000脉冲

*/
    #define WHEEL_D 6.0f

void encoder_count();

void Calculate_speed(left_wheel_rpm, right_wheel_rpm);

#endif


