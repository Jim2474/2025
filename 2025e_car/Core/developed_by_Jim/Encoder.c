#include "Encoder.h"
// --- 定义全局变量 ---
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;
volatile uint16_t last_left_encoder = 0;  // 改为uint16_t类型，与定时器计数器匹配
volatile uint16_t last_right_encoder = 0;
volatile float left_wheel_rpm = 0.0f;//转速
volatile float right_wheel_rpm = 0.0f;

 float left_wheel_speed = 0.0f;//速度
 float right_wheel_speed = 0.0f;

// 定义脉冲常量
#define PULSES_PER_REVOLUTION 56000.0f  // 500 * 28 * 4 = 56000脉冲/转  13*28*4=1456
#define MS_PER_MINUTE 60000.0f          // 1分钟 = 60000毫秒

// --- 在1ms的中断回调函数中 调用 注意：保护变量
void encoder_count()
{
    // --- 处理左轮 ---
    uint16_t current_left_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t left_pulse_diff = (int16_t)(current_left_count - last_left_encoder);
    last_left_encoder = current_left_count;
    // 累加总脉冲数，用于长期计算
    left_encoder_count += left_pulse_diff;
       // printf("count:%d\n",left_encoder_count);

    // 计算RPM: (脉冲差/每转脉冲数) * (毫秒/分钟)
    // 1ms采样时间下: (脉冲差/56000) * 60000
    left_wheel_rpm = (left_pulse_diff / PULSES_PER_REVOLUTION) * MS_PER_MINUTE;

    // --- 处理右轮 ---
    uint16_t current_right_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t right_pulse_diff = (int16_t)(current_right_count - last_right_encoder);
    last_right_encoder = current_right_count;
    
    // 累加总脉冲数，用于长期计算
    right_encoder_count += right_pulse_diff;
    
    // 计算RPM
    right_wheel_rpm = (right_pulse_diff / PULSES_PER_REVOLUTION) * MS_PER_MINUTE;
}

// 计算线速度，输入为rpm，输出为cm/s
void Calculate_speed(float left_wheel_rpm,float right_wheel_rpm)
{

    // 线速度 = 圆周长 * 转速 / 60
    // 圆周长 = π * D
    left_wheel_speed=(3.1415926f * WHEEL_D * left_wheel_rpm) / 60.0f;
    right_wheel_speed=-(3.1415926f * WHEEL_D * right_wheel_rpm) / 60.0f;

}