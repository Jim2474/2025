#include "Encoder.h"
// --- 定义全局变量 ---
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;
volatile int32_t last_left_encoder = 0;
volatile int32_t last_right_encoder = 0;
volatile float left_wheel_rpm = 0.0f;//转速
volatile float right_wheel_rpm = 0.0f;

volatile float left_wheel_speed = 0.0f;//速度
volatile float right_wheel_speed = 0.0f;


// --- 在1ms的中断回调函数中 调用 注意：保护变量
void encoder_count()
{
    // --- 处理左轮 ---
    left_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t left_pulse_diff = left_encoder_count - last_left_encoder;
    last_left_encoder = left_encoder_count;
    left_wheel_rpm = (left_pulse_diff / 56000.0f) * 60000.0f;

    // --- 处理右轮 ---
    right_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int32_t right_pulse_diff = right_encoder_count - last_right_encoder;
    last_right_encoder = right_encoder_count;
    right_wheel_rpm = (right_pulse_diff / 56000.0f) * 60000.0f;
  
}

// 计算线速度，输入为rpm，输出为cm/s
void Calculate_speed(left_wheel_rpm,right_wheel_rpm)
{

    // 线速度 = 圆周长 * 转速 / 60
    // 圆周长 = π * D
    left_wheel_speed=(3.1415926f * WHEEL_D * left_wheel_rpm) / 60.0f;
    right_wheel_speed=(3.1415926f * WHEEL_D * right_wheel_rpm) / 60.0f;

}