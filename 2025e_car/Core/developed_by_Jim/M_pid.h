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

// 初始化所有PID控制器
void pid_init_all(void);
// 初始化左右轮PID控制器
void wheels_pid_init(void);
// 左轮PID速度控制
float left_wheel_pid_control(float target_speed);
// 右轮PID速度控制
float right_wheel_pid_control(float target_speed);
// 航向角PID控制
float yaw_pid_control(float target_yaw);
// 处理航向角差值，确保在-180到180度范围内
float normalize_angle(float angle);
// 双轮PID速度控制，计算并输出PWM
void wheels_pid_control(float left_target_speed, float right_target_speed);
// 使用全局目标速度的PID控制
void wheels_pid_control_auto(void);
// 带转向控制的PID速度控制
void wheels_pid_control_with_yaw(float base_speed, float target_yaw);
// 使用全局目标速度和航向角的PID控制
void wheels_pid_control_auto_with_yaw(void);
// 设置目标速度
void set_target_speed(float left, float right);
// 设置目标航向角
void set_target_yaw(float yaw);
// 重置PID控制器
void pid_reset(PID_TypeDef *pid);
// 重置所有PID控制器
void wheels_pid_reset(void);

// 导出PID结构体实例，便于外部访问
extern PID_TypeDef left_wheel_pid;
extern PID_TypeDef right_wheel_pid;
extern PID_TypeDef yaw_pid;

// 导出全局目标速度变量
extern float g_left_target_speed;
extern float g_right_target_speed;
extern float g_target_yaw;

#endif