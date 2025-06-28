#include "M_pid.h"

// 定义左右轮PID结构体实例
PID_TypeDef left_wheel_pid;
PID_TypeDef right_wheel_pid;

// 定义采样时间常量
#define DT_100HZ 0.01f  // 100Hz的周期为10ms

// 左右轮PID参数宏定义
#define LEFT_WHEEL_KP  2.5f
#define LEFT_WHEEL_KI  0.1f
#define LEFT_WHEEL_KD  0.05f
#define RIGHT_WHEEL_KP 2.5f
#define RIGHT_WHEEL_KI 0.1f
#define RIGHT_WHEEL_KD 0.05f

// 初始化PID结构体
void pid_init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->set = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
    pid->out = 0;
    pid->out_max = out_max;
    pid->out_min = out_min;
}

// 计算PID输出
float pid_calc(PID_TypeDef *pid, float set, float actual)
{
    pid->set = set;
    pid->actual = actual;
    pid->err = pid->set - pid->actual;
    
    // 积分项考虑采样时间
    pid->err_sum += pid->err * DT_100HZ;
    
    // 积分限幅，防止积分饱和
    if(pid->err_sum > 1000.0f) pid->err_sum = 1000.0f;
    if(pid->err_sum < -1000.0f) pid->err_sum = -1000.0f;
    
    // 微分项考虑采样时间
    float d_err = (pid->err - pid->err_last) / DT_100HZ;
    
    // PID输出计算
    pid->out = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * d_err;

    // 输出限幅
    if(pid->out > pid->out_max) pid->out = pid->out_max;
    if(pid->out < pid->out_min) pid->out = pid->out_min;

    pid->err_last = pid->err;
    return pid->out;
}

// 初始化左右轮PID控制器
void wheels_pid_init(void)
{
    // 初始化左轮PID
    pid_init(&left_wheel_pid, LEFT_WHEEL_KP, LEFT_WHEEL_KI, LEFT_WHEEL_KD, PID_OUT_MAX, PID_OUT_MIN);
    
    // 初始化右轮PID
    pid_init(&right_wheel_pid, RIGHT_WHEEL_KP, RIGHT_WHEEL_KI, RIGHT_WHEEL_KD, PID_OUT_MAX, PID_OUT_MIN);
}

// 全局目标速度变量，可以被其他模块设置
float g_left_target_speed = 0.0f;
float g_right_target_speed = 0.0f;

// 设置目标速度函数
void set_target_speed(float left, float right)
{
    g_left_target_speed = left;
    g_right_target_speed = right;
}

// 左轮PID速度控制
float left_wheel_pid_control(float target_speed)
{
    return pid_calc(&left_wheel_pid, target_speed, left_wheel_speed);
}

// 右轮PID速度控制
float right_wheel_pid_control(float target_speed)
{
    return pid_calc(&right_wheel_pid, target_speed, right_wheel_speed);
}

// 双轮PID速度控制，计算并输出PWM
void wheels_pid_control(float left_target_speed, float right_target_speed)
{
    // 计算左轮PID输出
    float left_pwm = left_wheel_pid_control(left_target_speed);
    
    // 计算右轮PID输出
    float right_pwm = right_wheel_pid_control(right_target_speed);
    
    // 输出PWM到电机
    Motor_PWM_Output((int16_t)left_pwm, (int16_t)right_pwm);
}

// 使用全局目标速度的PID控制
void wheels_pid_control_auto(void)
{
    wheels_pid_control(g_left_target_speed, g_right_target_speed);
}

// 重置PID控制器
void pid_reset(PID_TypeDef *pid)
{
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
    pid->out = 0;
}

// 重置所有PID控制器
void wheels_pid_reset(void)
{
    pid_reset(&left_wheel_pid);
    pid_reset(&right_wheel_pid);
}