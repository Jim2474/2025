#include "M_pid.h"

// 定义左右轮PID结构体实例
PID_TypeDef left_wheel_pid;
PID_TypeDef right_wheel_pid;
// 定义转向环PID结构体实例
PID_TypeDef yaw_pid;

// 定义采样时间常量
#define DT_100HZ 0.01f // 100Hz的周期为10ms

// 左右轮PID参数宏定义
#define LEFT_WHEEL_KP 2.5f
#define LEFT_WHEEL_KI 0.1f
#define LEFT_WHEEL_KD 0.05f
#define RIGHT_WHEEL_KP 2.5f
#define RIGHT_WHEEL_KI 0.1f
#define RIGHT_WHEEL_KD 0.05f

// 转向环PID参数宏定义
#define YAW_KP 2.0f
#define YAW_KI 0.05f
#define YAW_KD 0.2f
#define YAW_OUT_MAX 500.0f // 转向输出限幅
#define YAW_OUT_MIN -500.0f

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
    if (pid->err_sum > 1000.0f)
        pid->err_sum = 1000.0f;
    if (pid->err_sum < -1000.0f)
        pid->err_sum = -1000.0f;

    // 微分项考虑采样时间
    float d_err = (pid->err - pid->err_last) / DT_100HZ;

    // PID输出计算
    pid->out = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * d_err;

    // 输出限幅
    if (pid->out > pid->out_max)
        pid->out = pid->out_max;
    if (pid->out < pid->out_min)
        pid->out = pid->out_min;

    pid->err_last = pid->err;
    return pid->out;
}

// 初始化所有PID控制器
void pid_init_all(void)
{
    // 初始化左轮PID
    pid_init(&left_wheel_pid, LEFT_WHEEL_KP, LEFT_WHEEL_KI, LEFT_WHEEL_KD, PID_OUT_MAX, PID_OUT_MIN);

    // 初始化右轮PID
    pid_init(&right_wheel_pid, RIGHT_WHEEL_KP, RIGHT_WHEEL_KI, RIGHT_WHEEL_KD, PID_OUT_MAX, PID_OUT_MIN);
    // 初始化转向环PID
    pid_init(&yaw_pid, YAW_KP, YAW_KI, YAW_KD, YAW_OUT_MAX, YAW_OUT_MIN);
}

// 全局目标速度变量，可以被其他模块设置
float g_left_target_speed = 0.0f;
float g_right_target_speed = 0.0f;
// 全局目标航向角
float g_target_yaw = 0.0f;

// 设置目标速度函数
void set_target_speed(float left, float right)
{
    g_left_target_speed = left;
    g_right_target_speed = right;
}

// 设置目标航向角函数
void set_target_yaw(float yaw)
{
    g_target_yaw = yaw;
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

// 处理航向角差值，确保在-180到180度范围内
float normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// 航向角PID控制
float yaw_pid_control(float target_yaw)
{
    float current_yaw = IMU_data.YawZ;// 获取当前航向角
    float yaw_error = normalize_angle(target_yaw - current_yaw);// 计算航向角误差，并进行规范化处理
    // 使用规范化后的误差进行PID计算
    yaw_pid.set = 0;  // 设置目标误差为0
    yaw_pid.actual = -yaw_error;  // 当前误差作为反馈值，取负是为了保持控制方向一致
    float output = pid_calc(&yaw_pid, yaw_pid.set, yaw_pid.actual); // 计算PID输出
    return output;
}

// 双轮PID速度控制，计算并输出PWM
void wheels_pid_control(float left_target_speed, float right_target_speed)
{
    float left_pwm = left_wheel_pid_control(left_target_speed);    // 计算左轮PID输出
    float right_pwm = right_wheel_pid_control(right_target_speed); // 计算右轮PID输出
    Motor_PWM_Output((int16_t)left_pwm, (int16_t)right_pwm);    // 输出PWM到电机
}


// 带转向控制的PID速度控制
void wheels_pid_control_with_yaw(float base_speed, float target_yaw)
{
    // 计算转向PID输出
    float yaw_output = yaw_pid_control(target_yaw);

    // 根据转向输出调整左右轮速度
    float left_speed = base_speed - yaw_output;
    float right_speed = base_speed + yaw_output;

    // 输出到电机
    wheels_pid_control(left_speed, right_speed);
    
}

// 使用全局目标速度和航向角的PID控制
void wheels_pid_control_auto_with_yaw(void)
{
    float yaw_output = yaw_pid_control(g_target_yaw);   // 计算转向PID输出
    float left_speed = g_left_target_speed - yaw_output;    // 根据转向输出调整左右轮速度
    float right_speed = g_right_target_speed + yaw_output;
    wheels_pid_control(left_speed, right_speed);  // 输出到电机
}

// 重置所有PID控制器
void wheels_pid_reset(void)
{
    pid_reset(&left_wheel_pid);
    pid_reset(&right_wheel_pid);
    pid_reset(&yaw_pid);
}

//************************暂放 内部调用或者不必要*************** */
// 重置PID控制器
void pid_reset(PID_TypeDef *pid)
{
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
    pid->out = 0;
}

// 使用全局目标速度的PID控制
void wheels_pid_control_auto(void)
{
    wheels_pid_control(g_left_target_speed, g_right_target_speed);
}
