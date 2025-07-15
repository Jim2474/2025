#include "M_pid.h"

// 定义左右轮PID结构体实例
PID_TypeDef left_wheel_pid;
PID_TypeDef right_wheel_pid;
// 定义转向环PID结构体实例
PID_TypeDef yaw_pid;

// 定义采样时间常量
#define DT_100HZ 0.01f // 100Hz的周期为10ms

// 左右轮PID参数宏定义
#define LEFT_WHEEL_KP 65.0f
#define LEFT_WHEEL_KI 30.0f
#define LEFT_WHEEL_KD 0.0f
#define RIGHT_WHEEL_KP 65.0f
#define RIGHT_WHEEL_KI 30.0f
#define RIGHT_WHEEL_KD 0.0f

// 转向环PID参数宏定义
#define YAW_KP 0.5f
#define YAW_KI 0.0f
#define YAW_KD 0.0f
#define YAW_OUT_MAX 400.0f // 转向输出限幅
#define YAW_OUT_MIN -400.0f

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
    pid->err_last_last = 0; // 初始化上上次误差
    pid->err_sum = 0;
    pid->out = 0;
    pid->out_max = out_max;
    pid->out_min = out_min;
}

/**
 * @brief 增量式PID计算
 * @param pid PID结构体指针
 * @param set 设定值
 * @param actual 实际值
 * @return 增量式PID输出
 */
float pid_calc(PID_TypeDef *pid, float set, float actual)
{
    float increment; // PID增量
    
    // 保存上上次误差
    float err_last_last = pid->err_last;
    
    // 更新设定值和实际值
    pid->set = set;
    pid->actual = actual;
    
    // 计算当前误差
    pid->err = pid->set - pid->actual;
    
    // 特殊处理停车情况 - 如果目标速度为0且实际速度很小，直接返回0
    if (fabsf(pid->set) < 0.1f && fabsf(pid->actual) < 2.0f) {
        pid->out = 0.0f;
        pid->err_last = 0.0f;
        return 0.0f;
    }
    
    // 计算增量：△u(k) = Kp[e(k)-e(k-1)] + Ki*e(k) + Kd[e(k)-2e(k-1)+e(k-2)]
    increment = pid->kp * (pid->err - pid->err_last) +           // 比例项
                pid->ki * pid->err * DT_100HZ +                  // 积分项
                pid->kd * (pid->err - 2*pid->err_last + err_last_last) / DT_100HZ; // 微分项
    
    // 更新误差历史
    pid->err_last = pid->err;
    
    // 更新输出 = 上次输出 + 增量
    pid->out += increment;
    
    // 输出限幅
    if (pid->out > pid->out_max)
        pid->out = pid->out_max;
    if (pid->out < pid->out_min)
        pid->out = pid->out_min;
    
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
    // 先将角度转换到-360到360度范围内
    angle = fmodf(angle, 360.0f);

    // 然后转换到-180到180度范围内
    if (angle > 180.0f)
        angle -= 360.0f;
    else if (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

// 航向角PID控制
float yaw_pid_control(float target_yaw)
{
    float current_yaw = IMU_data.YawZ; // 获取当前航向角（已转换为-180到180度范围）

    // 计算航向角误差，并进行规范化处理
    float yaw_error = normalize_angle(target_yaw - current_yaw);

    // 使用规范化后的误差进行PID计算
    yaw_pid.set = 0;             // 设置目标误差为0
    yaw_pid.actual = -yaw_error; // 当前误差作为反馈值，取负是为了保持控制方向一致

    // 计算PID输出
    float output = pid_calc(&yaw_pid, yaw_pid.set, yaw_pid.actual);

    return output;
}

// 双轮PID速度控制，计算并输出PWM
void wheels_pid_control(float left_target_speed, float right_target_speed)
{
    float left_pwm = left_wheel_pid_control(left_target_speed);                                                                                                                 // 计算左轮PID输出
    float right_pwm = right_wheel_pid_control(right_target_speed);    
                                                                                                          // 计算右轮PID输出
    Motor_PWM_Output((int16_t)left_pwm, (int16_t)right_pwm);
    //printf("112actual:%f,%f,%d,%f,%f,%f\n", left_wheel_speed, right_wheel_speed, (int16_t)g_left_target_speed, IMU_data.YawZ, left_wheel_pid.kp,  currentPosition.x); // 打印实际速度
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
    float yaw_output = yaw_pid_control(g_target_yaw);    // 计算转向PID输出
    float left_speed = g_left_target_speed - yaw_output; // 根据转向输出调整左右轮速度
    float right_speed = g_right_target_speed + yaw_output;
    wheels_pid_control(left_speed, right_speed); // 输出到电机
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
    pid->err_last_last = 0; // 重置上上次误差
    pid->err_sum = 0;
    pid->out = 0;
}

// 使用全局目标速度的PID控制
void wheels_pid_control_auto(void)
{
     wheels_pid_control(g_left_target_speed, g_right_target_speed);

}
