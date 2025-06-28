#include "M_pid.h"

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
    pid->err_sum += pid->err;
    float d_err = pid->err - pid->err_last;
    pid->out = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * d_err;

    // 限幅
    if(pid->out > pid->out_max) pid->out = pid->out_max;
    if(pid->out < pid->out_min) pid->out = pid->out_min;

    pid->err_last = pid->err;
    return pid->out;
}