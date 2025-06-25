#include "motor_pwm.h"
#include "tim.h"

// 设置指定通道占空比（0~1050）
void Motor_PWM_SetDuty(uint32_t channel, uint16_t duty)
{
    if (duty > 1050)
        duty = 1050;
    switch (channel)
    {
    case TIM_CHANNEL_1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
        break;
    case TIM_CHANNEL_2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
        break;
    case TIM_CHANNEL_3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);
        break;
    case TIM_CHANNEL_4:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);
        break;
    default:
        break;
    }
}

// 启动所有PWM通道
void Motor_PWM_StartAll(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

// 停止所有PWM通道
void Motor_PWM_StopAll(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    // 置零占空比，确保电机不转
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

// duty范围-1050~1050，正为正转，负为反转
void Motor_PWM_SetLeft(int16_t duty)
{
    if (duty > 1050)
        duty = 1050;
    if (duty < -1050)
        duty = -1050;
    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty); // 左轮正转
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    // 左轮反转
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -duty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void Motor_PWM_SetRight(int16_t duty)
{
    if (duty > 1050)
        duty = 1050;
    if (duty < -1050)
        duty = -1050;
    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty); // 右轮正转
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);    // 右轮反转
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -duty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

//定时调用 pwm输出总控制 函数
void Motor_PWM_Output(int16_t left, int16_t right)
{
    Motor_PWM_SetLeft(left);
    Motor_PWM_SetRight(right);
}
