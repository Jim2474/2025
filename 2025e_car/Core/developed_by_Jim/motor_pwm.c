#include "motor_pwm.h"
#include "tim.h"

#define GMR_PWM_MIN 580  // 电机启动最小PWM值
#define GMR_PWM_MAX 1050 // 最大PWM值
#define GMR_INPUT_MAX 1050 // 输入最大值
#define GMR_OFFSET_VALUE 150 // 固定偏移值
#define GMR_SPEED_THRESHOLD 0.5f // 速度阈值，当实际速度/目标速度大于此值时去除偏移

// 记录上次输入值
static int16_t last_left_input = 0;
static int16_t last_right_input = 0;
static uint16_t last_left_output = 0;
static uint16_t last_right_output = 0;
static uint8_t left_offset_state = 0; // 0:无偏移 1:有偏移
static uint8_t right_offset_state = 0;

// 外部变量，需要在其他地方定义
extern float left_wheel_speed;  // 左轮实际速度
extern float right_wheel_speed; // 右轮实际速度


/**
 * @brief 使用固定偏移计算PWM值
 * @param input 输入值，范围为0到GMR_INPUT_MAX
 * @param actual_speed 实际速度
 * @param target_speed 目标速度
 * @return 计算后的PWM值
 */
static uint16_t Motor_PWM_FixedOffset(uint16_t input, float actual_speed, float target_speed)
{
    if (input == 0) return 0; // 如果输入为0，输出也为0，电机停止
    
    // 计算速度比例
    float speed_ratio = 0.0f;
    if (target_speed != 0) 
	{
        speed_ratio = actual_speed / target_speed;
        if (speed_ratio < 0) speed_ratio = 0; // 防止方向相反
        if (speed_ratio > 1) speed_ratio = 1; // 限制最大值
    }
    
    // 判断是否需要偏移
    if (speed_ratio < GMR_SPEED_THRESHOLD) 
    {
        // 需要偏移，给予固定偏移值
        uint16_t pwm = GMR_PWM_MIN ;
        
        // 确保不超过最大值
        if (pwm > GMR_PWM_MAX) pwm = GMR_PWM_MAX;
        
        printf("@@@@@speed_ratio: speed_ratio=%.2f, pwm=%d\n", speed_ratio, pwm);
        return pwm;
    } 
    else 
    {
        // 不需要偏移，直接输出输入值
        return 0;
    }
}

// 设置指定通道占空比（0~1050）
void Motor_PWM_SetDuty(uint32_t channel, uint16_t duty)
{
    if (duty > GMR_PWM_MAX)
        duty = GMR_PWM_MAX;
    
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
    
    // 重置上次输入值
    last_left_input = 0;
    last_right_input = 0;
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
    
    // 重置上次输入值
    last_left_input = 0;
    last_right_input = 0;
}

// duty范围-1050~1050，正为正转，负为反转
void Motor_PWM_SetLeft(int16_t duty)
{
    uint16_t mapped_duty;
    
    if (duty > GMR_INPUT_MAX)
        duty = GMR_INPUT_MAX;
    if (duty < -GMR_INPUT_MAX)
        duty = -GMR_INPUT_MAX;
    if(Motor_PWM_FixedOffset(duty, left_wheel_speed, g_left_target_speed)!=0)
    {
        mapped_duty=Motor_PWM_FixedOffset(duty, left_wheel_speed, g_left_target_speed)+duty;
        printf("#####need offset: duty=%d, mapped_duty=%d\n", duty, mapped_duty);
    }
    else
    {
        mapped_duty=duty;
    }

    
    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, mapped_duty); // 左轮正转
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);           // 左轮反转
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, mapped_duty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    
    // 更新上次输入值
    last_left_input = duty;
}

void Motor_PWM_SetRight(int16_t duty)
{
    uint16_t mapped_duty;
    
    if (duty > GMR_INPUT_MAX)
        duty = GMR_INPUT_MAX;
    if (duty < -GMR_INPUT_MAX)
        duty = -GMR_INPUT_MAX;
    
    if(Motor_PWM_FixedOffset(duty, right_wheel_speed, g_right_target_speed)!=0)
    {
        mapped_duty=Motor_PWM_FixedOffset(duty, right_wheel_speed, g_right_target_speed)+duty;
    }
    else
    {
        mapped_duty=duty;
    }
    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, mapped_duty); // 右轮正转
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);           // 右轮反转
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, mapped_duty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    
    // 更新上次输入值
    last_right_input = duty;
}

//定时调用 pwm输出总控制 函数
void Motor_PWM_Output(int16_t left, int16_t right)
{
    Motor_PWM_SetLeft(left);
    Motor_PWM_SetRight(right);
}
