#include "motor_pwm.h"
#include "tim.h"

#define GMR_PWM_MIN 0  // 电机启动最小PWM值
#define GMR_PWM_MAX 1050 // 最大PWM值
#define GMR_INPUT_MAX 1050 // 输入最大值
#define GMR_SPEED_THRESHOLD 0.9f // 速度阈值，当实际速度/目标速度大于此值时去除偏移


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
       
       return pwm;
   } 
   else 
   {
       // 不需要偏移，直接输出输入值
       return 0;
   }
}


// 启动所有PWM通道
void Motor_PWM_StartAll(void)
{
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    
}

// 停止所有PWM通道
void Motor_PWM_StopAll(void)
{
   HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
   HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
   // 置零占空比，确保电机不转
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

}

// 修改后的左轮控制函数
void Motor_PWM_SetLeft(int16_t duty)
{
   uint16_t pwm_value;
   
   if (duty > GMR_INPUT_MAX)
       duty = GMR_INPUT_MAX;
   if (duty < -GMR_INPUT_MAX)
       duty = -GMR_INPUT_MAX;
   
   // 计算实际PWM值
   pwm_value = (duty > 0) ? duty : -duty;
   
   // 设置方向控制引脚
   if (duty > 0)
   {
       // 正转
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
   }
   else if (duty < 0)
   {
       // 反转
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
   }
   else
   {
       // 停止
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
       pwm_value = 0;
   }
   
   // 设置PWM值到PWMA引脚
   // 这里需要根据您的硬件连接修改定时器通道
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_value);
   

}

// 右轮控制函数也需要类似修改
void Motor_PWM_SetRight(int16_t duty)
{
  
   // 使用PE13和PE14作为方向控制
   // 使用PWMB引脚作为PWM输出
 uint16_t pwm_value;
   
   if (duty > GMR_INPUT_MAX)
       duty = GMR_INPUT_MAX;
   if (duty < -GMR_INPUT_MAX)
       duty = -GMR_INPUT_MAX;
   
   // 计算实际PWM值
   pwm_value = (duty > 0) ? duty : -duty;
   
   // 设置方向控制引脚
   if (duty > 0)
   {
       // 正转
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
   }
   else if (duty < 0)
   {
       // 反转
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
   }
   else
   {
       // 停止
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
       pwm_value = 0;
   }
   
   // 设置PWM值到PWMA引脚
   // 这里需要根据您的硬件连接修改定时器通道
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_value);
}   

//定时调用 pwm输出总控制 函数
void Motor_PWM_Output(int16_t left, int16_t right)
{
    Motor_PWM_SetLeft(left);
    Motor_PWM_SetRight(right);
}
