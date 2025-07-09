#include "M_servo.h"

/* 私有变量 */
static ServoControl_t servo_x; // X舵机控制
static ServoControl_t servo_y; // Y舵机控制
static VisionData_t vision_data; // 视觉反馈数据

/* 私有函数声明 */
static void Servo_SetPWM(uint32_t channel, float angle);
static float Servo_LimitAngle(float angle, float min_angle, float max_angle);

/**
 * @brief 舵机控制初始化
 */
void Servo_Init(void)
{
    /* 伪代码：定时器初始化部分，后续需要替换为实际的HAL库代码 */
    // 1. 初始化定时器为PWM模式
    // 配置TIMx为PWM模式，频率50Hz (周期20ms)
    // HAL_TIM_PWM_Init(&htimx);
    
    // 2. 配置通道
    // 配置TIMx的通道1用于X舵机
    // 配置TIMx的通道2用于Y舵机
    // HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_1);
    // HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_2);
    
    // 3. 启动PWM输出
    // HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_2);
    
    // 初始化X舵机PID控制器
    pid_init(&servo_x.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD, 
             SERVO_PID_MAX_OUT, SERVO_PID_MIN_OUT);
    servo_x.current_angle = SERVO_X_CENTER_ANGLE;
    servo_x.target_angle = SERVO_X_CENTER_ANGLE;
    
    // 初始化Y舵机PID控制器
    pid_init(&servo_y.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD, 
             SERVO_PID_MAX_OUT, SERVO_PID_MIN_OUT);
    servo_y.current_angle = SERVO_Y_CENTER_ANGLE;
    servo_y.target_angle = SERVO_Y_CENTER_ANGLE;
    
    // 初始化视觉数据
    vision_data.error_x = 0.0f;
    vision_data.error_y = 0.0f;
    vision_data.target_detected = 0;
    vision_data.last_update = HAL_GetTick();
    
    // 设置舵机到中心位置
    Servo_SetXAngle(SERVO_X_CENTER_ANGLE);
    Servo_SetYAngle(SERVO_Y_CENTER_ANGLE);
}

/**
 * @brief 设置X舵机角度
 * @param angle 目标角度(0-180度)
 */
void Servo_SetXAngle(float angle)
{
    // 限制角度范围
    angle = Servo_LimitAngle(angle, SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
    
    // 更新当前角度
    servo_x.current_angle = angle;
    
    // 设置PWM
    Servo_SetPWM(SERVO_X_CHANNEL, angle);
}

/**
 * @brief 设置Y舵机角度
 * @param angle 目标角度(0-180度)
 */
void Servo_SetYAngle(float angle)
{
    // 限制角度范围
    angle = Servo_LimitAngle(angle, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
    
    // 更新当前角度
    servo_y.current_angle = angle;
    
    // 设置PWM
    Servo_SetPWM(SERVO_Y_CHANNEL, angle);
}

/**
 * @brief 获取X舵机当前角度
 * @return 当前角度(度)
 */
float Servo_GetXAngle(void)
{
    return servo_x.current_angle;
}

/**
 * @brief 获取Y舵机当前角度
 * @return 当前角度(度)
 */
float Servo_GetYAngle(void)
{
    return servo_y.current_angle;
}

/**
 * @brief 处理视觉反馈数据
 * @param error_x X方向误差
 * @param error_y Y方向误差
 * @param target_detected 目标检测标志(1:检测到目标 0:未检测到目标)
 */
void Servo_ProcessVisionData(float error_x, float error_y, uint8_t target_detected)
{
    // 更新视觉数据
    vision_data.error_x = error_x;
    vision_data.error_y = error_y;
    vision_data.target_detected = target_detected;
    vision_data.last_update = HAL_GetTick();
}

/**
 * @brief 舵机控制周期性更新函数(建议在定时器中断中调用)
 */
void Servo_Update(void)
{
    float x_correction = 0.0f, y_correction = 0.0f;
    
    // 只在检测到目标时进行PID控制
    if (vision_data.target_detected) {
        // 计算X方向PID修正值(反向修正，误差为正时需要减小角度)
        x_correction = pid_calc(&servo_x.pid, 0, -vision_data.error_x);
        
        // 计算Y方向PID修正值(反向修正，误差为正时需要减小角度)
        y_correction = pid_calc(&servo_y.pid, 0, -vision_data.error_y);
        
        // 更新舵机角度
        Servo_SetXAngle(servo_x.current_angle + x_correction);
        Servo_SetYAngle(servo_y.current_angle + y_correction);
    }
}

/**
 * @brief 重置舵机到中心位置
 */
void Servo_Reset(void)
{
    // 重置PID控制器
    pid_reset(&servo_x.pid);
    pid_reset(&servo_y.pid);
    
    // 舵机回到中心位置
    Servo_SetXAngle(SERVO_X_CENTER_ANGLE);
    Servo_SetYAngle(SERVO_Y_CENTER_ANGLE);
}

/**
 * @brief 设置舵机PWM输出
 * @param channel 通道
 * @param angle 角度(0-180度)
 */
static void Servo_SetPWM(uint32_t channel, float angle)
{
    uint32_t pulse;
    
    // 角度到脉宽的转换
    pulse = (uint32_t)(SERVO_MIN_PULSE + (angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    
    /* 伪代码：设置PWM占空比，后续需要替换为实际的HAL库代码 */
    // 根据通道设置对应的PWM值
    switch (channel)
    {
        case TIM_CHANNEL_1:
            // __HAL_TIM_SET_COMPARE(&htimx, TIM_CHANNEL_1, pulse);
            break;
        case TIM_CHANNEL_2:
            // __HAL_TIM_SET_COMPARE(&htimx, TIM_CHANNEL_2, pulse);
            break;
        default:
            break;
    }
}

/**
 * @brief 限制角度在有效范围内
 * @param angle 输入角度
 * @param min_angle 最小角度
 * @param max_angle 最大角度
 * @return 限制后的角度
 */
static float Servo_LimitAngle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle)
     {
        return min_angle;
    } else if (angle > max_angle) 
    {
        return max_angle;
    } else {
        return angle;
    }
}










