#include "M_servo.h"

/* 宏定义 */
#define SERVO_MIN_PULSE 500       // 最小脉宽 500μs
#define SERVO_MAX_PULSE 2500      // 最大脉宽 2500μs
#define SERVO_DEFAULT_DURATION 20 // 默认运动时间(ms)

/* 私有变量 */
static ServoControl_t servo_x;   // X舵机控制
static ServoControl_t servo_y;   // Y舵机控制
static VisionData_t vision_data; // 视觉反馈数据

/* 私有函数声明 */
static void Servo_SetPWM(uint32_t channel, uint32_t pulse);
static float Servo_LimitAngle(float angle, float min_angle, float max_angle);
static uint32_t Servo_AngleToPulse(float angle, int offset);

/**
 * @brief 舵机控制初始化
 */
void Servo_Init(void)
{
    /* 伪代码：定时器初始化部分，后续需要替换为实际的HAL库代码 */
    // 1. 初始化定时器为PWM模式
    // 配置TIMx为PWM模式，频率50Hz (周期20ms)
    HAL_TIM_PWM_Init(&htim2);

    // 3. 启动PWM输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // 初始化X舵机PID控制器
   pid_init(&servo_x.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD,
            SERVO_PID_MAX_OUT, SERVO_PID_MIN_OUT);
    servo_x.current_angle = SERVO_X_CENTER_ANGLE;
    servo_x.target_angle = SERVO_X_CENTER_ANGLE;
    servo_x.offset = 0;
    servo_x.is_running = 0;
    servo_x.duration = SERVO_DEFAULT_DURATION;

//    // 初始化Y舵机PID控制器
   pid_init(&servo_y.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD,
            SERVO_PID_MAX_OUT, SERVO_PID_MIN_OUT);
    servo_y.current_angle = SERVO_Y_CENTER_ANGLE;
    servo_y.target_angle = SERVO_Y_CENTER_ANGLE;
    servo_y.offset = 0;
    servo_y.is_running = 0;
    servo_y.duration = SERVO_DEFAULT_DURATION;

    // 初始化视觉数据
    vision_data.error_x = 0.0f;
    vision_data.error_y = 0.0f;
    vision_data.target_detected = 0;
    vision_data.last_update = HAL_GetTick();

    // 设置舵机到中心位置
    Servo_SetXAngle(SERVO_X_CENTER_ANGLE, SERVO_DEFAULT_DURATION);
    Servo_SetYAngle(SERVO_Y_CENTER_ANGLE, SERVO_DEFAULT_DURATION);
//      Servo_SetXAngle(90, SERVO_DEFAULT_DURATION);
//    Servo_SetYAngle(90, SERVO_DEFAULT_DURATION);
}

/**
 * @brief 设置X舵机角度
 * @param angle 目标角度(0-180度)
 * @param duration 运动时间(ms)，0表示立即执行
 */
void Servo_SetXAngle(float angle, uint32_t duration)
{
    // 限制角度范围
    angle = Servo_LimitAngle(angle, SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);

    // 更新目标角度和运动时间
    servo_x.target_angle = angle;
    servo_x.duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration);

    if (duration == 0)
    {
        // 立即执行，不使用平滑过渡
        servo_x.current_angle = angle;
        uint32_t pulse = Servo_AngleToPulse(angle, servo_x.offset);
        Servo_SetPWM(SERVO_X_CHANNEL, pulse);
        servo_x.is_running = 0;
    }
    else
    {
        // 使用平滑过渡
        servo_x.inc_times = servo_x.duration / 20; // 计算需要递增的次数

        if (servo_x.target_angle > servo_x.current_angle)
        {
            servo_x.angle_inc = (servo_x.target_angle - servo_x.current_angle) / servo_x.inc_times;
        }
        else
        {
            servo_x.angle_inc = (servo_x.current_angle - servo_x.target_angle) / servo_x.inc_times;
            servo_x.angle_inc = -servo_x.angle_inc;
        }

        servo_x.is_running = 1;
    }
}

/**
 * @brief 设置Y舵机角度
 * @param angle 目标角度(0-180度)
 * @param duration 运动时间(ms)，0表示立即执行
 */
void Servo_SetYAngle(float angle, uint32_t duration)
{
    // 限制角度范围
    angle = Servo_LimitAngle(angle, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);

    // 更新目标角度和运动时间
    servo_y.target_angle = angle;
    servo_y.duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration);

    if (duration == 0)
    {
        // 立即执行，不使用平滑过渡
        servo_y.current_angle = angle;
        uint32_t pulse = Servo_AngleToPulse(angle, servo_y.offset);
        Servo_SetPWM(SERVO_Y_CHANNEL, pulse);
        servo_y.is_running = 0;
    }
    else
    {
        // 使用平滑过渡
        servo_y.inc_times = servo_y.duration / 20; // 计算需要递增的次数

        if (servo_y.target_angle > servo_y.current_angle)
        {
            servo_y.angle_inc = (servo_y.target_angle - servo_y.current_angle) / servo_y.inc_times;
        }
        else
        {
            servo_y.angle_inc = (servo_y.current_angle - servo_y.target_angle) / servo_y.inc_times;
            servo_y.angle_inc = -servo_y.angle_inc;
        }

        servo_y.is_running = 1;
    }
}

/**
 * @brief 设置X舵机偏移量
 * @param offset 偏移量(-100到100)
 */
void Servo_SetXOffset(int offset)
{
    servo_x.offset = offset < -100 ? -100 : (offset > 100 ? 100 : offset);

    // 更新当前位置的PWM
    uint32_t pulse = Servo_AngleToPulse(servo_x.current_angle, servo_x.offset);
    Servo_SetPWM(SERVO_X_CHANNEL, pulse);
}

/**
 * @brief 设置Y舵机偏移量
 * @param offset 偏移量(-100到100)
 */
void Servo_SetYOffset(int offset)
{
    servo_y.offset = offset < -100 ? -100 : (offset > 100 ? 100 : offset);

    // 更新当前位置的PWM
    uint32_t pulse = Servo_AngleToPulse(servo_y.current_angle, servo_y.offset);
    Servo_SetPWM(SERVO_Y_CHANNEL, pulse);
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
 * @param target_detected 1目标检测标志(1:检测到目标 0:未检测到目标)
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
 * @brief 舵机控制周期性更新函数(建议在20ms定时器中断中调用)
 */
void Servo_Update(void)
{
    // 处理X舵机平滑运动
    if (servo_x.is_running)
    {
        servo_x.current_angle += servo_x.angle_inc;
        servo_x.inc_times--;

        if (servo_x.inc_times == 0)
        {
            servo_x.current_angle = servo_x.target_angle;
            servo_x.is_running = 0;
        }

        uint32_t pulse = Servo_AngleToPulse(servo_x.current_angle, servo_x.offset);
        Servo_SetPWM(SERVO_X_CHANNEL, pulse);
    }

    // 处理Y舵机平滑运动
    if (servo_y.is_running)
    {
        servo_y.current_angle += servo_y.angle_inc;
        servo_y.inc_times--;

        if (servo_y.inc_times == 0)
        {
            servo_y.current_angle = servo_y.target_angle;
            servo_y.is_running = 0;
        }

        uint32_t pulse = Servo_AngleToPulse(servo_y.current_angle, servo_y.offset);
        Servo_SetPWM(SERVO_Y_CHANNEL, pulse);
    }

    //只在检测到目标且舵机不在运动状态时进行PID控制
   if (vision_data.target_detected && !servo_x.is_running && !servo_y.is_running)
   {
       float x_correction = 0.0f, y_correction = 0.0f;

       // 计算X方向PID修正值(反向修正，误差为正时需要减小角度)
       x_correction = pid_calc(&servo_x.pid, 0, -vision_data.error_x);

       // 计算Y方向PID修正值(反向修正，误差为正时需要减小角度)
       y_correction = pid_calc(&servo_y.pid, 0, -vision_data.error_y);

       // 更新舵机角度 - 使用较短的过渡时间
       Servo_SetXAngle(servo_x.current_angle + x_correction, 20);
       Servo_SetYAngle(servo_y.current_angle + y_correction, 20);
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
    Servo_SetXAngle(SERVO_X_CENTER_ANGLE, SERVO_DEFAULT_DURATION);
    Servo_SetYAngle(SERVO_Y_CENTER_ANGLE, SERVO_DEFAULT_DURATION);
}

/**
 * @brief 设置舵机PWM输出
 * @param channel 通道
 * @param pulse 脉冲宽度(μs)
 */
static void Servo_SetPWM(uint32_t channel, uint32_t pulse)
{
    // 使用HAL库设置PWM值
    switch (channel)
    {
    case TIM_CHANNEL_1:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
        break;
    case TIM_CHANNEL_2:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
        break;
    default:
        break;
    }
}

/**
 * @brief 角度转换为脉宽
 * @param angle 角度(0-180度)
 * @param offset 偏移量(-100到100)
 * @return 脉宽值(μs)
 */
static uint32_t Servo_AngleToPulse(float angle, int offset)
{
    uint32_t pulse;

    // 角度到脉宽的转换 (0-180度 映射到 500-2500μs)
    pulse = (uint32_t)(SERVO_MIN_PULSE + (angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));

    // 添加偏移量
    pulse += offset;

    // 限制脉宽范围
    if (pulse < SERVO_MIN_PULSE)
        pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE)
        pulse = SERVO_MAX_PULSE;

    return pulse;
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
    }
    else if (angle > max_angle)
    {
        return max_angle;
    }
    else
    {
        return angle;
    }
}
