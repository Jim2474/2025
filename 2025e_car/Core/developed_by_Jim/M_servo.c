#include "M_servo.h"

/* 宏定义 */
#define SERVO_MIN_PULSE 500       // 最小脉宽 500μs
#define SERVO_MAX_PULSE 2500      // 最大脉宽 2500μs
#define SERVO_DEFAULT_DURATION 20 // 默认运动时间(ms)

/* 私有变量 */
static ServoControl_t servo_x;   // X舵机控制
static ServoControl_t servo_y;   // Y舵机控制
 VisionData_t vision_data; // 视觉反馈数据

/* 私有函数声明 */
static void Servo_SetPWM(uint32_t channel, uint32_t pulse);
static float Servo_LimitAngle(float angle, float min_angle, float max_angle);
static uint32_t Servo_AngleToPulse(float angle, int offset);

static uint32_t Servo_AngleToPulse180(float angle, int offset);

/**
 * @brief 舵机控制初始化
 */
void Servo_Init(void)
{
    /* 伪代码：定时器初始化部分，后续需要替换为实际的HAL库代码 */
    // 1. 初始化定时器为PWM模式
    // 配置TIMx为PWM模式，频率50Hz (周期20ms)
    HAL_TIM_PWM_Init(&htim1);

    // 3. 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 初始化X舵机PID控制器
   pid_init_servo(&servo_x.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD,
            SERVO_PID_MAX_OUT, SERVO_PID_MIN_OUT);
    servo_x.current_angle = SERVO_X_CENTER_ANGLE;
    servo_x.target_angle = SERVO_X_CENTER_ANGLE;
    servo_x.offset = 0;
    servo_x.is_running = 0;
    servo_x.duration = SERVO_DEFAULT_DURATION;

//    // 初始化Y舵机PID控制器
   pid_init_servo(&servo_y.pid, SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD,
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
    // 确保角度在0-360范围内
    angle = fmodf(angle, 360.0f);
    if (angle < 0) angle = angle + 360.0f;
    
    // 直接计算最短路径
    float current = servo_x.current_angle;
    float diff = angle - current;
    
    // 标准化到-180到+180范围
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    // 计算目标角度
    float target = current + diff;
    
    
    // 限制角度范围
    target = Servo_LimitAngle(target, SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE);
    
    // 更新目标角度和运动时间
    servo_x.target_angle = target;
    servo_x.duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration);
    
    if (duration == 0)
    {
        // 立即执行，不使用平滑过渡
        servo_x.current_angle = target;
        uint32_t pulse = Servo_AngleToPulse(target, servo_x.offset);
        Servo_SetPWM(SERVO_X_CHANNEL, pulse);
        servo_x.is_running = 0;
    }
    else
    {
        // 使用平滑过渡
        servo_x.inc_times = servo_x.duration / 20; // 计算需要递增的次数
        
        // 计算角度增量，考虑最短路径
        float angle_diff = target - servo_x.current_angle;
        
        // 如果角度差超过180度，选择另一个方向
        if (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        
        servo_x.angle_inc = angle_diff / servo_x.inc_times;
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
    // 确保角度在0-180范围内（而不是0-360）
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 直接计算最短路径
    float current = servo_y.current_angle;
    float diff = angle - current;
    
    // 标准化到-90到+90范围（对于180度舵机）
    if (diff > 90.0f) {
        diff -= 180.0f;
    } else if (diff < -90.0f) {
        diff += 180.0f;
    }
    
    // 计算目标角度
    float target = current + diff;
    
    // 限制角度范围
    target = Servo_LimitAngle(target, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE);
    
    // 更新目标角度和运动时间
    servo_y.target_angle = target;
    servo_y.duration = duration < 20 ? 20 : (duration > 30000 ? 30000 : duration);
    
    if (duration == 0)
    {
        // 立即执行，不使用平滑过渡
        servo_y.current_angle = target;
        uint32_t pulse = Servo_AngleToPulse180(target, servo_y.offset);  // 使用180度专用函数
        Servo_SetPWM(SERVO_Y_CHANNEL, pulse);
        servo_y.is_running = 0;
    }
    else
    {
        // 使用平滑过渡
        servo_y.inc_times = servo_y.duration / 20; // 计算需要递增的次数
        servo_y.angle_inc = diff / servo_y.inc_times;
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
    uint32_t pulse = Servo_AngleToPulse180(servo_y.current_angle, servo_y.offset);
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
    // 静态变量记录上一次的视觉误差值
    static float last_error_x = -999.0f;  // 初始化为不可能的值
    static float last_error_y = -999.0f;  // 初始化为不可能的值
    static uint8_t first_run = 1;         // 首次运行标志

  

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

        // 使用180度专用函数
        uint32_t pulse = Servo_AngleToPulse180(servo_y.current_angle, servo_y.offset);
        Servo_SetPWM(SERVO_Y_CHANNEL, pulse);
    }
    // 如果不是首次运行且视觉误差值没有变化，直接返回，避免重复校准
    if (!first_run && vision_data.error_x == last_error_x && vision_data.error_y == last_error_y)
    {
        return;
    }

    // 首次运行后清除标志
    if (first_run)
    {
        first_run = 0;
    }
    //只在检测到目标且舵机不在运动状态时进行PID控制
  //if (vision_data.target_detected && !servo_x.is_running && !servo_y.is_running)
    //if (!servo_y.is_running&&!servo_y.is_running)
    if ( !servo_x.is_running && !servo_y.is_running)
   {
       float x_correction = 0.0f, y_correction = 0.0f;
       
       // 添加死区控制，忽略微小误差
       float deadzone = 2.0f; // 调整死区大小
       
       if (fabsf(vision_data.error_x) > deadzone)
           x_correction = pid_calc_servo(&servo_x.pid, 0, -vision_data.error_x);
       else
           pid_reset_servo(&servo_x.pid); // 在死区内重置积分项
           
       if (fabsf(vision_data.error_y) > deadzone)
           y_correction = pid_calc_servo(&servo_y.pid, 0, -vision_data.error_y);
       else
           pid_reset_servo(&servo_y.pid); // 在死区内重置积分项
       
       // 关键修改：大幅缩小PID输出到舵机角度的映射比例
       x_correction *= 0.05f;  // 缩小20倍
       y_correction *= 0.05f;  // 缩小20倍
       
       // 计算新角度，考虑角度范围
       float new_x_angle = fmodf(servo_x.current_angle + x_correction, 360.0f);
       if (new_x_angle < 0) new_x_angle += 360.0f;
       
       // Y舵机使用0-180度范围
       float new_y_angle = servo_y.current_angle + y_correction;
       if (new_y_angle < 0) new_y_angle = 0;
       if (new_y_angle > 180) new_y_angle = 180;
       
       // 更新舵机角度 - 使用较长的过渡时间增加平滑性
       if (fabsf(x_correction) > 0.1f)
           Servo_SetXAngle(new_x_angle, 40);
       
       if (fabsf(y_correction) > 0.1f)
           Servo_SetYAngle(new_y_angle, 40);
   }

   // 更新记录的误差值（在所有处理完成后）
   last_error_x = vision_data.error_x;
   last_error_y = vision_data.error_y;
}

/**
 * @brief 重置舵机到中心位置
 */
void Servo_Reset(void)
{
    // 重置PID控制器
    pid_reset_servo(&servo_x.pid);
    pid_reset_servo(&servo_y.pid);

    // 舵机回到中心位置
    Servo_SetXAngle(SERVO_X_CENTER_ANGLE, SERVO_DEFAULT_DURATION);
    Servo_SetYAngle(SERVO_Y_CENTER_ANGLE, SERVO_DEFAULT_DURATION); // Y舵机中心点为90度
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
    case TIM_CHANNEL_3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
        break;
    case TIM_CHANNEL_4:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
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
    
    // 确保角度在0-360范围内
    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;
    
    // 角度到脉宽的转换 (0-360度 映射到 500-2500μs)
    pulse = (uint32_t)(SERVO_MIN_PULSE + (angle / 360.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    
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
 * @brief 角度转换为脉宽（180度舵机专用）
 * @param angle 角度(0-180度)
 * @param offset 偏移量(-100到100)
 * @return 脉宽值(μs)
 */
static uint32_t Servo_AngleToPulse180(float angle, int offset)
{
    uint32_t pulse;
    
    // 确保角度在0-180范围内
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
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

/**
 * @brief 计算两个角度之间的最短路径
 * @param current_angle 当前角度(0-360度)
 * @param target_angle 目标角度(0-360度)
 * @return 最短路径的目标角度
 */
static float Servo_ShortestPath(float current_angle, float target_angle)
{
    // 确保角度在0-360范围内
    current_angle = fmodf(current_angle, 360.0f);
    if (current_angle < 0) current_angle += 360.0f;
    
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle < 0) target_angle += 360.0f;
    
    //printf("计算最短路径: 当前=%.1f, 目标=%.1f\n", current_angle, target_angle);
    
    // 计算两个方向的距离
    float diff = target_angle - current_angle;
    
    // 标准化到-180到+180范围
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    float result = current_angle + diff;
    //printf("最短路径结果: %.1f (差值=%.1f)\n", result, diff);
    
    return result;
}

/**
 * @brief 360度舵机基本功能测试
 */
void Servo_Test360Degrees(void)
{
    printf("开始360度舵机测试\n");
    
    // 测试Y舵机
    printf("测试Y舵机 - 0度\n");
    Servo_SetYAngle(30, 1000);
    HAL_Delay(1500);
    
    printf("测试Y舵机 - 90度\n");
    Servo_SetYAngle(90, 1000);
    HAL_Delay(1500);
    
    printf("测试Y舵机 - 180度\n");
    Servo_SetYAngle(130, 1000);
    HAL_Delay(1500);
    
    // printf("测试Y舵机 - 270度\n");
    // Servo_SetYAngle(270, 1000);
    // HAL_Delay(1500);
    
    // printf("测试Y舵机 - 359度\n");
    // Servo_SetYAngle(359, 1000);
    // HAL_Delay(1500);
    
    // printf("测试Y舵机 - 1度 (测试最短路径)\n");
    // Servo_SetYAngle(1, 1000);
    // HAL_Delay(1500);
    
    // printf("测试Y舵机 - 返回中心\n");
    // Servo_SetYAngle(180, 1000);
    // HAL_Delay(1500);
    
    // 对Y舵机进行类似测试...
    
    printf("360度舵机测试完成\n");
}

/**
 * @brief 舵机基本功能测试
 */
void Servo_TestBasic(void)
{
    printf("开始舵机基本功能测试\n");
    
    // 测试X舵机（0-360度）
    printf("测试X舵机 - 0度\n");
    Servo_SetXAngle(0, 1000);
    HAL_Delay(1500);
    
    printf("测试X舵机 - 180度\n");
    Servo_SetXAngle(180, 1000);
    HAL_Delay(1500);
    
    printf("测试X舵机 - 359度\n");
    Servo_SetXAngle(359, 1000);
    HAL_Delay(1500);
    
    // 测试Y舵机（0-180度）
    printf("测试Y舵机 - 0度\n");
    Servo_SetYAngle(0, 1000);
    HAL_Delay(1500);
    
    printf("测试Y舵机 - 90度\n");
    Servo_SetYAngle(90, 1000);
    HAL_Delay(1500);
    
    printf("测试Y舵机 - 180度\n");
    Servo_SetYAngle(130, 1000);
    HAL_Delay(1500);
    
    printf("舵机基本功能测试完成\n");
}
