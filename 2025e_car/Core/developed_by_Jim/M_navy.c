#include "M_navy.h"


// 全局位置变量定义
Position_t currentPosition = {0.0f, 0.0f, 0.0f};  // 当前位置，初始为原点
Position_t targetPosition = {0.0f, 0.0f};         // 目标位置，初始为原点

// 内部变量
static float lastLeftWheelDist = 0.0f;   // 上次左轮累计行驶距离
static float lastRightWheelDist = 0.0f;  // 上次右轮累计行驶距离
static float totalLeftDist = 0.0f;       // 左轮总行驶距离
static float totalRightDist = 0.0f;      // 右轮总行驶距离

static NavyState_t navyState = NAVY_STATE_IDLE;    // 当前导航状态
static float targetDistanceThreshold = 0.5f;       // 到达目标的距离阈值(dm)
static float targetVelocity = 10.0f;               // 导航时的目标速度(cm/s)
static float maxAngularVelocity = 60.0f;           // 最大角速度(度/秒)

/**
 * @brief 初始化导航系统
 */
void navy_init(void)
{
    // 重置位置到原点
    resetPosition();
    
    // 可以在这里添加其他初始化操作，如传感器校准等
    navyState = NAVY_STATE_IDLE;
}

/**
 * @brief 重置位置到原点
 */
void resetPosition(void)
{
    currentPosition.x = 0.0f;
    currentPosition.y = 0.0f;
    currentPosition.theta = deg2rad(IMU_data.YawZ);  // 使用当前陀螺仪读数作为初始方向
    
    // 重置累计距离变量
    totalLeftDist = 0.0f;
    totalRightDist = 0.0f;
    lastLeftWheelDist = 0.0f; 
    lastRightWheelDist = 0.0f;
}

/**
 * @brief 设置当前位置
 * @param x X坐标，单位：dm
 * @param y Y坐标，单位：dm
 * @param theta 航向角，单位：弧度
 */
void setCurrentPosition(float x, float y, float theta)
{
    currentPosition.x = x;
    currentPosition.y = y;
    currentPosition.theta = theta;
}

/**
 * @brief 设置目标位置
 * @param x X坐标，单位：dm
 * @param y Y坐标，单位：dm
 */
void setTargetPosition(float x, float y)
{
    targetPosition.x = x;
    targetPosition.y = y;
}

/**
 * @brief 获取当前位置
 * @return 当前位置结构体
 */
Position_t getCurrentPosition(void)
{
    return currentPosition;
}

/**
 * @brief 获取目标位置
 * @return 目标位置结构体
 */
Position_t getTargetPosition(void)
{
    return targetPosition;
}

/**
 * @brief 更新当前位置（在TIM2_Task_100Hz中调用）#####################################################################
 * ################################################################################################################
 * 使用编码器数据和陀螺仪数据更新位置
 */
void updatePosition(void)
{
    // 1. 计算左右轮的行驶距离增量（单位：dm）
    // 添加校准系数0.92来修正距离误差(60cm实际走了65cm，比例约为60/65=0.92)
    float calibrationFactor = 0.92f;
    
    float leftWheelDist = (totalLeftDist + (left_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor)) / 10.0f;  // cm转换为dm
    float rightWheelDist = (totalRightDist + (right_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor)) / 10.0f;
    
    float delta_Left_Dist = leftWheelDist - lastLeftWheelDist;
    float delta_Right_Dist = rightWheelDist - lastRightWheelDist;
    
    // 更新累计距离
    lastLeftWheelDist = leftWheelDist;
    lastRightWheelDist = rightWheelDist;
    totalLeftDist += left_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor;  // 累加当前速度对应的距离
    totalRightDist += right_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor;
    
    // 2. 计算中心点行驶距离和角度变化
    float delta_Dist = (delta_Left_Dist + delta_Right_Dist) / 2.0f;  // 中心点行驶距离
    
    // 3. 使用陀螺仪数据更新航向角（更准确）
    currentPosition.theta = deg2rad(IMU_data.YawZ);  // 陀螺仪数据是角度，转换为弧度
    
    // 4. 使用航向角和行驶距离更新位置坐标
    currentPosition.x += delta_Dist * cosf(currentPosition.theta);
    currentPosition.y += delta_Dist * sinf(currentPosition.theta);
    //printf("Position: (%.2f, %.2f), Theta: %.2f\n", 
          // currentPosition.x*10, currentPosition.y*10, rad2deg(currentPosition.theta));
}

/**
 * @brief 计算两点间距离
 * @param pos1 位置1
 * @param pos2 位置2
 * @return 距离，单位：dm
 */
float calculateDistance(Position_t pos1, Position_t pos2)
{
    float dx = pos2.x - pos1.x;
    float dy = pos2.y - pos1.y;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief 角度归一化到[-PI, PI]
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度（弧度）
 */
float normalizeAngle(float angle)
{
    while (angle > PI) 
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
     {
        angle += 2.0f * PI;
    }
    return angle;
}

/**
 * @brief 弧度转换为角度
 * @param rad 弧度值
 * @return 角度值
 */
float rad2deg(float rad)
{
    return rad * 180.0f / PI;
}

/**
 * @brief 角度转换为弧度
 * @param deg 角度值
 * @return 弧度值
 */
float deg2rad(float deg)
{
    return deg * PI / 180.0f;
}

/**
 * @brief 计算目标点相对于当前位置的方向角
 * @return 方向角，单位：弧度，范围[-PI, PI]
 */
float calculateTargetAngle(void)
{
    float dx = targetPosition.x - currentPosition.x;
    float dy = targetPosition.y - currentPosition.y;
    return normalizeAngle(atan2f(dy, dx));
}

/**
 * @brief 开始导航到目标点
 * @param x 目标点X坐标，单位：dm
 * @param y 目标点Y坐标，单位：dm
 * @return 1表示成功开始导航，0表示当前已在导航中或参数无效
 */
uint8_t startNavigation(float x, float y)
{
    // 检查是否已在导航状态
    if (navyState == NAVY_STATE_MOVING)
        return 0;
    
    // 设置目标点
    setTargetPosition(x, y);
    
    // 如果目标点与当前点距离小于阈值，认为已到达
    if (calculateDistance(currentPosition, targetPosition) < targetDistanceThreshold)
    {
        navyState = NAVY_STATE_ARRIVED;
        return 0;
    }
    
    // 开始导航
    navyState = NAVY_STATE_MOVING;
    return 1;
}

/**
 * @brief 停止当前导航
 */
void stopNavigation(void)
{
    navyState = NAVY_STATE_IDLE;
    set_target_speed(0.0f, 0.0f); // 停止小车
}

/**
 * @brief 获取当前导航状态
 * @return 导航状态枚举值
 */
NavyState_t getNavigationState(void)
{
    return navyState;
}

/**
 * @brief 设置导航参数
 * @param distThreshold 到达目标的距离阈值(dm)
 * @param velocity 导航时的目标线速度(cm/s)
 * @param angVelocity 最大角速度(度/秒)
 */
void setNavigationParameters(float distThreshold, float velocity, float angVelocity)
{
    targetDistanceThreshold = distThreshold;
    targetVelocity = velocity;
    
    maxAngularVelocity = angVelocity;
}

/**
 * @brief 更新导航控制######################################################################################################################################
 * 在TIM2_Task_100Hz中周期性调用，根据当前位置和目标位置计算控制量
 */
void updateNavigation_control(void)
{
    // 如果不在导航状态，直接返回
    if (navyState != NAVY_STATE_MOVING)
        return;
    
    // 计算到目标点的距离
    float distance = calculateDistance(currentPosition, targetPosition);
    
    // 检查是否已到达目标
    if (distance < targetDistanceThreshold)
    {
        navyState = NAVY_STATE_ARRIVED;
        set_target_speed(0.0f, 0.0f); // 停止小车
        return;
    }
    
    // 计算目标角度和当前角度的差值
    float targetAngle = calculateTargetAngle();
    float angleDiff = normalizeAngle(targetAngle - currentPosition.theta);
    
    // 根据角度差计算转向控制量（角度制）
    float angularControl = rad2deg(angleDiff);
    
    // 限制角速度
    if (angularControl > maxAngularVelocity)
        angularControl = maxAngularVelocity;
    else if (angularControl < -maxAngularVelocity)
        angularControl = -maxAngularVelocity;
    
    // 计算线速度控制量 - 根据距离和角度差调整速度
    float velocityControl = targetVelocity * (1.0f - fabsf(angleDiff) / PI);
    
    // 确保最小速度
    if (velocityControl < targetVelocity * 0.3f)
        velocityControl = targetVelocity * 0.3f;
    
    // 如果接近目标点，减速
    if (distance < 1.0f)
    {
        velocityControl *= (distance / 3.0f);
        if (velocityControl < 5.0f) // 最小速度限制
            velocityControl = 5.0f;
    }
    
    // 设置左右轮目标速度
    // 这里需要根据你的控制算法转换角速度和线速度为左右轮速度
    // 简单实现：根据角度差调整左右轮速度差异
    float leftSpeed = velocityControl - angularControl * 0.5f;
    float rightSpeed = velocityControl + angularControl * 0.5f;
    
    // 设置电机速度
    set_target_speed(leftSpeed, rightSpeed);
}