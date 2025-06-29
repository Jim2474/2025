#include "M_navy.h"
#include "jy61p.h"
#include "Encoder.h"

// 全局位置变量定义
Position_t currentPosition = {0.0f, 0.0f, 0.0f};  // 当前位置，初始为原点
Position_t targetPosition = {0.0f, 0.0f};         // 目标位置，初始为原点

// 内部变量
static float lastLeftWheelDist = 0.0f;   // 上次左轮累计行驶距离
static float lastRightWheelDist = 0.0f;  // 上次右轮累计行驶距离
static float totalLeftDist = 0.0f;       // 左轮总行驶距离
static float totalRightDist = 0.0f;      // 右轮总行驶距离

/**
 * @brief 初始化导航系统
 */
void navy_init(void)
{
    // 重置位置到原点
    resetPosition();
    
    // 可以在这里添加其他初始化操作，如传感器校准等
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
 * @brief 更新当前位置（在TIM2_Task_100Hz中调用）
 * 使用编码器数据和陀螺仪数据更新位置
 */
void updatePosition(void)
{
    // 1. 计算左右轮的行驶距离增量（单位：dm）
    float leftWheelDist = (totalLeftDist + (left_wheel_speed * POSITION_UPDATE_INTERVAL)) / 10.0f;  // cm转换为dm
    float rightWheelDist = (totalRightDist + (right_wheel_speed * POSITION_UPDATE_INTERVAL)) / 10.0f;
    
    float deltaLeftDist = leftWheelDist - lastLeftWheelDist;
    float deltaRightDist = rightWheelDist - lastRightWheelDist;
    
    // 更新累计距离
    lastLeftWheelDist = leftWheelDist;
    lastRightWheelDist = rightWheelDist;
    totalLeftDist += left_wheel_speed * POSITION_UPDATE_INTERVAL;  // 累加当前速度对应的距离
    totalRightDist += right_wheel_speed * POSITION_UPDATE_INTERVAL;
    
    // 2. 计算中心点行驶距离和角度变化
    float deltaDist = (deltaLeftDist + deltaRightDist) / 2.0f;  // 中心点行驶距离
    
    // 3. 使用陀螺仪数据更新航向角（更准确）
    currentPosition.theta = deg2rad(IMU_data.YawZ);  // 陀螺仪数据是角度，转换为弧度
    
    // 4. 使用航向角和行驶距离更新位置坐标
    currentPosition.x += deltaDist * cosf(currentPosition.theta);
    currentPosition.y += deltaDist * sinf(currentPosition.theta);
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
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
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