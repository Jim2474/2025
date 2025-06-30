#ifndef __M_NAVY_H
#define __M_NAVY_H

#include "board.h"
#include "math.h"

// 数学常量
#define PI 3.14159265358979323846f

// 坐标系统参数
#define WHEEL_BASE 14.0f    // 轮距，单位：cm
#define POSITION_UPDATE_INTERVAL 0.01f  // 位置更新间隔，100Hz

// 位置结构体
typedef struct {
    float x;        // X坐标，单位：dm
    float y;        // Y坐标，单位：dm
    float theta;    // 航向角，单位：弧度
} Position_t;

// 导航状态枚举
typedef enum {
    NAVY_STATE_IDLE,       // 空闲状态
    NAVY_STATE_MOVING,     // 移动状态
    NAVY_STATE_ARRIVED     // 已到达目标
} NavyState_t;

// 全局位置变量
extern Position_t currentPosition;  // 当前位置
extern Position_t targetPosition;   // 目标位置

// 初始化导航系统
void navy_init(void);

// 重置位置到原点
void resetPosition(void);

// 设置当前位置
void setCurrentPosition(float x, float y, float theta);

// 设置目标位置
void setTargetPosition(float x, float y);

// 获取当前位置
Position_t getCurrentPosition(void);

// 获取目标位置
Position_t getTargetPosition(void);

// 更新当前位置（在TIM2_Task_100Hz中调用）
void updatePosition(void);

// 计算两点间距离
float calculateDistance(Position_t pos1, Position_t pos2);

// 角度归一化到[-PI, PI]
float normalizeAngle(float angle);

// 弧度转换为角度
float rad2deg(float rad);

// 角度转换为弧度
float deg2rad(float deg);

// 计算目标点相对于当前位置的方向角
float calculateTargetAngle(void);

// 开始导航到目标点
uint8_t startNavigation(float x, float y);

// 停止当前导航
void stopNavigation(void);

// 获取当前导航状态
NavyState_t getNavigationState(void);

// 设置导航参数
void setNavigationParameters(float distThreshold, float velocity, float angVelocity);

// 更新导航控制
void updateNavigation(void);

#endif