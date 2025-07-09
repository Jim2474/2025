#ifndef __M_SERVO_H__
#define __M_SERVO_H__

#include "board.h"
#include "M_pid.h"

/* 舵机配置参数 */
// 这些参数后续需要根据实际硬件进行修改
#define SERVO_X_CHANNEL      TIM_CHANNEL_1  // X舵机使用的定时器通道
#define SERVO_Y_CHANNEL      TIM_CHANNEL_2  // Y舵机使用的定时器通道

/* 舵机PWM参数 */
#define SERVO_PERIOD         20000       // PWM周期，单位：us (50Hz)
#define SERVO_MIN_PULSE      500         // 最小脉宽，对应0度，单位：us
#define SERVO_MAX_PULSE      2500        // 最大脉宽，对应180度，单位：us

/* 舵机角度限制 */
#define SERVO_X_MIN_ANGLE    0.0f        // X舵机最小角度
#define SERVO_X_MAX_ANGLE    180.0f      // X舵机最大角度
#define SERVO_X_CENTER_ANGLE 90.0f       // X舵机中心角度
#define SERVO_Y_MIN_ANGLE    0.0f        // Y舵机最小角度
#define SERVO_Y_MAX_ANGLE    180.0f      // Y舵机最大角度
#define SERVO_Y_CENTER_ANGLE 90.0f       // Y舵机中心角度

/* PID参数宏定义 */
#define SERVO_PID_KP         2.0f        // P参数
#define SERVO_PID_KI         0.1f        // I参数
#define SERVO_PID_KD         0.5f        // D参数
#define SERVO_PID_MAX_OUT    20.0f       // 最大输出限制(度)
#define SERVO_PID_MIN_OUT    -20.0f      // 最小输出限制(度)

/* 舵机控制结构体 */
typedef struct {
    float current_angle;     // 当前角度
    float target_angle;      // 目标角度
    PID_TypeDef pid;         // PID控制器
} ServoControl_t;

/* 视觉反馈数据结构体 */
typedef struct {
    float error_x;           // X方向误差
    float error_y;           // Y方向误差
    uint8_t target_detected; // 目标检测标志
    uint32_t last_update;    // 最后更新时间戳
} VisionData_t;

/* 函数声明 */
// 舵机控制初始化
void Servo_Init(void);

// 设置舵机角度(0-180度)
void Servo_SetXAngle(float angle);
void Servo_SetYAngle(float angle);

// 获取当前舵机角度
float Servo_GetXAngle(void);
float Servo_GetYAngle(void);

// 处理视觉反馈数据
void Servo_ProcessVisionData(float error_x, float error_y, uint8_t target_detected);

// 舵机控制周期性更新函数(建议在定时器中断中调用)
void Servo_Update(void);

// 重置舵机到中心位置
void Servo_Reset(void);

#endif /* __M_SERVO_H__ */