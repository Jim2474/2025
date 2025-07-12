#ifndef M_SERVO_H
#define M_SERVO_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "board.h"
#include "M_pid.h"  // 确保M_pid.h被正确包含

/* 舵机通道定义 */
#define SERVO_X_CHANNEL TIM_CHANNEL_3
#define SERVO_Y_CHANNEL TIM_CHANNEL_4

/* 舵机角度限制 */
#define SERVO_X_MIN_ANGLE 0.0f
#define SERVO_X_MAX_ANGLE 180.0f
#define SERVO_X_CENTER_ANGLE 90.0f

#define SERVO_Y_MIN_ANGLE 0.0f
#define SERVO_Y_MAX_ANGLE 180.0f
#define SERVO_Y_CENTER_ANGLE 90.0f

/* PID控制参数 */
#define SERVO_PID_KP 0.5f
#define SERVO_PID_KI 0.01f
#define SERVO_PID_KD 0.1f
#define SERVO_PID_MAX_OUT 10.0f
#define SERVO_PID_MIN_OUT -10.0f


/* 外部变量声明 */
extern TIM_HandleTypeDef htim2;
typedef struct 
{
    float kp;
    float ki;
    float kd;
    float set;
    float actual;
    float err;
    float err_last;
    float err_sum;
    float out;
    float out_max;
    float out_min;
} PID_TypeDef1;

/* 舵机控制结构体 */
typedef struct 
{
    PID_TypeDef1 pid;               // PID控制器
    float current_angle;     // 当前角度
    float target_angle;      // 目标角度
    int offset;              // 舵机偏移量(-100到100)
    uint8_t is_running;      // 舵机运动标志
    uint32_t duration;       // 运动时间(ms)
    uint32_t inc_times;      // 递增次数
    float angle_inc;         // 角度增量
} ServoControl_t;

/* 视觉反馈数据结构体 */
typedef struct {
    float error_x;           // X方向误差
    float error_y;           // Y方向误差
    uint8_t target_detected; // 目标检测标志
    uint32_t last_update;    // 最后更新时间
} VisionData_t;
 extern VisionData_t vision_data; // 视觉反馈数据

/* 函数声明 */
void Servo_Init(void);
void Servo_SetXAngle(float angle, uint32_t duration);
void Servo_SetYAngle(float angle, uint32_t duration);
void Servo_SetXOffset(int offset);
void Servo_SetYOffset(int offset);
float Servo_GetXAngle(void);
float Servo_GetYAngle(void);
void Servo_ProcessVisionData(float error_x, float error_y, uint8_t target_detected);
void Servo_Update(void);
void Servo_Reset(void);

#endif /* M_SERVO_H */