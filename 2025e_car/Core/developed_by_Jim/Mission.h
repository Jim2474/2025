#ifndef __MISSION_H
#define __MISSION_H

#include "board.h"

// 任务状态枚举
typedef enum {
    MISSION_IDLE,           // 空闲状态
    MISSION_FIRE1_GOING,    // 前往火源1
    MISSION_FIRE1_FIGHTING, // 灭火1
    MISSION_FIRE1_RETURN,   // 火源1返回
    MISSION_FIRE2_GOING,    // 前往火源2
    MISSION_FIRE2_FIGHTING, // 灭火2
    MISSION_FIRE2_RETURN,   // 火源2返回
    MISSION_FIRE6_GOING,    // 前往火源6
    MISSION_FIRE6_FIGHTING, // 灭火6
    MISSION_FIRE6_RETURN,   // 火源6返回
    MISSION_COMPLETE        // 任务完成
} MissionState_t;

// 灭火过程状态枚举
typedef enum {
    FIRE_PROCESS_POSITIONING,  // 精确定位阶段
    FIRE_PROCESS_AIMING,       // 瞄准火源阶段
    FIRE_PROCESS_FIRING,       // 激光灭火阶段
    FIRE_PROCESS_CONFIRMING,   // 确认灭火成功阶段
    FIRE_PROCESS_COMPLETED     // 灭火完成阶段
} FireProcessState_t;

// 初始化任务
void Mission_Init(void);

// 开始执行灭火任务1（左上角火源）
void Mission_StartFire1(void);

// 开始执行灭火任务2（中上部火源）
void Mission_StartFire2(void);

// 开始执行灭火任务6（右下部火源）
void Mission_StartFire6(void);

// 根据火源ID启动对应任务
void Mission_StartByFireId(uint8_t fireId);

// 接收无人机发送的火源信息
void Mission_ReceiveFireInfo(float fire_x, float fire_y, uint8_t fire_id);

// 任务更新函数，在主循环中调用
void Mission_Update(void);

// 获取当前任务状态
MissionState_t Mission_GetState(void);

// 停止当前任务
void Mission_Stop(void);

// 重置任务
void Mission_Reset(void);

// 处理视觉反馈数据
void Mission_ProcessVisionData(float error_x, float error_y, uint8_t target_detected);

// 获取当前灭火处理状态
FireProcessState_t Mission_GetFireProcessState(void);
void SimpleMissionTest();
            //测试函数 
            // void TestMissionStateMachine(void);
            // void TestMultipleFireMissions(void);
            // void TestVisionFeedbackProcessing(void);
            // void TestMissionInterruptAndRecovery(void);
            // void TestFireProcessing(void);
            // void TestCompleteFire1Mission(void);
            // void TestVisionFeedback(void);

#endif /* __MISSION_H */ 