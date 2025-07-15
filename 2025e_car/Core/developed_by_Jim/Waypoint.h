#ifndef __WAYPOINT_H
#define __WAYPOINT_H

#include "board.h"

// 最大航点数
#define MAX_WAYPOINTS 20
// 路径点结构体
typedef struct
{
    float x;     // X坐标，单位：dm
    float y;     // Y坐标，单位：dm
    float speed; // 该点的推荐速度
} PathPoint_t1;
// 航点队列结构
typedef struct {
    PathPoint_t1 points[MAX_WAYPOINTS];  // 航点数组
    uint8_t count;                      // 航点数量
    uint8_t current_index;              // 当前航点索引
    uint8_t is_patrolling;              // 是否正在巡航
    uint8_t is_loop;                    // 是否循环巡航
} WaypointQueue_t;

// 初始化航点队列
void Waypoint_Init(uint8_t is_loop);

// 添加航点到队列
uint8_t Waypoint_Add(float x, float y, float speed);

// 开始航点巡航
uint8_t Waypoint_StartPatrol(void);

// 停止航点巡航
void Waypoint_StopPatrol(void);

// 更新航点巡航状态，在主循环中调用
uint8_t Waypoint_Update(void);

// 获取当前巡航状态
uint8_t Waypoint_GetStatus(uint8_t *current_index, uint8_t *total_count);

void init_example(void);

#endif