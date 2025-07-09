#include "Mission.h"

// 定义地图尺寸
#define MAP_WIDTH 48.0f  // 地图宽度，单位：dm
#define MAP_HEIGHT 40.0f // 地图高度，单位：dm

// 定义起点区域（红色区域）
#define START_X 2.5f     // 起点X坐标中心点，单位：dm
#define START_Y 3.5f     // 起点Y坐标中心点，单位：dm

// 定义建筑物1（左上角）火源位置
#define FIRE1_X 12.0f    // 火源1 X坐标，单位：dm
#define FIRE1_Y 26.0f    // 火源1 Y坐标，单位：dm

// 定义建筑物2（中上部）火源位置
#define FIRE2_X 24.0f    // 火源2 X坐标，单位：dm
#define FIRE2_Y 26.0f    // 火源2 Y坐标，单位：dm

// 定义建筑物6（右下部）火源位置
#define FIRE6_X 36.0f    // 火源6 X坐标，单位：dm
#define FIRE6_Y 14.0f    // 火源6 Y坐标，单位：dm

// 定义导航参数
#define ARRIVE_THRESHOLD 0.5f    // 到达目标点的阈值，单位：dm
#define FIRE_DISTANCE 5.0f       // 灭火距离，单位：dm
#define NORMAL_SPEED 20.0f       // 正常行驶速度
#define TURNING_SPEED 10.0f      // 转弯速度
#define APPROACH_SPEED 15.0f     // 接近目标点速度

// 定义超时参数
#define MISSION_TIMEOUT 20000    // 任务超时时间，单位：ms

// 路径点结构体
typedef struct {
    float x;        // X坐标，单位：dm
    float y;        // Y坐标，单位：dm
    float speed;    // 该点的推荐速度
} PathPoint_t;

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

// 全局变量
static MissionState_t currentMissionState = MISSION_IDLE;
static uint8_t currentPathIndex = 0;
static uint8_t totalPathPoints = 0;
static PathPoint_t *currentPath = NULL;
static uint32_t missionStartTime = 0;

// 火源1路径点（前往）- 从起点到左上角火源，沿左边界走
static PathPoint_t pathToFire1[] = {
    {START_X, START_Y, NORMAL_SPEED},      // 起点
    {5.0f, 5.0f, NORMAL_SPEED},            // 避开左下角建筑物
    {7.0f, 10.0f, TURNING_SPEED},          // 左转准备沿左边界
    {7.0f, 20.0f, NORMAL_SPEED},           // 沿左边界向上
    {7.0f, 26.0f, NORMAL_SPEED},           // 继续向上到火源1高度
    {FIRE1_X - FIRE_DISTANCE, FIRE1_Y, APPROACH_SPEED} // 接近火源1位置，保持5dm距离
};

// 火源1路径点（返回）- 原路返回
static PathPoint_t pathFromFire1[] = {
    {FIRE1_X - FIRE_DISTANCE, FIRE1_Y, NORMAL_SPEED}, // 火源1位置
    {7.0f, 26.0f, NORMAL_SPEED},           // 回到左边界
    {7.0f, 20.0f, NORMAL_SPEED},           // 沿左边界向下
    {7.0f, 10.0f, NORMAL_SPEED},           // 继续向下
    {5.0f, 5.0f, TURNING_SPEED},           // 避开左下角建筑物
    {START_X, START_Y, APPROACH_SPEED}     // 返回起点
};

// 火源2路径点（前往）- 从起点到中上部火源，走中间通道
static PathPoint_t pathToFire2[] = {
    {START_X, START_Y, NORMAL_SPEED},      // 起点
    {10.0f, 5.0f, NORMAL_SPEED},           // 向右移动
    {15.0f, 5.0f, NORMAL_SPEED},           // 继续向右移动
    {20.0f, 5.0f, NORMAL_SPEED},           // 到达中间通道入口
    {20.0f, 15.0f, NORMAL_SPEED},          // 沿中间通道向上
    {20.0f, 26.0f, NORMAL_SPEED},          // 继续向上到火源2高度
    {FIRE2_X - FIRE_DISTANCE, FIRE2_Y, APPROACH_SPEED} // 接近火源2位置，保持5dm距离
};

// 火源2路径点（返回）- 原路返回
static PathPoint_t pathFromFire2[] = {
    {FIRE2_X - FIRE_DISTANCE, FIRE2_Y, NORMAL_SPEED}, // 火源2位置
    {20.0f, 26.0f, NORMAL_SPEED},          // 回到中间通道
    {20.0f, 15.0f, NORMAL_SPEED},          // 沿中间通道向下
    {20.0f, 5.0f, NORMAL_SPEED},           // 到达中间通道出口
    {15.0f, 5.0f, NORMAL_SPEED},           // 向左移动
    {10.0f, 5.0f, NORMAL_SPEED},           // 继续向左移动
    {START_X, START_Y, APPROACH_SPEED}     // 返回起点
};

// 火源6路径点（前往）- 从起点到右下部火源，走右侧通道
static PathPoint_t pathToFire6[] = {
    {START_X, START_Y, NORMAL_SPEED},      // 起点
    {10.0f, 5.0f, NORMAL_SPEED},           // 向右移动
    {20.0f, 5.0f, NORMAL_SPEED},           // 继续向右移动
    {30.0f, 5.0f, NORMAL_SPEED},           // 继续向右移动
    {36.0f, 5.0f, NORMAL_SPEED},           // 到达右侧通道入口
    {36.0f, 10.0f, NORMAL_SPEED},          // 沿右侧通道向上
    {FIRE6_X, FIRE6_Y - FIRE_DISTANCE, APPROACH_SPEED} // 接近火源6位置，保持5dm距离
};

// 火源6路径点（返回）- 原路返回
static PathPoint_t pathFromFire6[] = {
    {FIRE6_X, FIRE6_Y - FIRE_DISTANCE, NORMAL_SPEED}, // 火源6位置
    {36.0f, 10.0f, NORMAL_SPEED},          // 回到右侧通道
    {36.0f, 5.0f, NORMAL_SPEED},           // 到达右侧通道出口
    {30.0f, 5.0f, NORMAL_SPEED},           // 向左移动
    {20.0f, 5.0f, NORMAL_SPEED},           // 继续向左移动
    {10.0f, 5.0f, NORMAL_SPEED},           // 继续向左移动
    {START_X, START_Y, APPROACH_SPEED}     // 返回起点
};

// 初始化任务
void Mission_Init(void)
{
    // 初始化导航系统
    navy_init();
    
    // 重置位置到起点
    setCurrentPosition(START_X, START_Y, 0.0f); // 假设初始朝向为0度（正东）
    
    // 设置导航参数
    setNavigationParameters(ARRIVE_THRESHOLD, NORMAL_SPEED, 30.0f);
    
    // 初始化任务状态
    currentMissionState = MISSION_IDLE;
    currentPathIndex = 0;
}

// 开始执行灭火任务1（左上角火源）
void Mission_StartFire1(void)
{
    if (currentMissionState == MISSION_IDLE) {
        currentMissionState = MISSION_FIRE1_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire1;
        totalPathPoints = sizeof(pathToFire1) / sizeof(PathPoint_t);
        
        // 设置第一个路径点
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        
        // 记录开始时间
        missionStartTime = HAL_GetTick();
    }
}

// 开始执行灭火任务2（中上部火源）
void Mission_StartFire2(void)
{
    if (currentMissionState == MISSION_IDLE) {
        currentMissionState = MISSION_FIRE2_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire2;
        totalPathPoints = sizeof(pathToFire2) / sizeof(PathPoint_t);
        
        // 设置第一个路径点
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        
        // 记录开始时间
        missionStartTime = HAL_GetTick();
    }
}

// 开始执行灭火任务6（右下部火源）
void Mission_StartFire6(void)
{
    if (currentMissionState == MISSION_IDLE)
     {
        currentMissionState = MISSION_FIRE6_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire6;
        totalPathPoints = sizeof(pathToFire6) / sizeof(PathPoint_t);
        
        // 设置第一个路径点
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        
        // 记录开始时间
        missionStartTime = HAL_GetTick();
    }
}

// 根据火源ID启动对应任务
void Mission_StartByFireId(uint8_t fireId)
{
    switch (fireId) 
    {
        case 1:
            Mission_StartFire1();
            break;
        case 2:
            Mission_StartFire2();
            break;
        case 6:
            Mission_StartFire6();
            break;
        default:
            // 无效的火源ID
            break;
    }
}

// 接收无人机发送的火源信息
void Mission_ReceiveFireInfo(float fire_x, float fire_y, uint8_t fire_id)
{
    // 根据接收到的火源信息启动相应任务
    Mission_StartByFireId(fire_id);
}

// 任务更新函数，在主循环中调用
void Mission_Update(void)
{
    NavyState_t navyState = getNavigationState();
    Position_t currentPos = getCurrentPosition();
    
    // 检查任务超时
    if (currentMissionState != MISSION_IDLE && currentMissionState != MISSION_COMPLETE) {
        if (HAL_GetTick() - missionStartTime > MISSION_TIMEOUT) {
            // 任务超时，停止当前任务
            Mission_Stop();
            return;
        }
    }
    
    switch (currentMissionState) {
        case MISSION_IDLE:
            // 空闲状态，等待命令
            break;
            
        case MISSION_FIRE1_GOING:
            // 前往火源1
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } else {
                    // 到达火源1附近，切换到灭火状态
                    currentMissionState = MISSION_FIRE1_FIGHTING;
                    
                    // 预留灭火代码位置
                    // TODO: 在此处添加灭火操作代码
                    // 1. PID精确定位控制
                    // 2. 舵机控制逻辑
                    // 3. 激光灭火操作
                    // 4. 灭火完成确认
                }
            }
            break;
            
        case MISSION_FIRE1_FIGHTING:
            // 灭火1状态
            // 预留灭火代码位置
            
            // 假设灭火完成，切换到返回状态
            currentMissionState = MISSION_FIRE1_RETURN;
            currentPathIndex = 0;
            currentPath = pathFromFire1;
            totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
            
            // 设置第一个返回路径点
            setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            break;
            
        case MISSION_FIRE1_RETURN:
            // 从火源1返回
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } else {
                    // 返回完成，任务结束
                    currentMissionState = MISSION_COMPLETE;
                    stopNavigation();
                }
            }
            break;
            
        case MISSION_FIRE2_GOING:
            // 前往火源2
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } else {
                    // 到达火源2附近，切换到灭火状态
                    currentMissionState = MISSION_FIRE2_FIGHTING;
                    
                    // 预留灭火代码位置
                    // TODO: 在此处添加灭火操作代码
                    // 1. PID精确定位控制
                    // 2. 舵机控制逻辑
                    // 3. 激光灭火操作
                    // 4. 灭火完成确认
                }
            }
            break;
            
        case MISSION_FIRE2_FIGHTING:
            // 灭火2状态
            // 预留灭火代码位置
            
            // 假设灭火完成，切换到返回状态
            currentMissionState = MISSION_FIRE2_RETURN;
            currentPathIndex = 0;
            currentPath = pathFromFire2;
            totalPathPoints = sizeof(pathFromFire2) / sizeof(PathPoint_t);
            
            // 设置第一个返回路径点
            setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            break;
            
        case MISSION_FIRE2_RETURN:
            // 从火源2返回
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } else {
                    // 返回完成，任务结束
                    currentMissionState = MISSION_COMPLETE;
                    stopNavigation();
                }
            }
            break;
            
        case MISSION_FIRE6_GOING:
            // 前往火源6
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } 
                else 
                {
                    // 到达火源6附近，切换到灭火状态
                    currentMissionState = MISSION_FIRE6_FIGHTING;
                    
                    // 预留灭火代码位置
                    // TODO: 在此处添加灭火操作代码
                    // 1. PID精确定位控制
                    // 2. 舵机控制逻辑
                    // 3. 激光灭火操作
                    // 4. 灭火完成确认
                }
            }
            break;
            
        case MISSION_FIRE6_FIGHTING:
            // 灭火6状态
            // 预留灭火代码位置
            
            // 假设灭火完成，切换到返回状态
            currentMissionState = MISSION_FIRE6_RETURN;
            currentPathIndex = 0;
            currentPath = pathFromFire6;
            totalPathPoints = sizeof(pathFromFire6) / sizeof(PathPoint_t);
            
            // 设置第一个返回路径点
            setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            break;
            
        case MISSION_FIRE6_RETURN:
            // 从火源6返回
            if (navyState == NAVY_STATE_ARRIVED) {
                // 到达当前路径点
                currentPathIndex++;
                
                if (currentPathIndex < totalPathPoints) {
                    // 前往下一个路径点
                    setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                } 
                else 
                {
                    // 返回完成，任务结束
                    currentMissionState = MISSION_COMPLETE;
                    stopNavigation();
                }
            }
            break;
            
        case MISSION_COMPLETE:
            // 任务完成
            break;
            
        default:
            break;
    }
}

// 获取当前任务状态
MissionState_t Mission_GetState(void)
{
    return currentMissionState;
}

// 停止当前任务
void Mission_Stop(void)
{
    stopNavigation();
    currentMissionState = MISSION_IDLE;
}

// 重置任务
void Mission_Reset(void)
{
    Mission_Stop();
    setCurrentPosition(START_X, START_Y, 0.0f);
}










