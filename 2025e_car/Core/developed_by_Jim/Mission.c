#include "Mission.h"

// 定义地图尺寸
#define MAP_WIDTH 48.0f  // 地图宽度，单位：dm
#define MAP_HEIGHT 40.0f // 地图高度，单位：dm

// 定义起点区域（红色区域）
#define START_X 2.5f // 起点X坐标中心点，单位：dm
#define START_Y 3.5f // 起点Y坐标中心点，单位：dm

// 定义建筑物1（左上角）火源位置
#define FIRE1_X 12.0f // 火源1 X坐标，单位：dm
#define FIRE1_Y 26.0f // 火源1 Y坐标，单位：dm

// 定义建筑物2（中上部）火源位置
#define FIRE2_X 24.0f // 火源2 X坐标，单位：dm
#define FIRE2_Y 26.0f // 火源2 Y坐标，单位：dm

// 定义建筑物6（右下部）火源位置
#define FIRE6_X 36.0f // 火源6 X坐标，单位：dm
#define FIRE6_Y 14.0f // 火源6 Y坐标，单位：dm

// 定义导航参数
#define ARRIVE_THRESHOLD 0.5f // 到达目标点的阈值，单位：dm
#define FIRE_DISTANCE 5.0f    // 灭火距离，单位：dm
#define NORMAL_SPEED 20.0f    // 正常行驶速度
#define TURNING_SPEED 10.0f   // 转弯速度
#define APPROACH_SPEED 15.0f  // 接近目标点速度

// 定义超时参数
#define MISSION_TIMEOUT 20000       // 任务超时时间，单位：ms
#define FIRE_FIGHTING_TIMEOUT 10000 // 灭火超时时间，单位：ms

// 定义灭火相关参数
#define FIRE_SUCCESS_THRESHOLD 10      // 灭火成功所需稳定目标锁定次数
#define FIRE_POSITION_STABLE_TIME 3000 // 灭火位置稳定所需时间，单位：ms

// 路径点结构体
typedef struct
{
    float x;     // X坐标，单位：dm
    float y;     // Y坐标，单位：dm
    float speed; // 该点的推荐速度
} PathPoint_t;

// 任务状态枚举
typedef enum
{
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

// 灭火处理状态枚举
typedef enum
{
    FIRE_PROCESS_POSITIONING, // 精确定位阶段
    FIRE_PROCESS_AIMING,      // 瞄准火源阶段
    FIRE_PROCESS_FIRING,      // 激光灭火阶段
    FIRE_PROCESS_CONFIRMING,  // 确认灭火成功阶段
    FIRE_PROCESS_COMPLETED,   // 灭火完成阶段
} FireProcessState_t;

// 全局变量
static MissionState_t currentMissionState = MISSION_IDLE;
static uint8_t currentPathIndex = 0;
static uint8_t totalPathPoints = 0;
static PathPoint_t *currentPath = NULL;
static uint32_t missionStartTime = 0;
static uint32_t fireProcessStartTime = 0;                              // 灭火过程开始时间
static FireProcessState_t fireProcessState = FIRE_PROCESS_POSITIONING; // 灭火过程状态
static uint8_t fireSuccessCounter = 0;                                 // 灭火成功计数器
static uint8_t currentFireId = 0;                                      // 当前火源ID

// 火源1路径点（前往）- 从起点到左上角火源，沿左边界走
static PathPoint_t pathToFire1[] = {
    {START_X, START_Y, NORMAL_SPEED},                  // 起点
    {5.0f, 5.0f, NORMAL_SPEED},                        // 避开左下角建筑物
    {7.0f, 10.0f, TURNING_SPEED},                      // 左转准备沿左边界
    {7.0f, 20.0f, NORMAL_SPEED},                       // 沿左边界向上
    {7.0f, 26.0f, NORMAL_SPEED},                       // 继续向上到火源1高度
    {FIRE1_X - FIRE_DISTANCE, FIRE1_Y, APPROACH_SPEED} // 接近火源1位置，保持5dm距离
};

// 火源1路径点（返回）- 原路返回
static PathPoint_t pathFromFire1[] = {
    {FIRE1_X - FIRE_DISTANCE, FIRE1_Y, NORMAL_SPEED}, // 火源1位置
    {7.0f, 26.0f, NORMAL_SPEED},                      // 回到左边界
    {7.0f, 20.0f, NORMAL_SPEED},                      // 沿左边界向下
    {7.0f, 10.0f, NORMAL_SPEED},                      // 继续向下
    {5.0f, 5.0f, TURNING_SPEED},                      // 避开左下角建筑物
    {START_X, START_Y, APPROACH_SPEED}                // 返回起点
};

// 火源2路径点（前往）- 从起点到中上部火源，走中间通道
static PathPoint_t pathToFire2[] = {
    {START_X, START_Y, NORMAL_SPEED},                  // 起点
    {10.0f, 5.0f, NORMAL_SPEED},                       // 向右移动
    {15.0f, 5.0f, NORMAL_SPEED},                       // 继续向右移动
    {20.0f, 5.0f, NORMAL_SPEED},                       // 到达中间通道入口
    {20.0f, 15.0f, NORMAL_SPEED},                      // 沿中间通道向上
    {20.0f, 26.0f, NORMAL_SPEED},                      // 继续向上到火源2高度
    {FIRE2_X - FIRE_DISTANCE, FIRE2_Y, APPROACH_SPEED} // 接近火源2位置，保持5dm距离
};

// 火源2路径点（返回）- 原路返回
static PathPoint_t pathFromFire2[] = {
    {FIRE2_X - FIRE_DISTANCE, FIRE2_Y, NORMAL_SPEED}, // 火源2位置
    {20.0f, 26.0f, NORMAL_SPEED},                     // 回到中间通道
    {20.0f, 15.0f, NORMAL_SPEED},                     // 沿中间通道向下
    {20.0f, 5.0f, NORMAL_SPEED},                      // 到达中间通道出口
    {15.0f, 5.0f, NORMAL_SPEED},                      // 向左移动
    {10.0f, 5.0f, NORMAL_SPEED},                      // 继续向左移动
    {START_X, START_Y, APPROACH_SPEED}                // 返回起点
};

// 火源6路径点（前往）- 从起点到右下部火源，走右侧通道
static PathPoint_t pathToFire6[] = {
    {START_X, START_Y, NORMAL_SPEED},                  // 起点
    {10.0f, 5.0f, NORMAL_SPEED},                       // 向右移动
    {20.0f, 5.0f, NORMAL_SPEED},                       // 继续向右移动
    {30.0f, 5.0f, NORMAL_SPEED},                       // 继续向右移动
    {36.0f, 5.0f, NORMAL_SPEED},                       // 到达右侧通道入口
    {36.0f, 10.0f, NORMAL_SPEED},                      // 沿右侧通道向上
    {FIRE6_X, FIRE6_Y - FIRE_DISTANCE, APPROACH_SPEED} // 接近火源6位置，保持5dm距离
};

// 火源6路径点（返回）- 原路返回
static PathPoint_t pathFromFire6[] = {
    {FIRE6_X, FIRE6_Y - FIRE_DISTANCE, NORMAL_SPEED}, // 火源6位置
    {36.0f, 10.0f, NORMAL_SPEED},                     // 回到右侧通道
    {36.0f, 5.0f, NORMAL_SPEED},                      // 到达右侧通道出口
    {30.0f, 5.0f, NORMAL_SPEED},                      // 向左移动
    {20.0f, 5.0f, NORMAL_SPEED},                      // 继续向左移动
    {10.0f, 5.0f, NORMAL_SPEED},                      // 继续向左移动
    {START_X, START_Y, APPROACH_SPEED}                // 返回起点
};

// 声明私有函数
static void Mission_HandleFireFighting(void);
static void Mission_StartFireProcessing(uint8_t fire_id);
static uint8_t Mission_IsFireExtinguished(void);

// 初始化任务
void Mission_Init(void)
{
    // 初始化导航系统
    navy_init();

    // 初始化舵机控制系统
    Servo_Init();

    // 重置位置到起点
    setCurrentPosition(START_X, START_Y, 0.0f); // 假设初始朝向为0度（正东）

    // 设置导航参数
    setNavigationParameters(ARRIVE_THRESHOLD, NORMAL_SPEED, 30.0f);

    // 初始化任务状态
    currentMissionState = MISSION_IDLE;
    currentPathIndex = 0;

    // 初始化灭火相关状态
    fireProcessState = FIRE_PROCESS_POSITIONING;
    fireSuccessCounter = 0;
}

// 开始执行灭火任务1（左上角火源）
void Mission_StartFire1(void)
{
    if (currentMissionState == MISSION_IDLE)
    {
        currentMissionState = MISSION_FIRE1_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire1;
        totalPathPoints = sizeof(pathToFire1) / sizeof(PathPoint_t);
        currentFireId = 1;

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
    if (currentMissionState == MISSION_IDLE)
    {
        currentMissionState = MISSION_FIRE2_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire2;
        totalPathPoints = sizeof(pathToFire2) / sizeof(PathPoint_t);
        currentFireId = 2;

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
        currentFireId = 6;

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

// 开始灭火处理过程
static void Mission_StartFireProcessing(uint8_t fire_id)
{
    // 设置灭火过程状态为定位阶段
    fireProcessState = FIRE_PROCESS_POSITIONING;

    // 记录灭火过程开始时间
    fireProcessStartTime = HAL_GetTick();

    // 重置灭火成功计数器
    fireSuccessCounter = 0;

    // 记录当前处理的火源ID
    currentFireId = fire_id;

    // 初始化舵机到中心位置
    Servo_Reset();
}

// 处理灭火任务
static void Mission_HandleFireFighting(void)
{
    uint32_t current_time = HAL_GetTick();
    static uint32_t position_stable_time = 0;
    static uint8_t is_position_stable = 0;
    NavyState_t navyState = getNavigationState();

    // 检查灭火超时
    if (current_time - fireProcessStartTime > FIRE_FIGHTING_TIMEOUT)
    {
        // 灭火超时，放弃当前灭火任务，进入返回状态
        switch (currentFireId)
        {
        case 1:
            currentMissionState = MISSION_FIRE1_RETURN;
            currentPath = pathFromFire1;
            totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
            break;
        case 2:
            currentMissionState = MISSION_FIRE2_RETURN;
            currentPath = pathFromFire2;
            totalPathPoints = sizeof(pathFromFire2) / sizeof(PathPoint_t);
            break;
        case 6:
            currentMissionState = MISSION_FIRE6_RETURN;
            currentPath = pathFromFire6;
            totalPathPoints = sizeof(pathFromFire6) / sizeof(PathPoint_t);
            break;
        }

        // 停止舵机
        Servo_Reset();

        // 重置路径索引，开始返回
        currentPathIndex = 0;
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);

        return;
    }

    // 根据灭火过程状态进行处理
    switch (fireProcessState)
    {
    case FIRE_PROCESS_POSITIONING:
        // 精确定位阶段，确保小车在合适的位置
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 已经到达目标位置，检查位置稳定性
            if (!is_position_stable)
            {
                position_stable_time = current_time;
                is_position_stable = 1;
            }

            // 检查位置是否已经稳定一段时间
            if (current_time - position_stable_time > FIRE_POSITION_STABLE_TIME)
            {
                // 位置已稳定，进入瞄准阶段
                fireProcessState = FIRE_PROCESS_AIMING;

                // 重置稳定标志
                is_position_stable = 0;
            }
        }
        else
        {
            // 还未到达目标位置，重置稳定标志
            is_position_stable = 0;
        }
        break;

    case FIRE_PROCESS_AIMING:
        // 瞄准火源阶段，等待视觉反馈确认目标锁定
        if (Servo_GetXAngle() != 0 && Servo_GetYAngle() != 0)
        {
            // 舵机已移动，进入激光灭火阶段
            fireProcessState = FIRE_PROCESS_FIRING;
        }
        break;

    case FIRE_PROCESS_FIRING:
        // 激光灭火阶段，保持激光照射火源
        // 通过视觉反馈确认是否已灭火
        if (Mission_IsFireExtinguished())
        {
            // 灭火成功，进入确认阶段
            fireProcessState = FIRE_PROCESS_CONFIRMING;
        }
        break;

    case FIRE_PROCESS_CONFIRMING:
        // 确认灭火成功阶段，再次检查火源是否已熄灭
        if (Mission_IsFireExtinguished())
        {
            // 再次确认灭火成功，完成灭火任务
            fireProcessState = FIRE_PROCESS_COMPLETED;

            // 停止舵机
            Servo_Reset();

            // 切换到返回状态
            switch (currentFireId)
            {
            case 1:
                currentMissionState = MISSION_FIRE1_RETURN;
                currentPath = pathFromFire1;
                totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
                break;
            case 2:
                currentMissionState = MISSION_FIRE2_RETURN;
                currentPath = pathFromFire2;
                totalPathPoints = sizeof(pathFromFire2) / sizeof(PathPoint_t);
                break;
            case 6:
                currentMissionState = MISSION_FIRE6_RETURN;
                currentPath = pathFromFire6;
                totalPathPoints = sizeof(pathFromFire6) / sizeof(PathPoint_t);
                break;
            }

            // 重置路径索引，开始返回
            currentPathIndex = 0;
            setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        }
        else
        {
            // 火源可能重新点燃，返回激光灭火阶段
            fireProcessState = FIRE_PROCESS_FIRING;
        }
        break;

    case FIRE_PROCESS_COMPLETED:
        // 灭火完成阶段，应该已经切换到返回状态
        break;
    }
}

// 检查火源是否已熄灭
static uint8_t Mission_IsFireExtinguished(void)
{
    // 此函数应该根据视觉反馈判断火源是否已熄灭
    // 在实际应用中，可能需要通过摄像头检测火源的亮度或颜色变化

    // 简化实现：如果舵机已经对准目标一段时间，认为灭火成功
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 每100ms检查一次
    if (current_time - last_check_time > 100)
    {
        last_check_time = current_time;

        // 检查舵机角度是否稳定
        if (fabs(Servo_GetXAngle() - SERVO_X_CENTER_ANGLE) < 20.0f &&
            fabs(Servo_GetYAngle() - SERVO_Y_CENTER_ANGLE) < 20.0f)
        {
            fireSuccessCounter++;

            // 如果连续多次检测到舵机稳定，则认为灭火成功
            if (fireSuccessCounter >= FIRE_SUCCESS_THRESHOLD)
            {
                return 1;
            }
        }
        else
        {
            // 舵机不稳定，重置计数器
            fireSuccessCounter = 0;
        }
    }

    return 0;
}

// 任务更新函数，在主循环中调用
void Mission_Update(void)
{
    NavyState_t navyState = getNavigationState();
    Position_t currentPos = getCurrentPosition();

    // 更新舵机控制
    Servo_Update();

    // 检查任务超时
    if (currentMissionState != MISSION_IDLE && currentMissionState != MISSION_COMPLETE)
    {
        if (HAL_GetTick() - missionStartTime > MISSION_TIMEOUT)
        {
            // 任务超时，中止任务
            Mission_Stop();
            return;
        }
    }

    // 根据当前任务状态进行处理
    switch (currentMissionState)
    {
    case MISSION_IDLE:
        // 空闲状态，不做处理
        break;

    case MISSION_FIRE1_GOING:
        // 前往火源1状态
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
                // 继续前往下一个路径点
                setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            }
            else
            {
                // 到达火源1附近，切换到灭火状态
                currentMissionState = MISSION_FIRE1_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(1);
            }
        }
        break;

    case MISSION_FIRE1_FIGHTING:
        // 灭火1状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE1_RETURN:
        // 火源1返回状态
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
                // 继续前往下一个路径点
                setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            }
            else
            {
                // 返回完成，任务结束
                currentMissionState = MISSION_COMPLETE;
            }
        }
        break;

    case MISSION_FIRE2_GOING:
        // 前往火源2状态
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
                // 继续前往下一个路径点
                setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            }
            else
            {
                // 到达火源2附近，切换到灭火状态
                currentMissionState = MISSION_FIRE2_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(2);
            }
        }
        break;

    case MISSION_FIRE2_FIGHTING:
        // 灭火2状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE2_RETURN:
        // 从火源2返回
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
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

    case MISSION_FIRE6_GOING:
        // 前往火源6状态
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
                // 继续前往下一个路径点
                setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
                startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            }
            else
            {
                // 到达火源6附近，切换到灭火状态
                currentMissionState = MISSION_FIRE6_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(6);
            }
        }
        break;

    case MISSION_FIRE6_FIGHTING:
        // 灭火6状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE6_RETURN:
        // 从火源6返回
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;

            if (currentPathIndex < totalPathPoints)
            {
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
        // 任务完成状态，不做处理
        break;

    default:
        // 未知状态，重置为空闲
        currentMissionState = MISSION_IDLE;
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
    Servo_Reset();
    currentMissionState = MISSION_IDLE;
}

// 重置任务
void Mission_Reset(void)
{
    Mission_Stop();
    Servo_Reset();
    setCurrentPosition(START_X, START_Y, 0.0f);
}

// 处理视觉反馈数据
void Mission_ProcessVisionData(float error_x, float error_y, uint8_t target_detected)
{
    // 如果当前正在执行灭火任务，则将视觉数据传递给舵机控制模块
    if (currentMissionState == MISSION_FIRE1_FIGHTING ||
        currentMissionState == MISSION_FIRE2_FIGHTING ||
        currentMissionState == MISSION_FIRE6_FIGHTING)
    {
        Servo_ProcessVisionData(error_x, error_y, target_detected);
    }
}

// 获取当前灭火处理状态
FireProcessState_t Mission_GetFireProcessState(void)
{
    return fireProcessState;
}
