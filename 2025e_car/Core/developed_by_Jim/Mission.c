

#include "Mission.h"

// 定义地图尺寸
#define MAP_WIDTH 48.0f  // 地图宽度，单位：dm
#define MAP_HEIGHT 40.0f // 地图高度，单位：dm

// 定义起点区域（红色区域）
#define START_X 0.0f // 起点X坐标中心点，单位：dm
#define START_Y 0.0f // 起点Y坐标中心点，单位：dm

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
#define ARRIVE_THRESHOLD 0.3f // 到达目标点的阈值，单位：dm
#define FIRE_DISTANCE 7.0f    // 灭火距离，单位：dm
#define NORMAL_SPEED 20.0f    // 正常行驶速度
#define TURNING_SPEED 10.0f   // 转弯速度
#define APPROACH_SPEED 15.0f  // 接近目标点速度

// 定义超时参数
#define MISSION_TIMEOUT 20000       // 任务超时时间，单位：ms
#define FIRE_FIGHTING_TIMEOUT 15000 // 灭火超时时间，单位：ms

// 定义灭火相关参数
#define FIRE_SUCCESS_THRESHOLD 5      // 灭火成功所需稳定目标锁定次数
#define FIRE_POSITION_STABLE_TIME 3000 // 灭火位置稳定所需时间，单位：ms

// 路径点结构体
typedef struct
{
    float x;     // X坐标，单位：dm
    float y;     // Y坐标，单位：dm
    float speed; // 该点的推荐速度
} PathPoint_t;



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


//测试用 
        // 添加导航错误处理
        static uint32_t last_nav_check_time = 0;
        static uint32_t nav_stuck_time = 0;
// 火源1路径点（前往）- 从起点到左上角火源，沿左边界走
static PathPoint_t pathToFire1[] = {
   // {START_X, START_Y, NORMAL_SPEED},                  // 起点

    {3.5f, 0.0f, NORMAL_SPEED},                        // 避开左下角建筑物
    {3.5f, 19.0f, TURNING_SPEED},                      // 左转准备沿左边界
    {0.0f, 19.0f, NORMAL_SPEED},                       // 沿左边界向上
    {0.0f, 20.0f, NORMAL_SPEED},                       // 沿左边界向上
    //{6.0f, 26.0f, NORMAL_SPEED},                       // 继续向上到火源1高度
    //     {START_X + 5.0f, START_Y, NORMAL_SPEED},           // 向右移动5dm
    // {START_X + 10.0f, START_Y, NORMAL_SPEED},          // 继续向右移动5dm

    //{FIRE1_X - FIRE_DISTANCE, FIRE1_Y, APPROACH_SPEED} // 接近火源1位置，保持5dm距离
};

// 火源1路径点（返回）- 原路返回
static PathPoint_t pathFromFire1[] = {
//   {FIRE1_X - FIRE_DISTANCE, FIRE1_Y, NORMAL_SPEED}, // 火源1位置
//    {7.0f, 26.0f, NORMAL_SPEED},                      // 回到左边界
//    {7.0f, 20.0f, NORMAL_SPEED},                      // 沿左边界向下
//    {7.0f, 10.0f, NORMAL_SPEED},                      // 继续向下
//    {5.0f, 5.0f, TURNING_SPEED},                      // 避开左下角建筑物
//    //     {START_X + 10.0f, START_Y, NORMAL_SPEED},          // 返回中间点
//    // {START_X + 5.0f, START_Y, NORMAL_SPEED},           // 继续返回
//    {START_X, START_Y, APPROACH_SPEED}                // 返回起点
    {0.0f, 20.0f, NORMAL_SPEED},                       // 沿左边界向上
    {0.0f, 19.0f, NORMAL_SPEED},                       // 沿左边界向上
    {3.5f, 19.0f, TURNING_SPEED},                      // 左转准备沿左边界
    {3.5f, 0.0f, NORMAL_SPEED},                        // 避开左下角建筑物
    {START_X, START_Y, APPROACH_SPEED}                 // 返回起点



};

// 火源2路径点（前往）- 从起点到中上部火源，走中间通道
static PathPoint_t pathToFire2[] = {
    {3.5f, 0.0f, NORMAL_SPEED},                        // 避开左下角建筑物
    {3.5f, 19.0f, TURNING_SPEED},              // 继续向右移动
    {13.0f, 19.0f, NORMAL_SPEED},                       // 到达中间通道入口
    {13.0f, 20.0f, NORMAL_SPEED},                      // 沿中间通道向上
};

// 火源2路径点（返回）- 原路返回
static PathPoint_t pathFromFire2[] = {
        {13.0f, 20.0f, NORMAL_SPEED},                      
        {13.0f, 19.0f, NORMAL_SPEED},                        
    {3.5f, 19.0f, TURNING_SPEED},              
    {3.5f, 0.0f, NORMAL_SPEED},                      
    {START_X, START_Y, APPROACH_SPEED}            
};
//火源3路径点
static PathPoint_t pathToFire3[] = {
    {3.5f, 0.0f, NORMAL_SPEED},                        
    {3.5f, 19.0f, TURNING_SPEED},              
    {33.0f, 19.0f, NORMAL_SPEED},                       
    {33.0f, 20.0f, NORMAL_SPEED},                      
};
// 火源3路径点（返回）- 原路返回
static PathPoint_t pathFromFire3[] = {       
    {33.0f, 20.0f, NORMAL_SPEED},                      
    {33.0f, 19.0f, NORMAL_SPEED},                        
    {3.5f, 19.0f, TURNING_SPEED},              
    {3.5f, 0.0f, NORMAL_SPEED},                      
    {START_X, START_Y, APPROACH_SPEED}            
};
//火源4路径点
static PathPoint_t pathToFire4[] = {
    {0.0f, 3.0f, NORMAL_SPEED},                       
    {-5.0f, 3.0f, TURNING_SPEED},              
    {-5.0f, 4.0f, NORMAL_SPEED},                       
};
// 火源4路径点（返回）- 原路返回
static PathPoint_t pathFromFire4[] = {
     {-5.0f, 4.0f, NORMAL_SPEED},                      
    {-5.0f, 3.0f, TURNING_SPEED},                        
    {0.0f, 3.0f, NORMAL_SPEED},                                   
    {START_X, START_Y, APPROACH_SPEED}            
};    
//火源5路径点
static PathPoint_t pathToFire5[] = {
    {3.5f, 0.0f, NORMAL_SPEED},                       
    {3.5f, 3.0f, TURNING_SPEED},              
    {12.5f, 3.0f, NORMAL_SPEED},                       
    {12.5f, 4.0f, NORMAL_SPEED}                     
};
// 火源5路径点（返回）- 原路返回
static PathPoint_t pathFromFire5[] = {
    {12.5f, 4.0f, NORMAL_SPEED},                      
    {12.5f, 3.0f, NORMAL_SPEED},                        
    {3.5f, 3.0f, TURNING_SPEED},              
    {3.5f, 0.0f, NORMAL_SPEED},                      
    {START_X, START_Y, APPROACH_SPEED}            
};
// 火源6路径点（前往）- 从起点到右下部火源，走右侧通道
static PathPoint_t pathToFire6[] = {
    {3.5f, 0.0f, NORMAL_SPEED},                       
    {3.5f, 3.0f, TURNING_SPEED},              
    {21.5f, 3.0f, NORMAL_SPEED},                       
    {21.0f, 6.0f, NORMAL_SPEED},    
        {22.0f, 6.0f, NORMAL_SPEED},    

};

// 火源6路径点（返回）- 原路返回
static PathPoint_t pathFromFire6[] = {
        {22.0f, 6.0f, NORMAL_SPEED},    
    {21.0f, 6.0f, NORMAL_SPEED},    
    {3.5f, 3.0f, TURNING_SPEED},              // 继续向右移动
    {3.5f, 0.0f, NORMAL_SPEED},                        // 避开左下角建筑物
    {START_X, START_Y, APPROACH_SPEED}         
};

// 声明私有函数
static void Mission_HandleFireFighting(void);
static void Mission_StartFireProcessing(uint8_t fire_id);
static uint8_t Mission_IsFireExtinguished(void);
uint8_t flag_jim =0;
// 初始化任务
void Mission_Init(void)
{
    // 初始化导航系统
    //navy_init();


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

    vision_data.target_detected=0;
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

// 开始执行灭火任务3
void Mission_StartFire3(void)
{
    if (currentMissionState == MISSION_IDLE)
    {
        currentMissionState = MISSION_FIRE3_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire3;
        totalPathPoints = sizeof(pathToFire3) / sizeof(PathPoint_t);
        currentFireId = 3;

        // 设置第一个路径点
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);

        // 记录开始时间
        missionStartTime = HAL_GetTick();
    }
}

// 开始执行灭火任务4
void Mission_StartFire4(void)
{
    if (currentMissionState == MISSION_IDLE)
    {
        currentMissionState = MISSION_FIRE4_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire4;
        totalPathPoints = sizeof(pathToFire4) / sizeof(PathPoint_t);
        currentFireId = 4;

        // 设置第一个路径点
        setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);

        // 记录开始时间
        missionStartTime = HAL_GetTick();
    }
}

// 开始执行灭火任务5
void Mission_StartFire5(void)
{
    if (currentMissionState == MISSION_IDLE)
    {
        currentMissionState = MISSION_FIRE5_GOING;
        currentPathIndex = 0;
        currentPath = pathToFire5;
        totalPathPoints = sizeof(pathToFire5) / sizeof(PathPoint_t);
        currentFireId = 5;

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
// 返回值：0=失败，1=成功启动，2=任务已在执行中
uint8_t Mission_StartByFireId(uint8_t fireId)
{

    // 验证火源ID的有效性并启动对应任务
    switch (fireId)
    {
    case 1:
        Mission_StartFire1();
        return 1;
        break;
    case 2:
        Mission_StartFire2();
        return 1;
        break;
    case 3:
        Mission_StartFire3();
        return 1;
        break;
    case 4:
        Mission_StartFire4();
        return 1;
        break;
    case 5:
        Mission_StartFire5();
        return 1;
        break;
    case 6:
        Mission_StartFire6();
        return 1;
        break;
    default:
        // 无效的火源ID
        return 0;
        break;
    }
}

// 接收无人机发送的火源信息
void Mission_ReceiveFireInfo(float fire_x, float fire_y, uint8_t fire_id)
{
    // 验证接收到的数据是否有效
    if (fire_id == 0 || fire_id > 6)
    {
        // 无效的火源ID，忽略
        return;
    }

    // 更新无人机数据结构（如果需要保存坐标信息）
    drone_data.drone_x = fire_x;
    drone_data.drone_y = fire_y;
    drone_data.fire_id = fire_id;

    // 根据接收到的火源信息启动相应任务
    uint8_t result = Mission_StartByFireId(fire_id);

    // 可以根据返回值进行相应处理
    switch (result)
    {
    case 0:
        // 任务启动失败（无效ID或系统忙）
        // 可以在这里添加错误处理逻辑，比如发送反馈给飞机
        break;
    case 1:
        // 任务成功启动
        // 可以在这里添加成功反馈逻辑
        break;
    case 2:
        // 相同任务已在执行中
        // 可以在这里添加状态反馈逻辑
        break;
    }
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
        flag_jim=666;
        //printf("timeouttttttttttttttt\n");
        // 灭火超时，放弃当前灭火任务，进入返回状态
        OLED_ShowString(0, 0, "Fire Fighting Timeout",12,0);
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
        case 3:
            currentMissionState = MISSION_FIRE3_RETURN;
            currentPath = pathFromFire3;
            totalPathPoints = sizeof(pathFromFire3) / sizeof(PathPoint_t);
            break;
        case 4:
            currentMissionState = MISSION_FIRE4_RETURN;
            currentPath = pathFromFire4;
            totalPathPoints = sizeof(pathFromFire4) / sizeof(PathPoint_t);
            break;
        case 5:
            currentMissionState = MISSION_FIRE5_RETURN;
            currentPath = pathFromFire5;
            totalPathPoints = sizeof(pathFromFire5) / sizeof(PathPoint_t);
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
                //printf("    位置稳定，开始定位\n");flag_jim=1;
            }

            // 检查位置是否已经稳定一段时间
           if (current_time - position_stable_time > FIRE_POSITION_STABLE_TIME)
           //if(flag_jim==1)//4 记得改回来 测试用
            {
                // 位置已稳定，进入瞄准阶段
                fireProcessState = FIRE_PROCESS_AIMING;
                //printf("    位置稳定，进入瞄准阶段\n");flag_jim=2;
                // 重置稳定标志
                is_position_stable = 0;
            }
        }
        else
        {
            // 还未到达目标位置，重置稳定标志
            is_position_stable = 0;flag_jim=3;
            //printf("    还未到达目标位置，等待导航完成\n");
        }
        break;

    case FIRE_PROCESS_AIMING:
        // 瞄准火源阶段，等待视觉反馈确认目标锁定
        //if (Servo_GetXAngle() != 0 && Servo_GetYAngle() != 0)//      1 记得改回来
        //if (Servo_GetXAngle() != 0 || Servo_GetYAngle() != 0)
        //if(1)
        //{
               OLED_ShowString(0, 1, "Aiming",12,0);

            // 检查舵机角度是否已经改变（不等于中心位置）
            if (Servo_GetXAngle() != SERVO_X_CENTER_ANGLE || Servo_GetYAngle() != SERVO_Y_CENTER_ANGLE)
            {
                flag_jim=4;
                // 舵机已移动，进入激光灭火阶段
                fireProcessState = FIRE_PROCESS_FIRING;
                //printf("舵机已移动，进入激光灭火阶段\n");
            }
        //}
        break;

    case FIRE_PROCESS_FIRING:
        // 激光灭火阶段，保持激光照射火源
        // 通过视觉反馈确认是否已灭火
        OLED_ShowString(0, 2, "Firing",12,0);
        if (Mission_IsFireExtinguished())
        {
            flag_jim=5;
            //printf("灭火成功，进入确认阶段\n");
            OLED_ShowString(0, 3, "Fire Extinguished",12,0);
            // 灭火成功，进入确认阶段
            fireProcessState = FIRE_PROCESS_CONFIRMING;
        }
        break;

    case FIRE_PROCESS_CONFIRMING:
        // 确认灭火成功阶段，再次检查火源是否已熄灭
        OLED_ShowString(0, 4, "FIRE_PROCESS_CONFIRMING",12,0);

        if (Mission_IsFireExtinguished())
        {

                           OLED_ShowString(0, 5, "CCconfirming",12,0);

            // 再次确认灭火成功，完成灭火任务
            fireProcessState = FIRE_PROCESS_COMPLETED;
           // printf("灭火确认成功，准备返回\n");

            // 停止舵机
            Servo_Reset();

            // 切换到返回状态
            switch (currentFireId)
            {
            case 1:
                currentMissionState = MISSION_FIRE1_RETURN;
                currentPath = pathFromFire1;
                totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
                //printf("设置返回状态：MISSION_FIRE1_RETURN，路径点数=%d\n", totalPathPoints);
                break;
            case 2:
                currentMissionState = MISSION_FIRE2_RETURN;
                currentPath = pathFromFire2;
                totalPathPoints = sizeof(pathFromFire2) / sizeof(PathPoint_t);
                break;
            case 3:
                currentMissionState = MISSION_FIRE3_RETURN;
                currentPath = pathFromFire3;
                totalPathPoints = sizeof(pathFromFire3) / sizeof(PathPoint_t);
                break;
            case 4:
                currentMissionState = MISSION_FIRE4_RETURN;
                currentPath = pathFromFire4;
                totalPathPoints = sizeof(pathFromFire4) / sizeof(PathPoint_t);
                break;
            case 5:
                currentMissionState = MISSION_FIRE5_RETURN;
                currentPath = pathFromFire5;
                totalPathPoints = sizeof(pathFromFire5) / sizeof(PathPoint_t);
                break;
            case 6:
                currentMissionState = MISSION_FIRE6_RETURN;
                currentPath = pathFromFire6;
                totalPathPoints = sizeof(pathFromFire6) / sizeof(PathPoint_t);
                break;
            }

            // 重置路径索引，开始返回
            currentPathIndex = 0;
            //printf("开始返回，设置第一个返回点：(%.1f, %.1f)\n", 
//                   currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//            setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//            startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        }
        else
        {
            // 火源可能重新点燃，返回激光灭火阶段
            fireProcessState = FIRE_PROCESS_FIRING;
        }
        break;

    case FIRE_PROCESS_COMPLETED:
    //printf( "灭火任务已完成\n");
        // 任务完成，切换到空闲状态
        // 如果状态是COMPLETED但任务状态还没有转换，执行转换
        // if (currentMissionState == MISSION_FIRE1_FIGHTING && currentFireId == 1) {
        //     currentMissionState = MISSION_FIRE1_RETURN;
        //     currentPath = pathFromFire1;
        //     totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
        //     currentPathIndex = 0;
        //     setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        //     startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
        // }
        // 类似地处理其他火源
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
    flag_jim=51;
    // 每100ms检查一次
    if (current_time - last_check_time > 200)
    {
        last_check_time = current_time;

        // 检查舵机角度是否稳定
       // if (fabs(Servo_GetXAngle() - SERVO_X_CENTER_ANGLE) < 20.0f &&
           // fabs(Servo_GetYAngle() - SERVO_Y_CENTER_ANGLE) < 20.0f)
        if(vision_data.target_detected == 1)//5记得改回来
        {
            if (fabs(vision_data.error_x)< 20.0f &&(vision_data.error_x) < 20.0f)
            {
                 fireSuccessCounter++;
            }
         
          //  printf("检测到舵机稳定，计数器：%d\n", fireSuccessCounter);
            flag_jim=52; // 显示进入计数状态

            // 如果连续多次检测到舵机稳定，则认为灭火成功
            if (fireSuccessCounter >= FIRE_SUCCESS_THRESHOLD)
            {
                flag_jim=53; // 显示灭火成功状态
                fireSuccessCounter = 0;
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
    //Servo_Update();

    // // 检查任务超时
    // if (currentMissionState != MISSION_IDLE && currentMissionState != MISSION_COMPLETE)
    // {
    //     if (HAL_GetTick() - missionStartTime > MISSION_TIMEOUT)
    //     {
    //         // 任务超时，中止任务
    //         Mission_Stop(); 
    //         return;
    //     }
    // }

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
        //printf("当前返回状态：路径点=%d, 总点数=%d, 导航状态=%d\n", 
//               currentPathIndex, totalPathPoints, navyState);
        
        // 添加更多调试信息
        //printf("当前位置：(%.1f, %.1f), 目标位置：(%.1f, %.1f), 距离：%.2f\n",
//               currentPos.x, currentPos.y, 
//               targetPosition.x, targetPosition.y,
//               calculateDistance(currentPos, targetPosition));
        
      //  uint32_t current_time = HAL_GetTick();
       // flag_jim++;
        //printf("nav_stuck_time=%d,\n",nav_stuck_time);
        // 每秒检查一次导航状态
//        if (current_time - last_nav_check_time > 1000)
//        {
//            last_nav_check_time = current_time;
//            
//            if (navyState != NAVY_STATE_ARRIVED)
//            {
//                nav_stuck_time += 1000;
//                printf("here");
//                // 如果5秒内导航状态没有变为ARRIVED，尝试重新启动导航
//                if (nav_stuck_time > 2000)
//                {
//                    printf("导航似乎卡住了，尝试重新启动导航\n");
//                    stopNavigation();
//                    HAL_Delay(100);
//                    startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//                    nav_stuck_time = 0;
//                }
//            }
//            else
//            {
//                // 重置卡住计时器
//                nav_stuck_time = 0;
//            }
//        }
        
        if (navyState == NAVY_STATE_ARRIVED)
        {
            // 到达当前路径点
            currentPathIndex++;
           // printf("到达返回路径点，前进到下一点：%d/%d\n", currentPathIndex, totalPathPoints);

            if (currentPathIndex < totalPathPoints)
            {
                // 继续前往下一个路径点
                //printf("设置下一个返回路径点：(%.1f, %.1f)\n", 
//                       currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
               setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
               startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
            }
            else
            {
                // 返回完成，任务结束
               // printf("返回完成，任务结束\n");
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
        // 火源2返回状态
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

    case MISSION_FIRE3_GOING:
        // 前往火源3状态
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
                // 到达火源3附近，切换到灭火状态
                currentMissionState = MISSION_FIRE3_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(3);
            }
        }
        break;

    case MISSION_FIRE3_FIGHTING:
        // 灭火3状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE3_RETURN:
        // 火源3返回状态
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

    case MISSION_FIRE4_GOING:
        // 前往火源4状态
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
                // 到达火源4附近，切换到灭火状态
                currentMissionState = MISSION_FIRE4_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(4);
            }
        }
        break;

    case MISSION_FIRE4_FIGHTING:
        // 灭火4状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE4_RETURN:
        // 火源4返回状态
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

    case MISSION_FIRE5_GOING:
        // 前往火源5状态
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
                // 到达火源5附近，切换到灭火状态
                currentMissionState = MISSION_FIRE5_FIGHTING;

                // 开始灭火处理
                Mission_StartFireProcessing(5);
            }
        }
        break;

    case MISSION_FIRE5_FIGHTING:
        // 灭火5状态
        Mission_HandleFireFighting();
        break;

    case MISSION_FIRE5_RETURN:
        // 火源5返回状态
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
        // 火源6返回状态
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

//￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**********测试函数
//￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**********测试函数
//￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**********测试函数
//￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**********测试函数
//￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥**********测试函数

// void SimpleMissionTest(void)//测试航点遍历
// {
//     // 初始化
//     Mission_Init();
    
//     // 设置简单路径点
//     PathPoint_t simplePoints[] = {
//         {START_X + 5.0f, START_Y, NORMAL_SPEED},
//         {START_X + 5.0f, START_Y + 5.0f, NORMAL_SPEED},
//         {START_X, START_Y + 5.0f, NORMAL_SPEED},
//         {START_X, START_Y, NORMAL_SPEED}
//     };
    
//     // 手动设置路径
//     currentPath = simplePoints;
//     totalPathPoints = 4;
//     currentPathIndex = 0;
    
//     // 开始导航
//     setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//     startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
    
//     // 在主循环中更新并打印状态
//     while (currentPathIndex < totalPathPoints)
//     {
//         NavyState_t state = getNavigationState();
//         Position_t pos = getCurrentPosition();
        
//         printf("navy: destination(%d/%d), position(%.2f, %.2f), stete:%d\r\n",
//                currentPathIndex+1, totalPathPoints,
//                pos.x, pos.y, state);
        
//         if (state == NAVY_STATE_ARRIVED)
//         {
//             currentPathIndex++;
//             if (currentPathIndex < totalPathPoints)
//             {
//                 setTargetPosition(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//                 startNavigation(currentPath[currentPathIndex].x, currentPath[currentPathIndex].y);
//             }
//         }
        
//         HAL_Delay(100);
//     }
    
//     printf("简单测试完成!\r\n");
// }

// // 设置导航状态（仅用于测试）
// void setNavigationStateForTest(NavyState_t state)
// {
//     // 需要修改navy.c中的navyState为全局变量或添加setter函数
//     extern NavyState_t navyState;
//     navyState = state;
// }

// // 设置任务状态（仅用于测试）
// void setMissionStateForTest(MissionState_t state)
// {
//     currentMissionState = state;
// }

// // 启动灭火处理（仅用于测试）
// void startFireProcessingForTest(uint8_t fire_id)
// {
//     Mission_StartFireProcessing(fire_id);
// }

// // 模拟灭火成功（仅用于测试）
// void simulateFireExtinguished(void)
// {
//     // 直接设置fireSuccessCounter为成功阈值
//     extern uint8_t fireSuccessCounter;
//     fireSuccessCounter = FIRE_SUCCESS_THRESHOLD;
    
//     // 设置舵机角度为中心位置附近
//     Servo_SetXAngle(135, 0);
//     Servo_SetYAngle(135, 0);
    
//     // 设置灭火处理状态为已完成
//     fireProcessState = FIRE_PROCESS_COMPLETED;
// }

// void TestMissionStateMachine(void)
// {
//     // 初始化任务系统
//     Mission_Init();
//     printf("initial state: %d\r\n", Mission_GetState());
    
//     // 启动火源1任务
//     printf("start fire1 mission\r\n");
//     Mission_StartFire1();
//     printf("current state: %d (should be MISSION_FIRE1_GOING=1)\r\n", Mission_GetState());
    
//     // 打印当前路径信息
//     printf("current path index: %d, total path points: %d\r\n", currentPathIndex, totalPathPoints);
    
//     // 模拟完成所有路径点到达火源1位置
//     currentPathIndex = totalPathPoints - 1;  // 设置为最后一个路径点
    
//     // 模拟导航到达火源1位置
//     setCurrentPosition(FIRE1_X - FIRE_DISTANCE, FIRE1_Y, 0.0f);
//     setNavigationStateForTest(NAVY_STATE_ARRIVED);
    
//     // 更新任务状态
//     Mission_Update();
//         printf("after arrived fire1 position, state: %d (should be MISSION_FIRE1_FIGHTING)\r\n", Mission_GetState());
    
//     // 如果状态没有变化，检查条件
//     if(Mission_GetState() == MISSION_FIRE1_GOING) {
//         printf("状态未变化，检查条件: navyState=%d, currentPathIndex=%d, totalPathPoints=%d\r\n",
//                getNavigationState(), currentPathIndex, totalPathPoints);
//     }
    
//     // 如果状态已经变为灭火状态，模拟灭火过程
//     if(Mission_GetState() == MISSION_FIRE1_FIGHTING) {
//         printf("当前火源ID: %d\r\n", currentFireId);
        
//         // 确保火源ID正确设置
//         currentFireId = 1;
        
//         // 直接设置任务状态为返回状态
//         setMissionStateForTest(MISSION_FIRE1_RETURN);
        
//         // 设置返回路径
//         currentPath = pathFromFire1;
//         totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
        
//         printf("manually set to return state: %d (MISSION_FIRE1_RETURN=3)\r\n", Mission_GetState());
        
//         // 模拟已经完成所有返回路径点
//         currentPathIndex = totalPathPoints;  // 设置为超过最后一个路径点
        
//         // 模拟返回到起点
//         setCurrentPosition(START_X, START_Y, 0.0f);
//         setNavigationStateForTest(NAVY_STATE_ARRIVED);
        
//         // 更新任务状态
//         Mission_Update();
//         printf("after returned to start point, state: %d (should be MISSION_COMPLETE=10)\r\n", Mission_GetState());
//     }
    
//     printf("test mission state machine completed\r\n");
// }
// //。。。。
// void TestFireProcessing(void)
// {
//     // 初始化
//     Mission_Init();
//     printf("start test fire processing function\r\n");
    
//     // 设置为火源1的灭火状态
//     setMissionStateForTest(MISSION_FIRE1_FIGHTING);
//     currentFireId = 1;
    
//     // 启动灭火处理
//     startFireProcessingForTest(1);
//     printf("initial fire processing state: %d\r\n", Mission_GetFireProcessState());
    
//     // 模拟位置稳定
//     setNavigationStateForTest(NAVY_STATE_ARRIVED);
    
//     // 记录灭火处理开始时间
//     fireProcessStartTime = HAL_GetTick();
    
//     // 更新多次，模拟位置稳定时间
//     for(int i = 0; i < 5; i++) {
//         Mission_Update();
//         printf("position stable, fire processing state: %d\r\n", Mission_GetFireProcessState());
//         HAL_Delay(1000); // 延时1秒
//     }
    
//     // 模拟舵机移动
//     Servo_SetXAngle(120, 0);
//     Servo_SetYAngle(120, 0);
    
//     // 更新并检查状态
//     Mission_Update();
//     printf("after servo moved, fire processing state: %d\r\n", Mission_GetFireProcessState());
    
//     // 设置为激光灭火阶段
//     fireProcessState = FIRE_PROCESS_FIRING;
    
//     // 更新并检查状态
//     Mission_Update();
//     printf("firing state, fire processing state: %d\r\n", Mission_GetFireProcessState());
    
//     // 设置为确认阶段
//     fireProcessState = FIRE_PROCESS_CONFIRMING;
    
//     // 模拟灭火成功
//     simulateFireExtinguished();
    
//     // 确保路径设置正确
//     currentPath = pathFromFire1;
//     totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
//             currentPathIndex = totalPathPoints;  // 设置为超过最后一个路径点

//     // 更新并检查状态
//     Mission_Update();
//     printf("after fire extinguished, mission state: %d (should be MISSION_FIRE1_RETURN=3), fire state: %d\r\n", 
//            Mission_GetState(), Mission_GetFireProcessState());
    
//     // 如果状态仍然没有变化，直接设置
//     if(Mission_GetState() != MISSION_FIRE1_RETURN) {
//         printf("状态未变化，手动设置为返回状态\r\n");
//         setMissionStateForTest(MISSION_FIRE1_RETURN);
        
//         // 确保路径设置正确
//         currentPath = pathFromFire1;
//         totalPathPoints = sizeof(pathFromFire1) / sizeof(PathPoint_t);
//         currentPathIndex = 0;
        
//         // 更新任务状态
//         Mission_Update();
//         printf("手动设置后，任务状态: %d\r\n", Mission_GetState());
//     }
    
//     printf("test fire processing function completed\r\n");
// }

// void TestCompleteFire1Mission(void)
// {
//     // 初始化
//     Mission_Init();
//         printf("start test complete fire1 mission\r\n");
    
//     // 启动火源1任务
//     Mission_StartFire1();
    
//     // 模拟完成路径点导航
//     for(int i = 0; i < totalPathPoints; i++) {
//         printf("导航到路径点%d/%d\r\n", i+1, totalPathPoints);
        
//         // 模拟到达当前路径点
//         setNavigationStateForTest(NAVY_STATE_ARRIVED);
        
//         // 更新任务状态
//         Mission_Update();
        
//         // 打印当前状态
//         printf("after arrived path point %d/%d, state: %d\r\n", i+1, totalPathPoints, Mission_GetState());
        
//         // 如果已经进入灭火状态，退出循环
//         if(Mission_GetState() == MISSION_FIRE1_FIGHTING)
//             break;
//     }
    
//     // 模拟灭火过程
//     if(Mission_GetState() == MISSION_FIRE1_FIGHTING) {
//         printf("start fire processing\r\n");
        
//         // 模拟位置稳定
//         for(int i = 0; i < 3; i++) {
//             Mission_Update();
//             HAL_Delay(1000);
//         }
        
//         // 模拟舵机移动
//         Servo_SetXAngle(120, 0);
//         Servo_SetYAngle(120, 0);
//         Mission_Update();
        
//         // 模拟灭火成功
//         simulateFireExtinguished();
//         Mission_Update();
        
//         printf("fire extinguished, state: %d\r\n", Mission_GetState());
//     }
    
//     // 模拟返回过程
//     if(Mission_GetState() == MISSION_FIRE1_RETURN) {
//         printf("start return process\r\n");
        
//         // 模拟完成返回路径点导航
//         for(int i = 0; i < totalPathPoints; i++) {
//             printf("导航到返回路径点%d/%d\r\n", i+1, totalPathPoints);
            
//             // 模拟到达当前路径点
//             setNavigationStateForTest(NAVY_STATE_ARRIVED);
            
//             // 更新任务状态
//             Mission_Update();
            
//             // 打印当前状态
//             printf("after arrived return path point %d/%d, state: %d\r\n", i+1, totalPathPoints, Mission_GetState());
            
//             // 如果任务已完成，退出循环
//             if(Mission_GetState() == MISSION_COMPLETE)
//                 break;
//         }
//     }
    
//     printf("test complete fire1 mission completed, final state: %d\r\n", Mission_GetState());
// }


// void TestVisionFeedback(void)
// {
//     // 初始化
//     Mission_Init();
//     printf("start test vision feedback processing\r\n");
    
//     // 设置为火源1的灭火状态
//     setMissionStateForTest(MISSION_FIRE1_FIGHTING);
    
//     // 模拟视觉反馈数据
//     float error_x = 10.0f;
//     float error_y = 5.0f;
//     uint8_t target_detected = 1;
    
//     // 处理视觉反馈数据
//     Mission_ProcessVisionData(error_x, error_y, target_detected);
//     printf("process vision feedback data: (%.2f, %.2f), target detected: %d\r\n", 
//            error_x, error_y, target_detected);
    
//     // 更新舵机控制
//     Servo_Update();
    
//     // 获取舵机角度
//     float x_angle = Servo_GetXAngle();
//     float y_angle = Servo_GetYAngle();
//     printf("servo angle: X=%.2f, Y=%.2f\r\n", x_angle, y_angle);
    
//     printf("test vision feedback processing completed\r\n");
// }

/**
 * @brief 优化的OLED状态显示函数，包含更多调试信息
 */
void Display_DebugStatus(void)
{
    OLED_Clear();
    
    // 第1行：显示任务状态和数值
    MissionState_t state = Mission_GetState();
    OLED_ShowString(0, 0, "S:", 12, 0);
    
    // 显示状态名称和数值
    switch (state)
    {
        case MISSION_IDLE:
            OLED_ShowString(16, 0, "IDLE", 12, 0);
            break;
        case MISSION_FIRE1_GOING:
            OLED_ShowString(16, 0, "GOING1", 12, 0);
            break;
        case MISSION_FIRE1_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT1", 12, 0);
            break;
        case MISSION_FIRE1_RETURN:
            OLED_ShowString(16, 0, "RETN1", 12, 0);
            break;
        case MISSION_FIRE2_GOING:
            OLED_ShowString(16, 0, "GOING2", 12, 0);
            break;
        case MISSION_FIRE2_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT2", 12, 0);
            break;
        case MISSION_FIRE2_RETURN:
            OLED_ShowString(16, 0, "RETN2", 12, 0);
            break;
        case MISSION_FIRE3_GOING:
            OLED_ShowString(16, 0, "GOING3", 12, 0);
            break;
        case MISSION_FIRE3_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT3", 12, 0);
            break;
        case MISSION_FIRE3_RETURN:
            OLED_ShowString(16, 0, "RETN3", 12, 0);
            break;
        case MISSION_FIRE4_GOING:
            OLED_ShowString(16, 0, "GOING4", 12, 0);
            break;
        case MISSION_FIRE4_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT4", 12, 0);
            break;
        case MISSION_FIRE4_RETURN:
            OLED_ShowString(16, 0, "RETN4", 12, 0);
            break;
        case MISSION_FIRE5_GOING:
            OLED_ShowString(16, 0, "GOING5", 12, 0);
            break;
        case MISSION_FIRE5_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT5", 12, 0);
            break;
        case MISSION_FIRE5_RETURN:
            OLED_ShowString(16, 0, "RETN5", 12, 0);
            break;
        case MISSION_FIRE6_GOING:
            OLED_ShowString(16, 0, "GOING6", 12, 0);
            break;
        case MISSION_FIRE6_FIGHTING:
            OLED_ShowString(16, 0, "FIGHT6", 12, 0);
            break;
        case MISSION_FIRE6_RETURN:
            OLED_ShowString(16, 0, "RETN6", 12, 0);
            break;
        case MISSION_COMPLETE:
            OLED_ShowString(16, 0, "DONE", 12, 0);
            break;
        default:
            OLED_ShowString(16, 0, "UNK", 12, 0);
            break;
    }
    // 显示状态数值
    OLED_ShowNum(60, 0, state, 1, 12, 0);
    
    // 第2行：显示火源处理状态和数值
    FireProcessState_t fireState = Mission_GetFireProcessState();
    OLED_ShowString(0, 2, "F:", 12, 0);
    
    switch (fireState)
    {
        case FIRE_PROCESS_POSITIONING:
            OLED_ShowString(16, 2, "POS", 12, 0);
            break;
        case FIRE_PROCESS_AIMING:
            OLED_ShowString(16, 2, "AIM", 12, 0);
            break;
        case FIRE_PROCESS_FIRING:
            OLED_ShowString(16, 2, "FIRE", 12, 0);
            break;
        case FIRE_PROCESS_CONFIRMING:
            OLED_ShowString(16, 2, "CONF", 12, 0);
            break;
        case FIRE_PROCESS_COMPLETED:
            OLED_ShowString(16, 2, "DONE", 12, 0);
            break;
        default:
            OLED_ShowString(16, 2, "UNK", 12, 0);
            break;
    }
    // 显示火源状态数值
    OLED_ShowNum(60, 2, fireState, 1, 12, 0);
    
    // 第3行：显示X舵机角度
    OLED_ShowString(0, 4, "SSX:", 12, 0);
    OLED_ShowNum(16, 4, (uint16_t)Servo_GetXAngle(), 3, 12, 0);
    
    // 显示X舵机运行状态
    OLED_ShowString(64, 4, "flag:", 12, 0);
    //if (servo_x.is_running)
        OLED_ShowNum(96, 4, flag_jim, 1, 12, 0);
    //else
        //OLED_ShowString(96, 4, "N", 12, 0);
    
    // 第4行：显示Y舵机角度
    OLED_ShowString(0, 6, "SSY:", 12, 0);
    OLED_ShowNum(16, 6, (uint16_t)Servo_GetYAngle(), 3, 12, 0);
    
    // 显示目标检测状态
    OLED_ShowString(64, 6, "Det:", 12, 0);
    if (vision_data.target_detected)
        OLED_ShowString(96, 6, "Y", 12, 0);
    else
        OLED_ShowString(96, 6, "N", 12, 0);
}

