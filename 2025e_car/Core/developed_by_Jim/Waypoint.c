#include "Waypoint.h"

// 全局航点队列
static WaypointQueue_t waypoint_queue;

/**
 * @brief 初始化航点队列
 * @param is_loop 是否循环巡航
 */
void Waypoint_Init(uint8_t is_loop)
{
    // 清空航点队列
    waypoint_queue.count = 0;
    waypoint_queue.current_index = 0;
    waypoint_queue.is_patrolling = 0;
    waypoint_queue.is_loop = is_loop;
}

/**
 * @brief 添加航点到队列
 * @param x X坐标，单位：dm
 * @param y Y坐标，单位：dm
 * @param speed 该点的推荐速度
 * @return 成功返回1，失败返回0
 */
uint8_t Waypoint_Add(float x, float y, float speed)
{
    if (waypoint_queue.count >= MAX_WAYPOINTS)
        return 0;  // 队列已满
    
    waypoint_queue.points[waypoint_queue.count].x = x;
    waypoint_queue.points[waypoint_queue.count].y = y;
    waypoint_queue.points[waypoint_queue.count].speed = speed;
    waypoint_queue.count++;
    
    return 1;
}

/**
 * @brief 开始航点巡航
 * @return 成功返回1，失败返回0
 */
uint8_t Waypoint_StartPatrol(void)
{
    if (waypoint_queue.count == 0)
        return 0;  // 没有航点
    
    if (waypoint_queue.is_patrolling)
        return 0;  // 已经在巡航
    
    // 设置第一个航点
    waypoint_queue.current_index = 0;
    waypoint_queue.is_patrolling = 1;
    
    // 设置导航参数
    setNavigationParameters(0.3f, 
                           waypoint_queue.points[0].speed, 
                           30.0f);
    
    // 开始导航到第一个点
    setTargetPosition(waypoint_queue.points[0].x, waypoint_queue.points[0].y);
    startNavigation(waypoint_queue.points[0].x, waypoint_queue.points[0].y);
    
    return 1;
}

/**
 * @brief 停止航点巡航
 */
void Waypoint_StopPatrol(void)
{
    waypoint_queue.is_patrolling = 0;
    stopNavigation();
}

/**
 * @brief 更新航点巡航状态，在主循环中调用
 * @return 当前巡航状态：0-未巡航，1-巡航中，2-巡航完成
 */
uint8_t Waypoint_Update(void)
{
    if (!waypoint_queue.is_patrolling)
        return 0;  // 未巡航
    
    NavyState_t navyState = getNavigationState();
    
    // 如果已到达当前航点
    if (navyState == NAVY_STATE_ARRIVED)
    {
        // 移动到下一个航点
        waypoint_queue.current_index++;
        
        // 检查是否已完成所有航点
        if (waypoint_queue.current_index >= waypoint_queue.count)
        {
            // 如果是循环模式，重新开始
            if (waypoint_queue.is_loop)
            {
                waypoint_queue.current_index = 0;
            }
            else
            {
                // 非循环模式，巡航结束
                waypoint_queue.is_patrolling = 0;
                return 2;  // 巡航完成
            }
        }
        
        // 设置下一个航点
        uint8_t idx = waypoint_queue.current_index;
        setNavigationParameters(0.3f, 
                               waypoint_queue.points[idx].speed, 
                               30.0f);
        setTargetPosition(waypoint_queue.points[idx].x, waypoint_queue.points[idx].y);
        startNavigation(waypoint_queue.points[idx].x, waypoint_queue.points[idx].y);
    }
    
    return 1;  // 巡航中
}

/**
 * @brief 获取当前巡航状态
 * @param current_index 当前航点索引
 * @param total_count 总航点数
 * @return 是否正在巡航
 */
uint8_t Waypoint_GetStatus(uint8_t *current_index, uint8_t *total_count)
{
    if (current_index)
        *current_index = waypoint_queue.current_index;
    
    if (total_count)
        *total_count = waypoint_queue.count;
    
    return waypoint_queue.is_patrolling;
}

// 在初始化时
void init_example(void)
{
    // 初始化导航系统
    //navy_init();
    
    // 初始化航点队列，设置为非循环模式
    Waypoint_Init(0);
    setCurrentPosition(0, 0, 0.0f); // 假设初始朝向为0度（正东）

    // 添加航点
    Waypoint_Add(5.0f, 0.0f, 20.0f);    // 点1：(5,5)，速度20cm/s
    Waypoint_Add(10.0f, 0.0f, 20.0f);   // 点2：(10,5)，速度15cm/s
        Waypoint_Add(0.0f, 0.0f, 20.0f);   // 点2：(10,5)，速度15cm/s

    // Waypoint_Add(10.0f, 10.0f, 15.0f);  // 点3：(10,10)，速度15cm/s
    // Waypoint_Add(5.0f, 10.0f, 20.0f);   // 点4：(5,10)，速度20cm/s
    // Waypoint_Add(5.0f, 5.0f, 15.0f);    // 点5：返回起点(5,5)，速度15cm/s
    
    // 开始巡航
    Waypoint_StartPatrol();
}

// 在主循环中
void main_loop_example(void)
{
    // 更新航点巡航状态
    uint8_t patrol_status = Waypoint_Update();
    
    if (patrol_status == 2)
    {
        // 巡航完成，可以做一些处理
        printf("巡航完成！\n");
    }
    else if (patrol_status == 1)
    {
        // 正在巡航，可以获取当前状态
        uint8_t current_idx, total_count;
        Waypoint_GetStatus(&current_idx, &total_count);
        printf("正在巡航：%d/%d\n", current_idx + 1, total_count);
    }
    
    // 可以在此处添加其他处理逻辑
}