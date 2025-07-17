#include "M_navy.h"


// 全局位置变量定义
Position_t currentPosition = {0.0f, 0.0f, 0.0f};  // 当前位置，初始为原点
Position_t targetPosition = {0.0f, 0.0f};         // 目标位置，初始为原点

// 内部变量
static float lastLeftWheelDist = 0.0f;   // 上次左轮累计行驶距离
static float lastRightWheelDist = 0.0f;  // 上次右轮累计行驶距离
static float totalLeftDist = 0.0f;       // 左轮总行驶距离
static float totalRightDist = 0.0f;      // 右轮总行驶距离

 NavyState_t navyState = NAVY_STATE_IDLE;    // 当前导航状态
static float targetDistanceThreshold = 0.5f;       // 到达目标的距离阈值(dm) 记得改回来
static float targetVelocity = 10.0f;               // 导航时的目标速度(cm/s)
static float maxAngularVelocity = 60.0f;           // 最大角速度(度/秒)

static float rotation_accum = 0.0f;
static float last_yaw = 0.0f;
static int rotating = 0; // 0:无旋转任务 1:正在旋转
static float rotation_target = 0.0f; // 目标旋转角度（正负表示方向，单位：度）
static int rotation_lock = 0; // 0:可触发旋转 1:旋转完成后需微调

/**
 * @brief 初始化导航系统
 */
void navy_init(void)
{
    // 重置位置到原点
    resetPosition();
    
    // 可以在这里添加其他初始化操作，如传感器校准等
    navyState = NAVY_STATE_IDLE;
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
 * @brief 更新当前位置（在TIM2_Task_100Hz中调用）#####################################################################
 * ################################################################################################################
 * 使用编码器数据和陀螺仪数据更新位置
 */
void updatePosition(void)
{
    // 1. 计算左右轮的行驶距离增量（单位：dm）
    // 添加校准系数0.92来修正距离误差(60cm实际走了65cm，比例约为60/65=0.92)
    float calibrationFactor = 1.0f;
    
    float leftWheelDist = (totalLeftDist + (left_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor)) / 10.0f;  // cm转换为dm
    float rightWheelDist = (totalRightDist + (right_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor)) / 10.0f;
    
    float delta_Left_Dist = leftWheelDist - lastLeftWheelDist;
    float delta_Right_Dist = rightWheelDist - lastRightWheelDist;
    
    // 更新累计距离
    lastLeftWheelDist = leftWheelDist;
    lastRightWheelDist = rightWheelDist;
    totalLeftDist += left_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor;  // 累加当前速度对应的距离
    totalRightDist += right_wheel_speed * POSITION_UPDATE_INTERVAL * calibrationFactor;
    
    // 2. 计算中心点行驶距离和角度变化
    float delta_Dist = (delta_Left_Dist + delta_Right_Dist) / 2.0f;  // 中心点行驶距离
    
    // 3. 使用陀螺仪数据更新航向角（更准确）
    currentPosition.theta = deg2rad(IMU_data.YawZ);  // 陀螺仪数据是角度，转换为弧度
    
    // 4. 使用航向角和行驶距离更新位置坐标
    currentPosition.x += delta_Dist * cosf(currentPosition.theta);
    currentPosition.y += delta_Dist * sinf(currentPosition.theta);
    //printf("Position: (%.2f, %.2f), Theta: %.2f\n", 
          // currentPosition.x*10, currentPosition.y*10, rad2deg(currentPosition.theta));
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
    while (angle > PI) 
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
     {
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

/**
 * @brief 计算目标点相对于当前位置的方向角
 * @return 方向角，单位：弧度，范围[-PI, PI]
 */
float calculateTargetAngle(void)
{
    float dx = targetPosition.x - currentPosition.x;
    float dy = targetPosition.y - currentPosition.y;
    return normalizeAngle(atan2f(dy, dx));
}

/**
 * @brief 开始导航到目标点
 * @param x 目标点X坐标，单位：dm
 * @param y 目标点Y坐标，单位：dm
 * @return 1表示成功开始导航，0表示当前已在导航中或参数无效
 */
uint8_t startNavigation(float x, float y)
{
    // 检查是否已在导航状态
    if (navyState == NAVY_STATE_MOVING)
        return 0;
    
    // 设置目标点
    setTargetPosition(x, y);
    
    // 如果目标点与当前点距离小于阈值，认为已到达
    if (calculateDistance(currentPosition, targetPosition) < targetDistanceThreshold)
    {
        navyState = NAVY_STATE_ARRIVED;
        return 0;
    }
    
    // 开始导航
    navyState = NAVY_STATE_MOVING;
    return 1;
}

/**
 * @brief 停止当前导航
 */
void stopNavigation(void)
{
    navyState = NAVY_STATE_IDLE;
    set_target_speed(0.0f, 0.0f); // 停止小车
}

/**
 * @brief 获取当前导航状态
 * @return 导航状态枚举值
 */
NavyState_t getNavigationState(void)
{
    return navyState;
}

/**
 * @brief 设置导航参数
 * @param distThreshold 到达目标的距离阈值(dm)
 * @param velocity 导航时的目标线速度(cm/s)
 * @param angVelocity 最大角速度(度/秒)
 */
void setNavigationParameters(float distThreshold, float velocity, float angVelocity)
{
    targetDistanceThreshold = distThreshold;
    targetVelocity = velocity;
    
    maxAngularVelocity = angVelocity;
}

/**
 * @brief 更新导航控制，支持原地旋转任务
 * 在TIM2_Task_100Hz中周期性调用
 */
void updateNavigation_control(void)
{
    // // 优先处理原地旋转任务
    // if (rotating) {
    //     update_rotation_task();
    //     return;
    // }

    // 如果不在导航状态，直接返回
    if (navyState != NAVY_STATE_MOVING)
        return;

    // 计算到目标点的距离
    float distance = calculateDistance(currentPosition, targetPosition);

    // 检查是否已到达目标
    if (distance < targetDistanceThreshold)
    {
        navyState = NAVY_STATE_ARRIVED;
        set_target_speed(0.0f, 0.0f); // 停止小车
        rotation_lock = 0; // 允许下次旋转

        return;
    }

    // 计算目标角度
    float targetAngle = calculateTargetAngle();
    float targetAngleDeg = rad2deg(targetAngle);
    float currentYaw = IMU_data.YawZ;

    // 用规范化到[-180,180]的角度差，保证走最短路径
    float angleDiff = normalize_angle(targetAngleDeg - currentYaw);
    float absAngleDiff = fabsf(angleDiff);
    // 旋转完成后，只有当角度差回到小于10度才解锁
    if (absAngleDiff < 10.0f)
    {
        rotation_lock = 0;
    }
    // 当角度差大于30度时，先原地旋转调整方向
    if (absAngleDiff > 60.0f && !rotation_lock)
    {
        // 设置旋转锁，防止重复进入旋转逻辑
        rotation_lock = 1;

        // 调用turn_in_place进行原地转向（非阻塞方式）
        // 传入目标角度差值和最大转向速度
        turn_in_place(targetAngleDeg, 15.0f);

        // 本周期不再做其他事，等待旋转任务完成
        return;
    }

    // 如果正在旋转锁定状态，继续执行转向直到完成
    if (rotation_lock)
    {
        // 继续调用turn_in_place检查是否完成
        if (turn_in_place(targetAngleDeg, 15.0f))
        {
            // 旋转完成，解除锁定
            rotation_lock = 0;
        }
        // 旋转未完成，本周期不执行前进逻辑
        return;
    }
    
    else
    {
        // 角度差较小，可以边前进边微调方向
        float velocityControl = targetVelocity;

        // 根据角度差调整速度
        if (absAngleDiff > 10.0f) {
            velocityControl *= (1.0f - 0.5f * absAngleDiff / 30.0f);
            if (velocityControl < targetVelocity * 0.5f)
                velocityControl = targetVelocity * 0.5f;
        }

        // 如果接近目标点，平滑减速
        if (distance < 1.0f)
        {
            float slowDownFactor = (distance * distance) / 1.0f;
            if (slowDownFactor < 0.5f) slowDownFactor = 0.5f;
            velocityControl *= slowDownFactor;
            if (velocityControl < 8.0f)
                velocityControl = 8.0f;
        }

        // 继续用你的PID函数
        float angularControl = yaw_pid_control(targetAngleDeg);

        // 限制转向控制输出，防止过大导致后退
        float maxAngularControl = velocityControl * 0.8f;
        if (angularControl > maxAngularControl) angularControl = maxAngularControl;
        if (angularControl < -maxAngularControl) angularControl = -maxAngularControl;

        // 设置左右轮目标速度
        float leftSpeed = velocityControl - angularControl;
        float rightSpeed = velocityControl + angularControl;

        // 确保两个轮子都是正向的（不后退）
        if (leftSpeed < 0) leftSpeed = 0;
        if (rightSpeed < 0) rightSpeed = 0;

        set_target_speed(leftSpeed, rightSpeed);
    }
}

/**
 * @brief 启动原地旋转任务
 * @param angle 目标旋转角度（正负表示方向，单位：度）
 */
void start_rotation(float angle) // angle: 360, 720, -360等
{
    rotation_accum = 0.0f;
    last_yaw = IMU_data.YawZ;
    rotation_target = angle;
    rotating = 1;
}

/**
 * @brief 更新原地旋转任务
 * @return 1表示旋转完成，0表示仍在进行
 */
void update_rotation_task(void)
{
    if (!rotating) return;

    float current_yaw = IMU_data.YawZ;
    float delta = current_yaw - last_yaw;

    // 处理跨0/360度跳变
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    rotation_accum += delta;
    last_yaw = current_yaw;

    float remain = rotation_target - rotation_accum;

    // 判断是否完成，增加±3度的误差容忍
    if ((rotation_target > 0 && rotation_accum >= rotation_target - 3.0f) ||
        (rotation_target < 0 && rotation_accum <= rotation_target + 3.0f))
    {
        set_target_speed(0, 0);
        rotating = 0;
        rotation_lock = 0;
        printf("旋转完成，累计角度=%.1f\n", rotation_accum);
        return;
    }

    // 旋转控制
    float speed = 10.0f;
    if (fabsf(remain) < 30.0f) speed = 5.0f; // 接近目标时减速

    if (remain > 0)
        set_target_speed(speed, -speed); // 顺时针
    else
        set_target_speed(-speed, speed); // 逆时针
}

/**
 * @brief ĺŻźčŞćľčŻĺ˝ć° - čŽŠĺ°č˝Śčľ°ä¸ä¸Şć­Łćšĺ˝˘
 */
void navyTest(void)
{
  // čŽžç˝ŽĺŻźčŞĺć° - ĺŻć šćŽĺŽéćĺľč°ć´
  setNavigationParameters(0.5f, 15.0f, 45.0f);  // čˇçŚťéĺź0.5dmďźçşżéĺşŚ15cm/sďźćĺ¤§č§éĺşŚ45Â°/s
  
  // ĺŽäšć­Łćšĺ˝˘çĺä¸ŞéĄśçšĺć 
  float waypoints[4][2] = {
    {0.0f, 0.0f},    // čľˇçš/çťçš
    {5.0f, 0.0f},   // çŹŹä¸ä¸ŞéĄśçš
    {5.0f, 5.0f},  // çŹŹäşä¸ŞéĄśçš
    {0.0f, 5.0f}    // çŹŹä¸ä¸ŞéĄśçš
  };
  
  // éç˝Žä˝ç˝Žĺ°ĺçš
  resetPosition();
  
  // çĄŽäżĺ°č˝Śĺ¤äşçŠşé˛çść
  stopNavigation();
  HAL_Delay(1000);
  
  // äžćŹĄĺŻźčŞĺ°ćŻä¸ŞéĄśçš
  for (int i = 1; i < 5; i++) 
  {
    int point_idx = i % 4;  // ĺžŞçŻĺĺ°čľˇçš
    float x = waypoints[point_idx][0];
    float y = waypoints[point_idx][1];
    
    
    // ĺ¨ĺźĺ§ć°çĺŻźčŞĺďźçĄŽäżĺ°č˝ŚĺŽĺ¨ĺć­˘
    set_target_speed(0.0f, 0.0f);
    HAL_Delay(2000);
    
    // ĺźĺ§ĺŻźčŞĺ°çŽć çš
    if (startNavigation(x, y))
    {
      
      // ç­ĺžĺŻźčŞĺŽć
      uint32_t startTime = HAL_GetTick();
      uint32_t lastPrintTime = 0;
      
      while (getNavigationState() == NAVY_STATE_MOVING)
      {
        uint32_t currentTime = HAL_GetTick();
        
        // ćŻé1ç§čžĺşĺ˝ĺä˝ç˝ŽĺçŽć äżĄćŻ
        if (currentTime - lastPrintTime >= 1000)
        {
          Position_t pos = getCurrentPosition();
          float targetAngle = calculateTargetAngle();
          float angleDiff = normalizeAngle(targetAngle - pos.theta);
          float distance = calculateDistance(pos, targetPosition);
          
 
          lastPrintTime = currentTime;
        }
        
        // čśćśäżć¤ďźé˛ć­˘ĺĄĺ¨ćä¸Şçš
        if (currentTime - startTime > 60000)  // 60ç§čśćś
        {
          stopNavigation();
          break;
        }
        
        // çťçłťçťćśé´ĺ¤çĺśäťäťťĺĄ
        HAL_Delay(10);
      }
      
      // ĺŻźčŞĺŽć
      if (getNavigationState() == NAVY_STATE_ARRIVED)
      {
        Position_t pos = getCurrentPosition();
        
        // ĺ¨ćŻä¸ŞéĄśçšĺçć´éżćśé´
        HAL_Delay(1000);
      }
      else
      {
        break;
      }
    }
    else
    {
      break;
    }
  }
  
  
  // çĄŽäżĺ°č˝Śĺć­˘
  stopNavigation();
  set_target_speed(0.0f, 0.0f);
}