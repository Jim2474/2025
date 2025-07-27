#include "M_timer.h"

static volatile uint32_t system_tick_ms = 0;

#include "tim.h"

// 定时器2中断调度
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 

    if(htim->Instance == TIM2)
    {
		Key_Loop();
        static uint8_t cnt_100Hz = 0;
        TIM2_Task_1000Hz(); // 每1ms调用一次
        if(++cnt_100Hz >= 10)
        {   
 // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  // PWMA使能
			 //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
            cnt_100Hz = 0;
            TIM2_Task_100Hz(); // 每10ms调用一次
        }

        // 系统毫秒计数器递增
        system_tick_ms++;
    }
}

/**
 * @brief 简单的非阻塞延时函数
 * @param delay_ms 延时时间(毫秒)
 * @return 1=延时完成, 0=延时未完成
 *
 * 使用方法：
 * if (delay_ms_nb(1000)) {
 *     // 每1000ms执行一次
 *     printf("1秒到了\n");
 * }
 */
uint8_t delay_ms_nb(uint32_t delay_ms)
{
    static uint32_t last_time = 0;

    if ((system_tick_ms - last_time) >= delay_ms)
     {
        last_time = system_tick_ms;
        return 1;  // 延时完成
    }

    return 0;  // 延时未完成
}

/**
 * @brief 带ID的非阻塞延时函数（支持多个独立延时）
 * @param delay_ms 延时时间(毫秒)
 * @param id 延时ID (0-7，最多支持8个独立延时)
 * @return 1=延时完成, 0=延时未完成
 *
 * 使用方法：
 * if (delay_ms_nb_id(500, 0)) {
 *     // 延时0：每500ms执行一次
 * }
 * if (delay_ms_nb_id(1000, 1)) {
 *     // 延时1：每1000ms执行一次
 * }
 */
uint8_t delay_ms_nb_id(uint32_t delay_ms, uint8_t id)
{
    static uint32_t last_time[8] = {0};  // 支持8个独立延时

    if (id >= 8) return 0;  // ID超出范围

    if ((system_tick_ms - last_time[id]) >= delay_ms) {
        last_time[id] = system_tick_ms;
        return 1;  // 延时完成
    }

    return 0;  // 延时未完成
}

/**
 * @brief 获取系统毫秒计数
 * @return 当前毫秒计数
 */
uint32_t get_system_tick_ms(void)
{
    return system_tick_ms;
}

//如果某个函数被__weak修饰，用户可以在其他文件中重新实现同名函数，链接时会自动用用户的实现覆盖掉弱定义。
//如果用户没有实现，则用弱定义的空函数，不会报错。 方便同名函数重新定义 但是在这里却可以指示
__weak void TIM2_Task_1000Hz(void)
{
    // 用户自定义1kHz任务
}

__weak void TIM2_Task_100Hz(void)
{
    // 用户自定义100Hz任务
}








