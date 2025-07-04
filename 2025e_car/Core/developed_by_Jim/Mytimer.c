#include "Mytimer.h"
#include "tim.h"


// 定时器2中断调度
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 

    if(htim->Instance == TIM2)
    {

        static uint8_t cnt_100Hz = 0;
        TIM2_Task_1000Hz(); // 每1ms调用一次
        if(++cnt_100Hz >= 10)
        {   
 // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  // PWMA使能
			 //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
            cnt_100Hz = 0;
            TIM2_Task_100Hz(); // 每10ms调用一次
        }
    }
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








