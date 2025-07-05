/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int32_t left_encoder_count;
extern int32_t right_encoder_count;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//小车初始�????????????
void car_init(void)
{
  
  Motor_PWM_StartAll();//TIM1 pwm11
  HAL_TIM_Base_Start_IT(&htim2);//1ms定时

  // 启动左轮编码�????????????(TIM3)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // 启动右轮编码�????????????(TIM4)
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  Uart_Init();
      
  // 初始化PID控制�????????????
  pid_init_all();
  // 设置初始目标速度0
  set_target_speed(30.0f, 30.0f);
  // 初始化导航系�????????????
  navy_init();
	
}
int jim =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_Task_1000Hz(void)
{
  // 1000Hz任务，每1ms执行�????????????�????????????
  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
  // 1. 更新编码器计数和转�?�计�????????????
  encoder_count();
  // 2. 计算线�?�度
  Calculate_speed(left_wheel_rpm, right_wheel_rpm);
}

void TIM2_Task_100Hz(void)
{
  // 100Hz任务，每10ms执行�????????????�????????????
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);

//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
  // 1. 更新当前位置坐标
  updatePosition();
 	//HAL_UART_Transmit(&huart3, (uint8_t *)&jim, 1, HAL_MAX_DELAY);
//printf("%f,%f\n",g_left_target_speed,g_right_target_speed);
  // 2. 更新导航控制
 // updateNavigation();
  
  // 3. 执行PID控制计算和电机输�????????????
  // 包含速度环和转向环的控制
  //wheels_pid_control_auto_with_yaw();
  wheels_pid_control_auto();
  // 4. 可以添加其他低频任务，如LED状�?�更新�?�按键检测等
  // 这里暂时不添加其他任�????????????
}

/**
 * @brief 导航测试函数
 * @param x 目标X坐标
 * @param y 目标Y坐标
 */
void navyTest(float x, float y)
{
  // 设置导航参数 - 可根据实际情况调�????????????
  setNavigationParameters(0.5f, 15.0f, 45.0f);  // 距离阈�??0.5dm，线速度15cm/s，最大角速度45°/s
  
  // �????????????始导航到目标�????????????
  if (startNavigation(x, y))
  {
    printf("�????????????始导航到目标�????????????: (%.2f, %.2f)\r\n", x, y);
    
    // 等待导航完成
    while (getNavigationState() == NAVY_STATE_MOVING)
    {
      // 每隔�????????????秒输出当前位�????????????
      HAL_Delay(1000);
      Position_t pos = getCurrentPosition();
      printf("当前位置: (%.2f, %.2f), 航向�????????????: %.2f°\r\n", 
             pos.x, pos.y, rad2deg(pos.theta));
    }
    
    // 导航完成
    if (getNavigationState() == NAVY_STATE_ARRIVED)
    {
      printf("已到达目标点!\r\n");
    }
    else
    {
      printf("导航停止!\r\n");
    }
  }
  else
  {
    printf("无法�????????????始导航，请检查当前状态或目标点是否有效\r\n");
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  car_init();
  Uart_Init();
//   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  // PWMA使能
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
      // 使能引脚
      //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);  // PWMA使能
      //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);  // PWMB使能
      // 使能引脚


//      // 左轮正转


//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);      // AIN2=0
//   int qwe=580;
////      // 右轮正转
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, qwe);    // BIN1=PWM
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);      // BIN2=0
//	  printf("%d\n",qwe);

    /*
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
      // 按下按键时，执行导航测试
      HAL_Delay(200);  // 按键消抖
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
      {
        // 测试导航到坐标点(10.0, 10.0)
        navyTest(10.0f, 10.0f);
      }
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
