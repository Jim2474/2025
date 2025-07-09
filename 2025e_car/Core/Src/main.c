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
//ĺ°č˝Śĺĺ§???????????????
void car_init(void)
{
  
  Motor_PWM_StartAll();//TIM1 pwm11
  HAL_TIM_Base_Start_IT(&htim2);//1msĺŽćś
  OLED_Init();
  OLED_Clear();
  // ĺŻĺ¨ĺˇŚč˝Žçźç ???????????????(TIM3)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // ĺŻĺ¨ĺłč˝Žçźç ???????????????(TIM4)
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  Uart_Init();
      
  // ĺĺ§ĺPIDć§ĺś???????????????
  pid_init_all();
  // čŽžç˝Žĺĺ§çŽć éĺşŚ0
  set_target_speed(0.0f, 0.0f);
  // ĺĺ§ĺĺŻźčŞçłť???????????????
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
  // 1000HzäťťĺĄďźćŻ1msć§čĄ??????????????????????????????
  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
  // 1. ć´ć°çźç ĺ¨čŽĄć°ĺč˝Ź?čŽĄ???????????????
  encoder_count();
  // 2. čŽĄçŽçşż?ĺşŚ 
  Calculate_speed(left_wheel_rpm, right_wheel_rpm);
}

void TIM2_Task_100Hz(void)
{
  // 100HzäťťĺĄďźćŻ10msć§čĄ??????????????????????????????
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);

//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
  // 1. ć´ć°ĺ˝ĺä˝ç˝Žĺć 
  updatePosition();
 	//HAL_UART_Transmit(&huart3, (uint8_t *)&jim, 1, HAL_MAX_DELAY);
//printf("%f,%f\n",g_left_target_speed,g_right_target_speed);
  // 2. ć´ć°ĺŻźčŞć§ĺś
  updateNavigation_control();
  
  // 3. ć§čĄPIDć§ĺśčŽĄçŽĺçľćşčž???????????????
  // ĺĺŤéĺşŚçŻĺč˝ŹĺçŻçć§ĺś
  //wheels_pid_control_auto_with_yaw();
	wheels_pid_control_auto();
  // 4. ĺŻäťĽćˇťĺ ĺśäťä˝é˘äťťĺĄďźĺŚLEDçś?ć´ć°?ćéŽćŁćľç­
  // čżéććśä¸ćˇťĺ ĺśäťäťť???????????????

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
    {8.0f, 0.0f},   // çŹŹä¸ä¸ŞéĄśçš
    {8.0f, 8.0f},  // çŹŹäşä¸ŞéĄśçš
    {0.0f, 8.0f}    // çŹŹä¸ä¸ŞéĄśçš
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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  car_init();
  Uart_Init();
  //setNavigationParameters(0.5f, 20.0f, 45.0f); 
 //startNavigation(8,0);
navyTest();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
    
   //OLED_ShowFloatNum(1, 1,IMU_data.YawZ, 4, 1, OLED_8X16);
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
//       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
//           __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 80);
//              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 50);




    /*
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
    {
      // ćä¸ćéŽćśďźć§čĄĺŻźčŞćľčŻ
      HAL_Delay(200);  // ćéŽćść
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
      {
        // ćľčŻĺŻźčŞĺ°ĺć çš(10.0, 10.0)
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
