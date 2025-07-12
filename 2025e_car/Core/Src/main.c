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

// 视觉数据结构体定�???
typedef struct {
    float error_x;           // X方向误差
    float error_y;           // Y方向误差
    uint8_t target_detected; // 目标�???测标�???
    uint8_t data_ready;      // 数据就绪标志
} Vision_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 小车初始化函�???
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
  Servo_Init();
	
}
int jim =0;

// 视觉数据接收缓冲�???
uint8_t vision_rx_buffer[20]; // 视觉数据接收缓冲�???
Vision_Data_t vision_data = {0, 0, 0, 0}; // 视觉数据结构体实�???

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

    // // 处理视觉数据
    // if (vision_data.data_ready) {
    //     // 将视觉数据传递给舵机控制模块
    //     Mission_ProcessVisionData(vision_data.error_x, vision_data.error_y, vision_data.target_detected);
        
    //     // 清除数据就绪标志
    //     vision_data.data_ready = 0;
    // }
    
    // 更新舵机控制
    Servo_Update();
}

// // 解析视觉数据 这部分还要另外写�???个函数放在里�??? 放在这里不行
// void Parse_Vision_Data(uint8_t *data, uint8_t length)
// {
//     // �???单的解析示例，实际应根据视觉传感器的数据格式调整
//     // 假设数据格式�???: 帧头(1字节) + error_x(4字节) + error_y(4字节) + target_detected(1字节) + 校验(1字节)
//     if (length >= 11 && data[0] == 0xAA) { // 0xAA为帧�???
//         // 解析error_x（浮点数�???
//         float *px = (float*)(data + 1);
//         vision_data.error_x = *px;
        
//         // 解析error_y（浮点数�???
//         float *py = (float*)(data + 5);
//         vision_data.error_y = *py;
        
//         // 解析target_detected（布尔�?�）
//         vision_data.target_detected = data[9];
        
//         // 校验（简单示例，实际应根据需求实现）
//         uint8_t checksum = 0;
//         for (int i = 0; i < 10; i++) {
//             checksum += data[i];
//         }
        
//         if (checksum == data[10]) {
//             // 校验通过，设置数据就绪标�???
//             vision_data.data_ready = 1;
//         }
//     }
// }

// // UART接收回调函数
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     // 假设使用UART6接收视觉数据
//     if (huart->Instance == USART6) {
//         // 解析接收到的视觉数据
//         Parse_Vision_Data(vision_rx_buffer, sizeof(vision_rx_buffer));
        
//         // 重新启动接收
//         HAL_UART_Receive_DMA(huart, vision_rx_buffer, sizeof(vision_rx_buffer));
//     }
// }



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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  car_init();
  Uart_Init();
  //setNavigationParameters(0.5f, 20.0f, 45.0f); 
 //startNavigation(8,0);
  //navyTest();

  // 初始化任务系�???
  //Mission_Init();
  //HAL_Delay(5000);
//Servo_SetXAngle(180,10000);
//Servo_SetXAngle(180,20);
//SimpleMissionTest();
//navyTest();
//TestFireProcessing();
//TestCompleteFire1Mission();
//TestMissionStateMachine();
//TestVisionFeedback();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
    
   //OLED_ShowFloatNum(1, 1,IMU_data.YawZ, 4, 1, OLED_8X16);
//     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
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
