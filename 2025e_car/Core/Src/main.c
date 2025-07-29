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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
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

// 视觉数据结构体定�????????????
typedef struct {
    float error_x;           // X方向误差
    float error_y;           // Y方向误差
    uint8_t target_detected; // 目标�????????????测标�????????????
    uint8_t data_ready;      // 数据就绪标志
} Vision_Data_t;
extern SPI_HandleTypeDef hspi1;  // ??SPI??

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 任务状�?�标�???
static uint8_t mission_started = 0;  // 任务是否已启动标�???
static uint8_t last_fire_id = 0;     // 上次接收到的火源ID
// ???????4KB?
uint32_t sector_addr = 0x000000;  // ??????
//uint32_t write_addr = 0x000000;  // ????

// 小车初始化函�????????????
void car_init(void)
{

  Motor_PWM_StartAll();//TIM1 pwm11
  HAL_TIM_Base_Start_IT(&htim2);//1msĺŽćś
  OLED_Init();
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
	Mission_Init();

  // __HAL_RCC_SPI1_CLK_ENABLE();
  // W25Q128_Init(&hspi1);
  // W25Q128_SectorErase(sector_addr);


}

// �???查并处理飞机发�?�的火源ID 
void Check_And_Start_Mission(void)
{
    // �???查是否接收到有效的火源ID
    if (drone_data.fire_id != 0 && drone_data.fire_id != last_fire_id)
    {
        // 接收到新的火源ID
        last_fire_id = drone_data.fire_id;

        // 验证火源ID是否在有效范围内
        if (drone_data.fire_id >= 1 && drone_data.fire_id <= 6)
        {
            // 尝试启动对应的灭火任�???
            uint8_t result = Mission_StartByFireId(drone_data.fire_id);

            switch (result)
            {
            case 0:
                // 任务启动失败（无效ID或未定义任务�???
                // 可以在这里添加错误指示，比如LED闪烁
               // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // 错误指示
                break;
            case 1:
                // 任务成功启动
                mission_started = 1;
                // 可以在这里添加成功指示，比如LED常亮
                //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // 成功指示
                break;
            case 2:
                // 相同任务已在执行�???
                mission_started = 1; // 确保标志位正�???
                break;
            }
        }
    }
}



int jim =0;
uint8_t ftest=0;
uint32_t write_addr = 0x001000;  // Flash????
uint32_t flash_id = 0;           // Flash ID????
uint8_t flash_byte1 = 0;         // Flash ID?1???
uint8_t flash_byte2 = 0;         // Flash ID?2???
uint8_t flash_byte3 = 0;         // Flash ID?3???
uint8_t spi_status_code = 0;     // SPI???


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

  // 1. ć´ć°ĺ˝ĺä˝ç˝Žĺć 
  updatePosition();
 	//HAL_UART_Transmit(&huart3, (uint8_t *)&jim, 1, HAL_MAX_DELAY);
  // 2. ć´ć°ĺŻźčŞć§ĺś
  updateNavigation_control();
  
  // 3. ć§čĄPIDć§ĺśčŽĄçŽĺçľćşčž???????????????
  // ĺĺŤéĺşŚçŻĺč˝ŹĺçŻçć§ĺś
  //wheels_pid_control_auto_with_yaw();
	wheels_pid_control_auto();
  // 4. ĺŻäťĽćˇťĺ ĺśäťä˝é˘äťťĺĄďźĺŚLEDçś?ć´ć°?ćéŽćŁćľç­
  // čżéććśä¸ćˇťĺ ĺśäťäťť???????????????
	//printf("vision:%f,%f\n",vision_data.error_x,vision_data.error_y);

    printf("count:%d,%d\n",left_encoder_count,right_encoder_count);
   // printf("%f,%f\n", left_wheel_speed, right_wheel_speed);
   //printf("%f\n", IMU_data.YawZ);
    // 更新舵机控制
    Servo_Update(); //已在Mission_Update();调用 3 记得改回�??????
    	//printf("%f\n",IMU_data.YawZ);

  //update_rotation_task();

      // 10Hz任务计数�??
    static uint8_t counter_10hz = 0;
    counter_10hz++;
    if (counter_10hz >= 10) // �??10次调用，�??100ms
    {
        counter_10hz = 0;
     // drone_display_task(); 
          //OLED_ShowNum(10,5,111,2,16,0);

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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  car_init();
  Uart_Init();
  //init_example();
 // drone_data.drone_x=27.0f;
  //Mission_StartFire1();

  HAL_Delay(1000);


  //vision_data.target_detected = 0;//测试�???

  // 可�?�：测试用，模拟接
  
  //收火源ID（调试时使用�???
  // Test_Fire_ID_Reception(1);  // 取消注释来测试火�???1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // �???查并处理飞机发�?�的火源ID（只在任务未启动时检查）
    Process_Drone_Data();
	  Key();
    if (!mission_started)
    {
        Check_And_Start_Mission();
    }
      if (delay_ms_nb_id(500, 5)) 
      {  // 使用ID=3的延�??

          //drone_data.fire_id=2;
          //drone_display_task();
          //set_image_aph(drone_data.fire_id, 127); // 显示火源1
          draw_fly((int)drone_data.drone_x, (int)drone_data.drone_y);
              //display_coordinates("main.t0", drone_data.drone_x, drone_data.drone_y, 0);  // 假设t0是坐标显示控�?
          // Flash????????????Flash
          ftest++;

          // ??????????????????ftest=1????
                  // if (ftest == 1) {
                  //     W25Q128_SectorErase(write_addr);
                  // }

                  // // ??ftest?Flash
                  // W25Q128_PageProgram(write_addr, (uint8_t*)&ftest, sizeof(ftest));
	     // set_image_aph(3, 0); // 显示火源1
      }
//    OLED_ShowNum(10,1,drone_data.drone_x,5,16,0); 
//    OLED_ShowNum(30,1,drone_data.fire_id,5,16,0); 
//    OLED_ShowNum(10,3,drone_data.drone_y,5,16,0); 
//        OLED_ShowNum(10,5,fire_x_f,5,16,0);
//    OLED_ShowNum(30,5,fire_y_f,5,16,0);
	 // OLED_ShowNum(10,1,Key_KeyNumber,5,16,0); 

	
	
	  
   // OLED_ShowNum(10,1,left_wheel_speed,5,16,0);
   //  OLED_ShowNum(10,3,right_wheel_speed,5,16,0); 
    // printf("%f,%f\n",left_wheel_speed,right_wheel_speed);
	//	navyTest();
    // 更新任务状�?�机
    Mission_Update();

    // 可�?�：显示调试信息
     OLED_ShowNum(10,1,vision_data.error_x,10,16,0);
    OLED_ShowNum(10,4,vision_data.error_y,10,16,0);

     // ??SPI???Flash ID?????0x90???
    //  OLED_ShowNum(10,5,spi_status_code,1,16,0);  // SPI?? (0=??)
    //  OLED_ShowNum(30,5,flash_byte1,3,16,0);      // ???ID
      //OLED_ShowNum(60,5,IMU_data.YawZ,3,16,0);      // ??ID
    // OLED_ShowNum(90,5,flash_id,5,16,0);         // ??ID?
    // OLED_ShowNum(10,6,drone_data.fire_id,1,16,0);  // 显示接收到的火源ID

    // Display_DebugStatus();
    // Waypoint_Update();
       //HAL_Delay(100);  // 添加延时，降低刷新频�???????

// Motor_PWM_SetRight(300);
// Motor_PWM_SetLeft(100);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
