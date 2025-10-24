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
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "common_defs.h"
#include "debug.h"
#include "fdcan.h"
#include "emc2303.h"
#include "sensors.h"
#include "sma_actuator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern char log_msg[MAX_LOG_MSG_LEN];
System_typedef system;
__attribute__((section(".RAM_D2"))) __attribute__((aligned(4)))
volatile uint16_t buf_adc1[CTRL_CH] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
    {
        return;
    }

    /* Retrieve Rx messages from RX FIFO0 */
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    //force sensor update
    Sensor_UpdateCAN(&rx_header, rx_data);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
	{
		LED1_toggle;
		Sensor_UpdateADC(buf_adc1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != htim8.Instance) {
        return;
    }

    static uint32_t t = 0;

    // SMA 액추에이터 제어 업데이트 (10ms 주기)
    SMA_Update();

    if (t++ >= 5-1) {  // tick: 100Hz*5 --> 50ms
        t = 0;

        const SensorData_t *sensor = Sensor_GetData();

        snprintf(log_msg, sizeof(log_msg),
                 "PWR: %u %u %u %u | BioTorq: %u %u %u %u\r\n",
                 sensor->can.pwr[0], sensor->can.pwr[1],
                 sensor->can.pwr[2], sensor->can.pwr[3],
                 sensor->can.biotorq[0], sensor->can.biotorq[1],
                 sensor->can.biotorq[2], sensor->can.biotorq[3]);
        LogFifo_Push(log_msg);

        snprintf(log_msg, sizeof(log_msg),
                 "TEMP: %.1f %.1f %.1f %.1f %.1f %.1f\r\n",
                 sensor->adc.temp_c[0], sensor->adc.temp_c[1],
                 sensor->adc.temp_c[2], sensor->adc.temp_c[3],
                 sensor->adc.temp_c[4], sensor->adc.temp_c[5]);
        LogFifo_Push(log_msg);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdin, NULL);
  RegisterErrorCallback(MyErrorCallback);
//RegisterErrorCallback(MyErrorCallback_Msg);
//RegisterErrorCallback(MyErrorCallback_Code_Msg);
  LogFifo_Init();
  DWT_Init();

  Sensor_Init();

  GPIO_WRITE_0(CAN2_STB_GPIO_Port, CAN2_STB_Pin);
  FDCAN_Init();
  HAL_Delay(200);

  HAL_ADCEx_LinearCalibration_FactorLoad(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_Delay(10);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buf_adc1, 6);

  uint32_t target_hz = 100u;
  emc_profile_t profile = EMC_PROFILE_POWER_GATING;
//emc_profile_t profile = EMC_PROFILE_4WIRE_FAN;
  Fans6_Init(profile, target_hz);
//Fan6_SetDuty(1u, 10.0f);

//HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start_IT(&htim8);

  // SMA 액추에이터 컨트롤러 초기화 (제어 주기: 10ms)
  SMA_Init(10);

  system.state_level = SYSTEM_GO;
  LED1_on;
  printf("System initialized\r\n");

  // SMA 액추에이터 사용 예제
  // 예제 1: 오픈루프 제어 (채널 0을 50% PWM으로 설정)
  // SMA_SetMode(0, SMA_MODE_OPEN_LOOP);
  // SMA_SetPWM(0, 50.0f);

  // 예제 2: 온도 제어 (채널 1을 60°C로 제어)
  // SMA_SetMode(1, SMA_MODE_TEMP_CONTROL);
  // SMA_SetTargetTemp(1, 60.0f);
  // SMA_SetPIDGains(1, 5.0f, 0.1f, 0.5f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_check = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	while (LogFifo_Pop(log_msg) == 0)
	{
		printf("%s", log_msg);
	}
	uint32_t now = HAL_GetTick();
	if ((now - last_check) >= 1000) {
		last_check = now;

		// sensor timeout check (500ms)
	    if (Sensor_CheckTimeout(50)) {
	    	printf("WARNING: Sensor timeout!\r\n");
	        }

	    // can validity check
	    if (!Sensor_IsCANDataValid()) {
	    	printf("WARNING: CAN data incomplete!\r\n");
	        }

	    // sensor failure check
	    for (uint8_t i = 0; i < SENSOR_TEMP_CH; i++) {
	    	if (Sensor_IsTempSensorFault(i)) {
	    		printf("ERROR: Temp sensor CH%u fault!\r\n", i);
	    	}
	    }
	}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
