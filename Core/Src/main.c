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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "octospi.h"
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
#include "comm_protocol.h"
#include "nrf7002_test.h"
#include "nrf7002_fw.h"

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

// UART 수신 버퍼 (단일 바이트 수신용)
static uint8_t uart_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
extern void RTOS_UART_RxCallback(uint8_t byte);
extern void RTOS_ADC_ConvCpltCallback(void);
extern void RTOS_FDCAN_RxCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	LED2_toggle;
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
    //Sensor_UpdateCAN(&rx_header, rx_data);
    RTOS_FDCAN_RxCallback(&rx_header, rx_data);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
	{
		//LED1_toggle;
		//Sensor_UpdateADC(buf_adc1);
		RTOS_ADC_ConvCpltCallback();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {

    	RTOS_UART_RxCallback(uart_rx_byte);
        // DEBUG: Toggle LED2 to show RX interrupt is firing
        //LED2_toggle;

        // 수신된 바이트를 통신 프로토콜로 전달
        //Comm_RxByteCallback(uart_rx_byte);

        // 다음 바이트 수신 준비 - CRITICAL for continuous reception
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
        if (status != HAL_OK) {
            // If re-arming fails, force UART state reset and retry
            huart3.RxState = HAL_UART_STATE_READY;
            HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
        }
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
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */

  // nRF7002 (WM02C) 통신 테스트
  printf("\r\n\r\n*** Starting nRF7002 Communication Test ***\r\n");

  // 테스트 드라이버 초기화
  if (NRF70_Test_Init(&hospi1) != HAL_OK) {
      printf("ERROR: NRF70_Test_Init failed!\r\n");
  } else {
      // 풀 테스트 실행
      NRF70_TestResult_t test_result;
      bool all_passed = NRF70_RunFullTest(&test_result);

      // 결과 출력
      NRF70_PrintTestResult(&test_result);

      if (all_passed) {
          printf("\r\n*** nRF7002 Ready for Wi-Fi Driver Porting! ***\r\n");

          // 펌웨어 정보 출력
          NRF70_PrintFirmwareInfo(nrf7002_fw_bin, NRF7002_FW_SIZE);

          // 펌웨어 로딩 시도 (stub)
          NRF70_LoadFirmware(nrf7002_fw_bin, NRF7002_FW_SIZE);
      } else {
          printf("\r\n*** Check hardware connections ***\r\n");
      }
  }

  printf("\r\n");


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

  // USART 통신 프로토콜 초기화
  Comm_Init(&huart3);

  // UART 수신 인터럽트 시작 (명령 수신용)
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);

  system.state_level = SYSTEM_GO;
  LED1_on;

  // Send startup message via UART (proves TX works)
  const char *startup_msg = "\r\n=== SYSTEM READY - Commands enabled ===\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)startup_msg, strlen(startup_msg), 100);

  //printf("System initialized\r\n");
  //printf("Ready to receive commands via USART3\r\n");


#if CH0_ENABLE
  SMA_SetMode(CH_FOREARM_L, SMA_MODE_FORCE_CONTROL);
  SMA_SetTargetForce(CH_FOREARM_L, 50.0f);
  SMA_SetPIDGains(CH_FOREARM_L, 3.0f, 0.05f, 0.3f);
#endif

#if CH1_ENABLE
  SMA_SetMode(CH_FOREARM_R, SMA_MODE_FORCE_CONTROL);
  SMA_SetTargetForce(CH_FOREARM_R, 50.0f);
  SMA_SetPIDGains(CH_FOREARM_R, 3.0f, 0.05f, 0.3f);
#endif

#if CH2_ENABLE
  SMA_SetMode(CH_BICEPS_L, SMA_MODE_FORCE_CONTROL);
  SMA_SetTargetForce(CH_BICEPS_L, 50.0f);
  SMA_SetPIDGains(CH_BICEPS_L, 3.0f, 0.05f, 0.3f);
#endif

// #if CH3_ENABLE
//   SMA_SetMode(CH_BICEPS_R , SMA_MODE_FORCE_CONTROL);
//   SMA_SetTargetForce(CH_BICEPS_R , 50.0f);  
//   SMA_SetPIDGains(CH_BICEPS_R , 3.0f, 0.05f, 0.3f);
// #endif

#if CH3_ENABLE
  SMA_SetMode(CH_BICEPS_R, SMA_MODE_TEMP_CONTROL);
  SMA_SetTargetTemp(CH_BICEPS_R, 20.0f);  // 초기 목표 온도 20°C
  SMA_SetPIDGains(CH_BICEPS_R, 2.5f, 0.3f, 0.05f);
#endif

  // 활성화된 채널 초기 설정 (CH4, CH5 - 허리 L/R)
#if CH4_ENABLE
  SMA_SetMode(CH_WAIST_L, SMA_MODE_TEMP_CONTROL);
  SMA_SetTargetTemp(CH_WAIST_L, 20.0f);  // 초기 목표 온도 20°C
  SMA_SetPIDGains(CH_WAIST_L, 2.5f, 0.2f, 0.05f);
#endif

#if CH5_ENABLE
  SMA_SetMode(CH_WAIST_R, SMA_MODE_TEMP_CONTROL);
  SMA_SetTargetTemp(CH_WAIST_R, 20.0f);  // 초기 목표 온도 20°C
  SMA_SetPIDGains(CH_WAIST_R, 2.5f, 0.3f, 0.05f);
#endif

  // SMA 액추에이터 사용 예제
  // 예제 1: 오픈루프 제어 (채널 0을 50% PWM으로 설정)
  // SMA_SetMode(0, SMA_MODE_OPEN_LOOP);
  // SMA_SetPWM(0, 50.0f);

  // 예제 2: 온도 제어 (채널 1을 60°C로 제어)
  // SMA_SetMode(1, SMA_MODE_TEMP_CONTROL);
  // SMA_SetTargetTemp(1, 60.0f);
  // SMA_SetPIDGains(1, 5.0f, 0.1f, 0.5f);

  // 예제 3: 힘 제어 (채널 2를 50N 목표로 제어)
  // SMA_SetMode(2, SMA_MODE_FORCE_CONTROL);
  // SMA_SetTargetForce(2, 50.0f);
  // SMA_SetPIDGains(2, 3.0f, 0.05f, 0.3f);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_check = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// USART 명령 처리  → CommandTask로 이동됨
	//Comm_ProcessCommands();

	/* COMMENT OUT ALL ASCII DEBUG OUTPUT TO GET CLEAN BINARY TELEMETRY */

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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance != htim8.Instance) {
//        return;
    }
//
//    // SMA 액추에이터 제어 업데이트 (10ms 주기)
//    SMA_Update();
//
//    // 텔레메트리 전송 (80Hz = 4회마다 5번 전송)
//    // Pattern: send at 0,1,2,3 -> skip 4 -> repeat
//    // This gives 4 sends per 5 cycles = 80Hz exactly
//    static uint8_t telem_counter = 0;
//    if (telem_counter < 4) {  // Send first 4 out of every 5 cycles
//        Comm_SendTelemetry();
//    }
//    telem_counter++;
//    if (telem_counter >= 5) {
//        telem_counter = 0;
//    }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
