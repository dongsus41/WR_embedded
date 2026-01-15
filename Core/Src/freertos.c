/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "sma_actuator.h"
#include "sensors.h"
#include "comm_protocol.h"
#include "emc2303.h"
#include "debug.h"
#include "common_defs.h"

#include "nrf7002_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_TASK_PERIOD_MS      10      // 100Hz 제어 루프
#define TELEMETRY_TASK_PERIOD_MS    12      // ~80Hz 텔레메트리 (12.5ms)
#define CMD_BUFFER_SIZE             128     // 명령 버퍼 크기
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define NOTIFY_CONTROL_TICK     (1UL << 0)
#define NOTIFY_ADC_COMPLETE     (1UL << 1)
#define NOTIFY_CAN_RX           (1UL << 2)

// UART 수신 버퍼 (명령 파싱용)
static char cmd_rx_buffer[CMD_BUFFER_SIZE];
static uint16_t cmd_rx_index = 0;

/* USER CODE END Variables */
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for NRF70_Task */
osThreadId_t NRF70_TaskHandle;
const osThreadAttr_t NRF70_Task_attributes = {
  .name = "NRF70_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for uartRxQueue */
osMessageQueueId_t uartRxQueueHandle;
const osMessageQueueAttr_t uartRxQueue_attributes = {
  .name = "uartRxQueue"
};
/* Definitions for cmdQueue */
osMessageQueueId_t cmdQueueHandle;
const osMessageQueueAttr_t cmdQueue_attributes = {
  .name = "cmdQueue"
};
/* Definitions for sensorDataMutex */
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};
/* Definitions for smaChannelMutex */
osMutexId_t smaChannelMutexHandle;
const osMutexAttr_t smaChannelMutex_attributes = {
  .name = "smaChannelMutex"
};
/* Definitions for uartTxMutex */
osMutexId_t uartTxMutexHandle;
const osMutexAttr_t uartTxMutex_attributes = {
  .name = "uartTxMutex"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ProcessReceivedByte(uint8_t byte);
/* USER CODE END FunctionPrototypes */

void StartCommandTask(void *argument);
void StartControlTask(void *argument);
void StartTelemetryTask(void *argument);
void NRF70_TestTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

    // 스택 오버플로우 발생 시 LED 점멸 또는 로그 출력
    // 주의: 이 함수에서는 블로킹 함수 사용 불가
    __disable_irq();
    while(1) {
        // LED 토글로 오류 표시 (디버깅용)
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        for(volatile uint32_t i = 0; i < 1000000; i++);
    }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of sensorDataMutex */
  sensorDataMutexHandle = osMutexNew(&sensorDataMutex_attributes);

  /* creation of smaChannelMutex */
  smaChannelMutexHandle = osMutexNew(&smaChannelMutex_attributes);

  /* creation of uartTxMutex */
  uartTxMutexHandle = osMutexNew(&uartTxMutex_attributes);

  /* creation of i2cMutex */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartRxQueue */
  uartRxQueueHandle = osMessageQueueNew (128, 1, &uartRxQueue_attributes);

  /* creation of cmdQueue */
  cmdQueueHandle = osMessageQueueNew (4, 128, &cmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of NRF70_Task */
  NRF70_TaskHandle = osThreadNew(NRF70_TestTask, NULL, &NRF70_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCommandTask */
/**
  * @brief  Function implementing the CommandTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommandTask */
__weak void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
    uint8_t rx_byte;
    char cmd_string[CMD_BUFFER_SIZE];

  /* Infinite loop */
    for(;;)
    {
        // uartRxQueue에서 바이트 수신 대기 (블로킹)
        if (osMessageQueueGet(uartRxQueueHandle, &rx_byte, NULL, osWaitForever) == osOK)
        {
            // LED 토글 (디버그용 - 수신 확인)
            LED2_toggle;

            // 수신된 바이트 처리
            ProcessReceivedByte(rx_byte);
        }

        // cmdQueue에서 완성된 명령 확인 (논블로킹)
        if (osMessageQueueGet(cmdQueueHandle, cmd_string, NULL, 0) == osOK)
        {
            // 명령 파싱 및 실행
            ParsedCommand_t parsed_cmd;

            if (Comm_ParseCommand(cmd_string, &parsed_cmd) == 0)
            {
                // [수정] Comm_ExecuteCommand 내부에서 mutex 획득하므로
                // 여기서는 mutex 획득하지 않음
                if (Comm_ExecuteCommand(&parsed_cmd) == 0) {
                    Comm_SendResponse("OK\r\n");
                } else {
                    Comm_SendResponse("ERROR: Execution failed\r\n");
                }
            }
            else
            {
                Comm_SendResponse("ERROR: Invalid command\r\n");
            }
        }
    }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @note [수정됨] 제어 로직이 여기에 있어야 함 (PRD 기준)
*/
/* USER CODE END Header_StartControlTask */
__weak void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS);  // 10ms

    // 현재 틱 저장 (주기적 실행 기준점)
    xLastWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for(;;)
    {
        // 정확한 10ms 주기로 실행 (vTaskDelayUntil 사용)
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // LED1 토글 (제어 루프 동작 확인용)
        LED1_toggle;

        // 센서 데이터 mutex 획득 후 온도 읽기
        if (osMutexAcquire(sensorDataMutexHandle, 5) == osOK)
        {
            // 센서 데이터는 ISR에서 이미 업데이트됨
            // 여기서는 읽기만 수행
            osMutexRelease(sensorDataMutexHandle);
        }

        // SMA 제어 mutex 획득 후 제어 업데이트
        if (osMutexAcquire(smaChannelMutexHandle, 5) == osOK)
        {
            // SMA 액추에이터 제어 업데이트 (PID + PWM)
            SMA_Update();

            osMutexRelease(smaChannelMutexHandle);
        }

        static uint8_t fan_update_counter = 0;
        // I2C mutex 획득 후 팬 제어 (100ms마다 = 10회 중 1회)
        fan_update_counter++;
        if (fan_update_counter >= 10)
        {
            fan_update_counter = 0;

			// I2C mutex 획득 후 팬 제어
			if (osMutexAcquire(i2cMutexHandle, 5) == osOK)
			{
				// 팬 자동 제어 업데이트
				SMA_UpdateFans();

				osMutexRelease(i2cMutexHandle);
			}
        }
    }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @note [수정됨] 텔레메트리 전송만 담당 (PRD 기준)
*/
/* USER CODE END Header_StartTelemetryTask */
__weak void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
    TickType_t xLastWakeTime;
    // [수정] 올바른 주기 상수 사용
    const TickType_t xPeriod = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);  // 12ms (~80Hz)

    // 현재 틱 저장 (주기적 실행 기준점)
    xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        // 정확한 12ms 주기로 실행
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // [수정] 텔레메트리 전송 추가
        // uartTxMutex는 Comm_SendTelemetry_Safe() 내부에서 획득
        Comm_SendTelemetry_Safe();
    }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_NRF70_TestTask */
/**
* @brief Function implementing the NRF70_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRF70_TestTask */
void NRF70_TestTask(void *argument)
{
  /* USER CODE BEGIN NRF70_TestTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NRF70_TestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief UART 수신 바이트 처리 (명령 라인 조립)
 * @param byte 수신된 바이트
 */
static void ProcessReceivedByte(uint8_t byte)
{
    // 명령 종료 문자 감지 ('\r' 또는 '\n')
    if (byte == '\r' || byte == '\n')
    {
        if (cmd_rx_index > 0)
        {
            // Null terminator 추가
            cmd_rx_buffer[cmd_rx_index] = '\0';

            // 완성된 명령을 cmdQueue에 전송
            osMessageQueuePut(cmdQueueHandle, cmd_rx_buffer, 0, 0);

            // 버퍼 리셋
            cmd_rx_index = 0;
            memset(cmd_rx_buffer, 0, sizeof(cmd_rx_buffer));
        }
        return;
    }

    // 버퍼 오버플로우 방지
    if (cmd_rx_index >= (CMD_BUFFER_SIZE - 1))
    {
        cmd_rx_index = 0;
        memset(cmd_rx_buffer, 0, sizeof(cmd_rx_buffer));
        return;
    }

    // 문자 추가
    cmd_rx_buffer[cmd_rx_index++] = (char)byte;
}

/* ============================================================================
 * ISR에서 호출할 함수들 (main.c의 콜백에서 사용)
 * ============================================================================ */

/**
 * @brief UART 수신 완료 콜백에서 호출 (ISR Context)
 * @param byte 수신된 바이트
 */
void RTOS_UART_RxCallback(uint8_t byte)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ISR에서 큐에 바이트 전송
    xQueueSendFromISR(uartRxQueueHandle, &byte, &xHigherPriorityTaskWoken);

    // 높은 우선순위 태스크가 깨어났으면 컨텍스트 스위칭
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief ADC 변환 완료 콜백에서 호출 (ISR Context)
 */
void RTOS_ADC_ConvCpltCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 센서 데이터 업데이트 (ISR에서 직접 수행 - 빠른 처리)
    extern volatile uint16_t buf_adc1[];
    Sensor_UpdateADC(buf_adc1);

    // ControlTask에 알림 (선택적)
    // xTaskNotifyFromISR(ControlTaskHandle, NOTIFY_ADC_COMPLETE,
    //                    eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief FDCAN 수신 콜백에서 호출 (ISR Context)
 * @param rx_header FDCAN 헤더
 * @param rx_data 수신 데이터
 */
void RTOS_FDCAN_RxCallback(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 센서 데이터 업데이트 (ISR에서 직접 수행)
    Sensor_UpdateCAN(rx_header, rx_data);

    // ControlTask에 알림 (선택적)
    // xTaskNotifyFromISR(ControlTaskHandle, NOTIFY_CAN_RX,
    //                    eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




/* USER CODE END Application */

