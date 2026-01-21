/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "stm32h7xx_hal.h"
#include "common_defs.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

__attribute__((unused)) static void FDCAN_Config_RX(FDCAN_HandleTypeDef* hfdcan, uint32_t *add_list, uint16_t add_list_size);
static void FDCAN_Config_RX_range(FDCAN_HandleTypeDef* hfdcan, uint32_t add_range1, uint32_t add_range2);
__attribute__((unused)) static void FDCAN_Config_TX_TDC(FDCAN_HandleTypeDef *hfdcan);

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 21;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 21;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 5;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 21;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 3;
  hfdcan2.Init.DataTimeSeg1 = 21;
  hfdcan2.Init.DataTimeSeg2 = 3;
  hfdcan2.Init.MessageRAMOffset = 1024;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 16;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */
    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = CAN2_RX_Pin|CAN2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, CAN1_RX_Pin|CAN1_TX_Pin);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN2_RX_Pin|CAN2_TX_Pin);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//---------------------------------------------------------------------------------------------------------------------------------------------
// 사용자 정의 함수들
//---------------------------------------------------------------------------------------------------------------------------------------------

// seungchan.2025.07.03 :  Tx 예제
// FDCAN_TxHeaderTypeDef tx_header;
// FDCAN_Config_TX(&tx_header, 0x400, FDCAN_DLC_BYTES_64);
// if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, buf_can1_pid.u8) != HAL_OK) {
//     Error_Handler();
// }

void FDCAN_Init(void)
{
    // CAN1: 변위센서 (0x100~0x10F)
    FDCAN_Config_RX_range(&hfdcan1, CAN1_RXID_DISP_START, CAN1_RXID_DISP_END);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN2: 힘센서 (0x001~0x003)
    FDCAN_Config_RX_range(&hfdcan2, CAN2_RXID_PWR_START, CAN2_RXID_PWR_END);
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN_Config_TX(FDCAN_TxHeaderTypeDef *fdcan_tx_header, uint32_t tx_add, uint32_t tx_byte)
{
    /* Prepare Tx Header */
    fdcan_tx_header->Identifier = tx_add;
    fdcan_tx_header->IdType = FDCAN_STANDARD_ID;
    fdcan_tx_header->TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header->DataLength = tx_byte;
    fdcan_tx_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header->BitRateSwitch = FDCAN_BRS_OFF;
    fdcan_tx_header->FDFormat = FDCAN_FD_CAN;
    fdcan_tx_header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header->MessageMarker = 0;
}

__attribute__((unused)) static void FDCAN_Config_RX(FDCAN_HandleTypeDef* hfdcan, uint32_t *add_list, uint16_t add_list_size)
{
    FDCAN_FilterTypeDef sFilterConfig;

    for (uint8_t i=0; i<add_list_size; i++)
    {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = i;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = add_list[i];
        sFilterConfig.FilterID2 = 0x7FF;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void FDCAN_Config_RX_range(FDCAN_HandleTypeDef* hfdcan, uint32_t add_range1, uint32_t add_range2)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = add_range1;
    sFilterConfig.FilterID2 = add_range2;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

__attribute__((unused)) static void FDCAN_Config_TX_TDC(FDCAN_HandleTypeDef *hfdcan)
{
    /* Configure and enable Tx Delay Compensation, required for BRS mode.
    TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
    TdcFilter default recommended value: 0 */
    uint32_t tdcOffset = hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1;
    if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, tdcOffset, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

// STM32H5 FDCAN Timing Calc. Func.
// float fdcanclk_mhz = 120.0f; // FDCAN(MHz)
// float nominal_bitrate_kbps = 1000.0f; //(kbps)
// float data_bitrate_kbps = 1000.0f; //(kbps)
// if (CalculateFDCANTiming(fdcanclk_mhz, nominal_bitrate_kbps, data_bitrate_kbps, &hfdcan1.Init) != 0) {
//     Error_Handler();
// }
int32_t CalculateFDCANTiming(float fdcanclk_mhz, float nominal_bitrate_kbps, float data_bitrate_kbps, FDCAN_InitTypeDef *init)
{
    if (fdcanclk_mhz <= 0.0f || nominal_bitrate_kbps <= 0.0f || data_bitrate_kbps <= 0.0f || init == NULL) {
        return -1;
    }

    float fdcanclk_hz = fdcanclk_mhz * 1e6;

    float nominal_bit_time = 1.0f / (nominal_bitrate_kbps * 1e3);
    uint32_t nominal_tq = 16; // 일반적인 TQ 수 (16~32 사이에서 선택)
    float nominal_tq_time = nominal_bit_time / nominal_tq;
    uint32_t nominal_prescaler = (uint32_t)(fdcanclk_hz * nominal_tq_time + 0.5f); // 반올림
    if (nominal_prescaler < 1 || nominal_prescaler > 512) {
        return -1;
    }

    float actual_nominal_tq_time = (float)nominal_prescaler / fdcanclk_hz;
    uint32_t total_nominal_tq = (uint32_t)(nominal_bit_time / actual_nominal_tq_time + 0.5f); // 반올림
    if (total_nominal_tq < 8 || total_nominal_tq > 128) {
        return -1;
    }

    init->NominalPrescaler = nominal_prescaler;
    init->NominalTimeSeg1 = (uint32_t)(total_nominal_tq * 0.875) - 1; // 샘플 포인트 87.5%
    init->NominalTimeSeg2 = total_nominal_tq - init->NominalTimeSeg1 - 1;
    init->NominalSyncJumpWidth = init->NominalTimeSeg2; // SJW = TSeg2
    if (init->NominalSyncJumpWidth < 1) {
        init->NominalSyncJumpWidth = 1;
    }

    if (data_bitrate_kbps > 8000) {
        data_bitrate_kbps = 8000;
    }
    float data_bit_time = 1.0f / (data_bitrate_kbps * 1e3);
    uint32_t data_tq = 16; // 일반적인 TQ 수
    float data_tq_time = data_bit_time / data_tq;
    uint32_t data_prescaler = (uint32_t)(fdcanclk_hz * data_tq_time + 0.5f); // 반올림
    if (data_prescaler < 1 || data_prescaler > 32) {
        return -1;
    }

    float actual_data_tq_time = (float)data_prescaler / fdcanclk_hz;
    uint32_t total_data_tq = (uint32_t)(data_bit_time / actual_data_tq_time + 0.5f); // 반올림
    if (total_data_tq < 8 || total_data_tq > 32) {
        return -1;
    }

    init->DataPrescaler = data_prescaler;
    init->DataTimeSeg1 = (uint32_t)(total_data_tq * 0.875) - 1; // 샘플 포인트 87.5%
    init->DataTimeSeg2 = total_data_tq - init->DataTimeSeg1 - 1;
    init->DataSyncJumpWidth = init->DataTimeSeg2; // SJW = TSeg2
    if (init->DataSyncJumpWidth < 1) {
        init->DataSyncJumpWidth = 1;
    }

    return 0;
}

/* USER CODE END 1 */
