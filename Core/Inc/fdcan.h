/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief  FDCAN 초기화 함수 (필터 설정 및 시작)
 *         - FDCAN2: CAN2_RXID_PWR_START ~ CAN2_RXID_PWR_END 범위 필터 설정
 */
void FDCAN_Init(void);

/**
 * @brief  FDCAN 송신 헤더 설정
 * @param  fdcan_tx_header: 송신 헤더 구조체 포인터
 * @param  tx_add: CAN ID (11-bit Standard)
 * @param  tx_byte: 데이터 길이 (FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_64 등)
 */
void FDCAN_Config_TX(FDCAN_TxHeaderTypeDef *fdcan_tx_header, uint32_t tx_add, uint32_t tx_byte);

/**
 * @brief  FDCAN 비트레이트 타이밍 자동 계산
 * @param  fdcanclk_mhz: FDCAN 클럭 주파수 (MHz)
 * @param  nominal_bitrate_kbps: Nominal 비트레이트 (kbps)
 * @param  data_bitrate_kbps: Data 비트레이트 (kbps, FD-CAN only)
 * @param  init: FDCAN_InitTypeDef 구조체 포인터 (결과 저장)
 * @retval 0: 성공, -1: 실패
 *
 * @example
 *   float fdcanclk_mhz = 120.0f;
 *   float nominal_bitrate_kbps = 1000.0f;  // 1 Mbps
 *   float data_bitrate_kbps = 2000.0f;     // 2 Mbps (FD mode)
 *   if (CalculateFDCANTiming(fdcanclk_mhz, nominal_bitrate_kbps,
 *                            data_bitrate_kbps, &hfdcan1.Init) != 0) {
 *       Error_Handler();
 *   }
 */
int32_t CalculateFDCANTiming(float fdcanclk_mhz, float nominal_bitrate_kbps,
                             float data_bitrate_kbps, FDCAN_InitTypeDef *init);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

