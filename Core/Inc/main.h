/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "common_defs.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define ALERT_Pin GPIO_PIN_15
#define ALERT_GPIO_Port GPIOC
#define SW_Pin GPIO_PIN_5
#define SW_GPIO_Port GPIOA
#define WM_IRQ_Pin GPIO_PIN_4
#define WM_IRQ_GPIO_Port GPIOC
#define WM_BUCK_Pin GPIO_PIN_5
#define WM_BUCK_GPIO_Port GPIOC
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define CAN2_STB_Pin GPIO_PIN_14
#define CAN2_STB_GPIO_Port GPIOB
#define CAN2_DIR_Pin GPIO_PIN_15
#define CAN2_DIR_GPIO_Port GPIOB
#define CAN2_RESET_Pin GPIO_PIN_6
#define CAN2_RESET_GPIO_Port GPIOC
#define CAN2_SYNC_Pin GPIO_PIN_7
#define CAN2_SYNC_GPIO_Port GPIOC
#define CAN1_DIR_Pin GPIO_PIN_9
#define CAN1_DIR_GPIO_Port GPIOC
#define CAN1_RESET_Pin GPIO_PIN_8
#define CAN1_RESET_GPIO_Port GPIOA
#define CAN1_SYNC_Pin GPIO_PIN_9
#define CAN1_SYNC_GPIO_Port GPIOA
#define CAN1_STB_Pin GPIO_PIN_10
#define CAN1_STB_GPIO_Port GPIOA
#define CAN1_RX_Pin GPIO_PIN_11
#define CAN1_RX_GPIO_Port GPIOA
#define CAN1_TX_Pin GPIO_PIN_12
#define CAN1_TX_GPIO_Port GPIOA
#define BZ_Pin GPIO_PIN_12
#define BZ_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
typedef enum {
	SYSTEM_INIT		= 0,
	SYSTEM_READY	= 1,
	SYSTEM_GO		= 2
}System_State_typedef;

//#define TX_BYTE_FDCAN				24
#define TX_BYTE_USART				24
#define TX_BYTE_MONITOR				(CTRL_CH*4)
//#define TX_BYTE_RESERVED			(TX_BYTE_FDCAN-TX_BYTE_MONITOR)
#define TX_BYTE_RESERVED			(TX_BYTE_USART-TX_BYTE_MONITOR)

//typedef struct {
//	uint8_t pwm[CTRL_CH];
//	uint8_t fan[CTRL_CH];
//	uint16_t temp[CTRL_CH];
////	uint8_t reserved[TX_BYTE_RESERVED];
//}Buf_FDCAN_Tx_typedef;
//
//typedef union {
//	uint8_t uint8[TX_BYTE_FDCAN];
//	Buf_FDCAN_Tx_typedef struc;
//}Buf_FDCAN_Tx_Union_typedef;

typedef struct {
	uint8_t pwm[CTRL_CH];
	uint8_t fan[CTRL_CH];
	uint16_t temp[CTRL_CH];
//	uint8_t reserved[TX_BYTE_RESERVED];
}Buf_USART_Tx_typedef;

typedef union {
	uint8_t uint8[TX_BYTE_USART];
	Buf_USART_Tx_typedef struc;
}Buf_USART_Tx_Union_typedef;


typedef struct {
	uint8_t pwm[CTRL_CH];
	uint8_t fan[CTRL_CH];
	uint8_t ctrl_mode[CTRL_CH];
	uint8_t enable_pid[CTRL_CH];
	uint16_t target_temp[CTRL_CH];
	uint16_t target_force[CTRL_CH];
	uint16_t target_displacement[CTRL_CH];
}Ctrl_Param_typedef;


typedef struct {
	System_State_typedef state_level;
	//Buf_FDCAN_Tx_Union_typedef buf_fdcan_tx;
	Buf_USART_Tx_Union_typedef buf_usart_tx;
	Ctrl_Param_typedef ctrl_param_now;
	Ctrl_Param_typedef ctrl_param_save;
	volatile uint32_t* pnt_pwm[CTRL_CH];
	uint8_t lock_motion;
	uint8_t state_fsw[CTRL_CH];
	uint8_t state_pwm[CTRL_CH];
	float state_temp[CTRL_CH];
	uint16_t state_temp_raw[CTRL_CH];
//	uint8_t flag_first_param;
//	uint8_t flag_lock_motion;
	char uart_char[100];
	uint16_t n_rx_motion;
	uint16_t n_rx_motion_limit;
	uint16_t t_count;
	uint16_t t_target;
	uint32_t t_sec;
} System_typedef;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
