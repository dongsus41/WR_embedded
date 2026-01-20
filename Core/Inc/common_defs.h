#ifndef __COMMON_DEFS_H
#define __COMMON_DEFS_H

#define UART_DEBUG					(huart3)
#define UART_DEBUG_ADD				(&huart3)

#define CAN1_RXID_DISP_START      0x100  // 변위센서 시작
#define CAN1_RXID_DISP_END        0x10F  // 0x10X 전체 받기

#define CAN2_RXID_PWR_START 		0x001
#define CAN2_RXID_PWR_END 			0x003

#define N_MOTION					12
#define N_TIMEBIN					2
#define CTRL_CH						6

#define SWAP32(d)	((((d) & 0xff000000) >> 24)|(((d) & 0x00ff0000) >> 8)|(((d) & 0x0000ff00) << 8)|(((d) & 0x000000ff) << 24))

#define GPIO_WRITE_0(PORTx, PINx)	(PORTx->BSRR = (uint32_t)PINx << 16U)
//#define GPIO_WRITE_0(PORTx, PINx)	(PORTx->BRR = PINx)
#define GPIO_WRITE_1(PORTx, PINx)	(PORTx->BSRR = PINx)
#define GPIO_TOGGLE(PORTx, PINx)	(PORTx->BSRR = (((PORTx->ODR) & PINx) << 16U) | (~(PORTx->ODR) & PINx))
#define GPIO_READ(GPIOx, GPIO_Pin)		(((GPIOx->IDR & GPIO_Pin) != 0x00U) ? 1U : 0U)

#define LED1_on				GPIO_WRITE_0(LED1_GPIO_Port, LED1_Pin)
#define LED1_off			GPIO_WRITE_1(LED1_GPIO_Port, LED1_Pin)
#define LED1_toggle			GPIO_TOGGLE(LED1_GPIO_Port, LED1_Pin)
#define LED2_on				GPIO_WRITE_0(LED2_GPIO_Port, LED2_Pin)
#define LED2_off			GPIO_WRITE_1(LED2_GPIO_Port, LED2_Pin)
#define LED2_toggle			GPIO_TOGGLE(LED2_GPIO_Port, LED2_Pin)

#define CH_FOREARM_L   0  // 전완 L
#define CH_FOREARM_R   1  // 전완 R
#define CH_BICEPS_L    2  // 이두 L
#define CH_BICEPS_R    3  // 이두 R
#define CH_WAIST_L    4  // 허리 L
#define CH_WAIST_R    5  // 허리 R

//-----------------------------------------------------------------------------
// DEVELOPER OPTIONS - Channel Enable/Disable
//-----------------------------------------------------------------------------
// Set to 1 to enable channel, 0 to disable
// Disabled channels will not be initialized or included in telemetry
#define CH0_ENABLE     0  // Channel 0 (Forearm L)
#define CH1_ENABLE     0  // Channel 1 (Forearm R)
#define CH2_ENABLE     0  // Channel 2 (Biceps L)
#define CH3_ENABLE     0  // Channel 3 (Biceps R)
#define CH4_ENABLE     1  // Channel 4 (Waist L) - ACTIVE
#define CH5_ENABLE     1  // Channel 5 (Waist R) - ACTIVE

// CAN sensor enable/disable
#define CAN_SENSORS_ENABLE  1  // Set to 1 to enable CAN force/displacement sensors

// Count active channels (for optimization)
#define ACTIVE_CHANNELS (CH0_ENABLE + CH1_ENABLE + CH2_ENABLE + CH3_ENABLE + CH4_ENABLE + CH5_ENABLE)

// Helper macro to check if a channel is enabled
#define IS_CHANNEL_ENABLED(ch) ( \
    ((ch) == 0 && CH0_ENABLE) || \
    ((ch) == 1 && CH1_ENABLE) || \
    ((ch) == 2 && CH2_ENABLE) || \
    ((ch) == 3 && CH3_ENABLE) || \
    ((ch) == 4 && CH4_ENABLE) || \
    ((ch) == 5 && CH5_ENABLE) \
)

//#define FSW0_on				GPIO_WRITE_1(FSW0_GPIO_Port, FSW0_Pin)
//#define FSW0_off			GPIO_WRITE_0(FSW0_GPIO_Port, FSW0_Pin)
//#define FSW1_on				GPIO_WRITE_1(FSW1_GPIO_Port, FSW1_Pin)
//#define FSW1_off			GPIO_WRITE_0(FSW1_GPIO_Port, FSW1_Pin)
//#define FSW2_on				GPIO_WRITE_1(FSW2_GPIO_Port, FSW2_Pin)
//#define FSW2_off			GPIO_WRITE_0(FSW2_GPIO_Port, FSW2_Pin)
//#define FSW3_on				GPIO_WRITE_1(FSW3_GPIO_Port, FSW3_Pin)
//#define FSW3_off			GPIO_WRITE_0(FSW3_GPIO_Port, FSW3_Pin)
//#define FSW4_on				GPIO_WRITE_1(FSW4_GPIO_Port, FSW4_Pin)
//#define FSW4_off			GPIO_WRITE_0(FSW4_GPIO_Port, FSW4_Pin)
//#define FSW5_on				GPIO_WRITE_1(FSW5_GPIO_Port, FSW5_Pin)
//#define FSW5_off			GPIO_WRITE_0(FSW5_GPIO_Port, FSW5_Pin)

//#define PWM0_TIM			htim1
//#define PWM1_TIM			htim1
//#define PWM2_TIM			htim1
//#define PWM3_TIM			htim1
//#define PWM4_TIM			htim2
//#define PWM5_TIM			htim2
//
//#define PWM0_TIM_CH			TIM_CHANNEL_1
//#define PWM1_TIM_CH			TIM_CHANNEL_2
//#define PWM2_TIM_CH			TIM_CHANNEL_3
//#define PWM3_TIM_CH			TIM_CHANNEL_4
//#define PWM4_TIM_CH			TIM_CHANNEL_1
//#define PWM5_TIM_CH			TIM_CHANNEL_2
//
//#define PWM0_TIM_CCR		CCR1
//#define PWM1_TIM_CCR		CCR2
//#define PWM2_TIM_CCR		CCR3
//#define PWM3_TIM_CCR		CCR4
//#define PWM4_TIM_CCR		CCR1
//#define PWM5_TIM_CCR		CCR2

//#define FSW_on(n)		((n != 0) ? ((n != 1) ? (((n != 2) ? (((n != 3) ? (((n != 4) ? FSW5_on : FSW4_on)) : FSW3_on)) : FSW2_on)) : FSW1_on) : FSW0_on)
//#define FSW_off(n)		((n != 0) ? ((n != 1) ? (((n != 2) ? (((n != 3) ? (((n != 4) ? FSW5_off : FSW4_off)) : FSW3_off)) : FSW2_off)) : FSW1_off) : FSW0_off)

#endif

