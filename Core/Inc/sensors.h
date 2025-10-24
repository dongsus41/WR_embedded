#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* ========== 센서 채널 정의 ========== */
#define SENSOR_MAX_CH           4   // CAN 센서 최대 채널 (4관절)
#define SENSOR_TEMP_CH          6   // 온도 센서 최대 채널

/* ========== CAN ID 정의 ========== */
#define CAN_ID_PWR_FOREARM      0x001  // CAN2_RXID_PWR_START
#define CAN_ID_PWR_BICEPS       0x002  // CAN2_RXID_PWR_START + 1
#define CAN_ID_PWR_CAP          0x003  // CAN2_RXID_PWR_START + 2

/* ========== 센서 채널 인덱스 ========== */
typedef enum {
    SENSOR_CH_FOREARM_L = 0,  // 전완 L
    SENSOR_CH_FOREARM_R = 1,  // 전완 R
    SENSOR_CH_BICEPS_L  = 2,  // 이두 L
    SENSOR_CH_BICEPS_R  = 3,  // 이두 R
} SensorChannel_t;

/* ========== 온도 센서 한계값 ========== */
#define TEMP_MIN_VALID          -40.0f   // TMP235 최소 동작 온도
#define TEMP_MAX_VALID          125.0f   // TMP235 최대 동작 온도
#define TEMP_WARN_THRESHOLD     80.0f    // 경고 온도
#define TEMP_CRIT_THRESHOLD     100.0f   // 위험 온도

/* ========== CAN 센서 데이터 구조체 ========== */
typedef struct {
    uint16_t pwr[SENSOR_MAX_CH];        // 전력 (W)
    uint16_t biotorq[SENSOR_MAX_CH];    // 생체 토크
    uint16_t cap[SENSOR_MAX_CH];        // 정전용량
    uint32_t timestamp;                 // 마지막 업데이트 시간 (ms)
    uint8_t  valid_flag;                // 데이터 유효성 플래그 (비트마스크)
} CANSensorData_t;

/* ========== ADC 온도 센서 데이터 구조체 ========== */
typedef struct {
    uint16_t raw[SENSOR_TEMP_CH];       // ADC Raw 값 (0~65535)
    float    temp_c[SENSOR_TEMP_CH];    // 변환된 온도 (°C)
    uint8_t  sensor_fault[SENSOR_TEMP_CH]; // 센서 오류 플래그 (0=정상, 1=오류)
    uint32_t timestamp;                 // 마지막 업데이트 시간 (ms)
} ADCSensorData_t;

/* ========== 통합 센서 데이터 구조체 ========== */
typedef struct {
    CANSensorData_t can;    // CAN 버스 센서 데이터
    ADCSensorData_t adc;    // ADC 온도 센서 데이터
} SensorData_t;

/* ========== 공용 API ========== */

/**
 * @brief 센서 데이터 모듈 초기화
 */
void Sensor_Init(void);

/**
 * @brief CAN 수신 데이터 업데이트
 * @param rx_header FDCAN 수신 헤더
 * @param rx_data 수신 데이터 버퍼 (8바이트)
 * @return 0=성공, -1=실패
 */
int32_t Sensor_UpdateCAN(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

/**
 * @brief ADC 온도 데이터 업데이트
 * @param adc_buffer ADC DMA 버퍼 (6채널)
 */
void Sensor_UpdateADC(volatile uint16_t *adc_buffer);

/**
 * @brief 전체 센서 데이터 읽기 (읽기 전용)
 * @return 센서 데이터 구조체 포인터 (const)
 */
const SensorData_t* Sensor_GetData(void);

/**
 * @brief 특정 채널의 CAN 센서 데이터 읽기
 * @param ch 센서 채널 (0~3)
 * @param pwr 전력 출력 포인터
 * @param biotorq 생체토크 출력 포인터
 * @param cap 정전용량 출력 포인터
 * @return 0=성공, -1=실패
 */
int32_t Sensor_GetCANChannel(uint8_t ch, uint16_t *pwr, uint16_t *biotorq, uint16_t *cap);

/**
 * @brief 특정 채널의 온도 읽기
 * @param ch 온도 센서 채널 (0~5)
 * @param temp_c 온도 출력 포인터 (°C)
 * @return 0=성공, -1=실패 (센서 오류 포함)
 */
int32_t Sensor_GetTemperature(uint8_t ch, float *temp_c);

/**
 * @brief CAN 데이터 유효성 검사
 * @return 1=모든 데이터 유효, 0=일부 누락
 */
uint8_t Sensor_IsCANDataValid(void);

/**
 * @brief 온도 센서 오류 검사
 * @param ch 온도 센서 채널 (0~5)
 * @return 1=오류, 0=정상
 */
uint8_t Sensor_IsTempSensorFault(uint8_t ch);

/**
 * @brief 센서 데이터 타임아웃 검사
 * @param timeout_ms 타임아웃 기준 시간 (ms)
 * @return 1=타임아웃, 0=정상
 */
uint8_t Sensor_CheckTimeout(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H */
