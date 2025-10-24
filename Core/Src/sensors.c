#include <string.h>
#include <stdio.h>
#include "sensors.h"
#include "debug.h"
#include "main.h"

/* ========== Private Variables ========== */
static SensorData_t sensor_data;
extern volatile uint16_t buf_adc1[CTRL_CH];


/* ========== Private Function Prototypes ========== */
static inline uint16_t u16le(const uint8_t *p);
static float temp_raw_to_celsius(uint16_t adc_val);
static void check_temp_sensor_fault(uint8_t ch, float temp);

/* ========== Public Functions ========== */

void Sensor_Init(void)
{
    memset(&sensor_data, 0, sizeof(SensorData_t));

    // 센서 오류 플래그 초기화 (모두 정상)
    for (uint8_t i = 0; i < SENSOR_TEMP_CH; i++) {
        sensor_data.adc.sensor_fault[i] = 0;
    }

    // 타임스탬프 초기화
    sensor_data.can.timestamp = HAL_GetTick();
    sensor_data.adc.timestamp = HAL_GetTick();
}

int32_t Sensor_UpdateCAN(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) {
        REPORT_ERROR_MSG("NULL pointer");
        return -1;
    }

    // 데이터 길이 검증 (8바이트 고정)
    if (rx_header->DataLength != FDCAN_DLC_BYTES_8) {
        REPORT_ERROR_MSG("Invalid CAN data length");
        return -1;
    }

    uint32_t can_id = rx_header->Identifier;

    // Critical Section 진입
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    switch (can_id) {
        case CAN_ID_PWR_FOREARM:
            sensor_data.can.biotorq[SENSOR_CH_FOREARM_L] = u16le(&rx_data[0]);
            sensor_data.can.pwr[SENSOR_CH_FOREARM_L]     = u16le(&rx_data[2]);
            sensor_data.can.biotorq[SENSOR_CH_FOREARM_R] = u16le(&rx_data[4]);
            sensor_data.can.pwr[SENSOR_CH_FOREARM_R]     = u16le(&rx_data[6]);
            sensor_data.can.valid_flag |= (1U << 0);
            break;

        case CAN_ID_PWR_BICEPS:
            sensor_data.can.biotorq[SENSOR_CH_BICEPS_L] = u16le(&rx_data[0]);
            sensor_data.can.pwr[SENSOR_CH_BICEPS_L]     = u16le(&rx_data[2]);
            sensor_data.can.biotorq[SENSOR_CH_BICEPS_R] = u16le(&rx_data[4]);
            sensor_data.can.pwr[SENSOR_CH_BICEPS_R]     = u16le(&rx_data[6]);
            sensor_data.can.valid_flag |= (1U << 1);
            break;

        case CAN_ID_PWR_CAP:
            sensor_data.can.cap[SENSOR_CH_FOREARM_L] = u16le(&rx_data[0]);
            sensor_data.can.cap[SENSOR_CH_FOREARM_R] = u16le(&rx_data[2]);
            sensor_data.can.cap[SENSOR_CH_BICEPS_L]  = u16le(&rx_data[4]);
            sensor_data.can.cap[SENSOR_CH_BICEPS_R]  = u16le(&rx_data[6]);
            sensor_data.can.valid_flag |= (1U << 2);
            break;

        default:
            __set_PRIMASK(primask);
            return -1;  // 알 수 없는 CAN ID
    }

    // 타임스탬프 업데이트
    sensor_data.can.timestamp = HAL_GetTick();

    // Critical Section 종료
    __set_PRIMASK(primask);

    return 0;
}

void Sensor_UpdateADC(volatile uint16_t *adc_buffer)
{
    if (adc_buffer == NULL) {
        REPORT_ERROR_MSG("NULL ADC buffer");
        return;
    }
    sensor_data.adc.raw[0] = buf_adc1[0];

    // Critical Section 진입
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t i = 0; i < SENSOR_TEMP_CH; i++) {
        // Raw 값 저장
        sensor_data.adc.raw[i] = adc_buffer[i];

        // 온도 변환
        sensor_data.adc.temp_c[i] = temp_raw_to_celsius(adc_buffer[i]);

        // 센서 오류 검사
        check_temp_sensor_fault(i, sensor_data.adc.temp_c[i]);
    }

    // 타임스탬프 업데이트
    sensor_data.adc.timestamp = HAL_GetTick();

    // Critical Section 종료
    __set_PRIMASK(primask);
}

const SensorData_t* Sensor_GetData(void)
{
    return &sensor_data;
}

int32_t Sensor_GetCANChannel(uint8_t ch, uint16_t *pwr, uint16_t *biotorq, uint16_t *cap)
{
    if (ch >= SENSOR_MAX_CH) {
        return -1;
    }

    if (pwr == NULL || biotorq == NULL || cap == NULL) {
        return -1;
    }

    // Critical Section
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *pwr     = sensor_data.can.pwr[ch];
    *biotorq = sensor_data.can.biotorq[ch];
    *cap     = sensor_data.can.cap[ch];

    __set_PRIMASK(primask);

    return 0;
}

int32_t Sensor_GetTemperature(uint8_t ch, float *temp_c)
{
    if (ch >= SENSOR_TEMP_CH || temp_c == NULL) {
        return -1;
    }

    // 센서 오류 체크
    if (sensor_data.adc.sensor_fault[ch]) {
        return -1;
    }

    // Critical Section
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *temp_c = sensor_data.adc.temp_c[ch];

    __set_PRIMASK(primask);

    return 0;
}

uint8_t Sensor_IsCANDataValid(void)
{
    // 모든 비트가 세트되어야 함 (0x07 = 0b111)
    return (sensor_data.can.valid_flag == 0x07U) ? 1U : 0U;
}

uint8_t Sensor_IsTempSensorFault(uint8_t ch)
{
    if (ch >= SENSOR_TEMP_CH) {
        return 1;
    }
    return sensor_data.adc.sensor_fault[ch];
}

uint8_t Sensor_CheckTimeout(uint32_t timeout_ms)
{
    uint32_t current_tick = HAL_GetTick();

    // CAN 데이터 타임아웃
    if ((current_tick - sensor_data.can.timestamp) > timeout_ms) {
        sensor_data.can.valid_flag = 0;
        return 1;
    }

    // ADC 데이터 타임아웃
    if ((current_tick - sensor_data.adc.timestamp) > timeout_ms) {
        return 1;
    }

    return 0;
}

/* ========== Private Functions ========== */

/**
 * @brief Little-Endian 16비트 변환
 */
static inline uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[1] | ((uint16_t)p[0] << 8);
}

/**
 * @brief ADC Raw → 온도 변환
 * @details TMP235: V_out = 1.25V + (T_C × 5mV/°C), ADC 16-bit, Vref=3.3V
 */
static float temp_raw_to_celsius(uint16_t adc_val)
{
    // ADC → 전압 (0~65535 → 0~3.3V)
    float voltage = ((float)adc_val / 65535.0f) * 3.3f;

    // 전압 → 온도
    float temp_c = (voltage - 1.25f) / 0.005f;

    return temp_c;
}

/**
 * @brief 온도 센서 오류 검사
 * @details 범위: -40°C ~ 125°C (TMP235 규격)
 */
static void check_temp_sensor_fault(uint8_t ch, float temp)
{
    if (ch >= SENSOR_TEMP_CH) {
        return;
    }

    if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID) {
        // 오류 발생
        if (!sensor_data.adc.sensor_fault[ch]) {
            sensor_data.adc.sensor_fault[ch] = 1;

            // 에러 로그 (최초 1회만)
            char msg[MAX_LOG_MSG_LEN];
            snprintf(msg, sizeof(msg),
                     "TEMP_FAULT: CH%u = %.1f°C (Range: %.0f~%.0f)\r\n",
                     ch, temp, TEMP_MIN_VALID, TEMP_MAX_VALID);
            LogFifo_Push(msg);
        }
    } else {
        // 정상 복귀
        if (sensor_data.adc.sensor_fault[ch]) {
            sensor_data.adc.sensor_fault[ch] = 0;

            char msg[MAX_LOG_MSG_LEN];
            snprintf(msg, sizeof(msg), "TEMP_OK: CH%u = %.1f°C\r\n", ch, temp);
            LogFifo_Push(msg);
        }
    }
}
