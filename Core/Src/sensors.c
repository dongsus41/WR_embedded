#include <string.h>
#include <stdio.h>
#include "sensors.h"
#include "debug.h"
#include "main.h"

#include "cmsis_os.h"

extern osMutexId_t sensorDataMutexHandle;

/* ========== Private Variables ========== */
static SensorData_t sensor_data;
extern volatile uint16_t buf_adc1[CTRL_CH];

// 온도 필터 버퍼 (이동평균용)
static float temp_filter_buffer[SENSOR_TEMP_CH][TEMP_FILTER_SIZE];
static uint8_t temp_filter_index[SENSOR_TEMP_CH];
static uint8_t temp_filter_filled[SENSOR_TEMP_CH];

// 이전 온도 값 (이상치 필터용)
static float temp_last_valid[SENSOR_TEMP_CH];

/* ========== Private Function Prototypes ========== */
static inline uint16_t u16le(const uint8_t *p);
static float temp_raw_to_celsius(uint16_t adc_val);
static void check_temp_sensor_fault(uint8_t ch, float temp);
static float apply_outlier_filter(uint8_t ch, float new_temp);
static float apply_moving_average_filter(uint8_t ch, float new_temp);

/* ========== Public Functions ========== */

void Sensor_Init(void)
{
    memset(&sensor_data, 0, sizeof(SensorData_t));

    // 센서 오류 플래그 초기화 (모두 정상)
    for (uint8_t i = 0; i < SENSOR_TEMP_CH; i++) {
        sensor_data.adc.sensor_fault[i] = 0;
    }

    // 온도 필터 버퍼 초기화
    memset(temp_filter_buffer, 0, sizeof(temp_filter_buffer));
    memset(temp_filter_index, 0, sizeof(temp_filter_index));
    memset(temp_filter_filled, 0, sizeof(temp_filter_filled));
    memset(temp_last_valid, 0, sizeof(temp_last_valid));

    // 타임스탬프 초기화
    sensor_data.can.timestamp = HAL_GetTick();
    sensor_data.adc.timestamp = HAL_GetTick();
    sensor_data.adc.update_count = 0;
}

/**
 * @brief CAN 센서 데이터 업데이트
 * @note ISR(FDCAN RX)에서 호출됨 - mutex 사용 불가
 * @note [수정됨] 인터럽트 복원 정상화
 */
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

    // 0x100~0x10F 변위센서 처리 (CAN1)
    if (can_id >= 0x100 && can_id <= 0x10F) {
        uint8_t idx = can_id - 0x100;  // 0~15 인덱스
        sensor_data.can.displacement[idx] = u16le(&rx_data[0]);
        sensor_data.can.displacement_valid_flag |= (1U << idx);
        sensor_data.can.timestamp = HAL_GetTick();
        __set_PRIMASK(primask);
        return 0;
    }

    // 힘센서 처리 (CAN2)
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
            // [수정] 인터럽트 복원 후 리턴
            __set_PRIMASK(primask);
            return -1;  // 알 수 없는 CAN ID
    }

    // 타임스탬프 업데이트
    sensor_data.can.timestamp = HAL_GetTick();

    // [수정] Critical Section 종료 - 인터럽트 복원
    __set_PRIMASK(primask);

    return 0;
}

/**
 * @brief ADC 온도 데이터 업데이트
 * @note ISR(DMA Complete)에서 호출됨 - mutex 사용 불가
 * @note [수정됨] 인터럽트 복원 정상화
 */
void Sensor_UpdateADC(volatile uint16_t *adc_buffer)
{
    if (adc_buffer == NULL) {
        REPORT_ERROR_MSG("NULL ADC buffer");
        return;
    }

    // Critical Section 진입
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t i = 0; i < SENSOR_TEMP_CH; i++) {
        // Raw 값 저장
        sensor_data.adc.raw[i] = adc_buffer[i];

        // 온도 변환 (필터 미적용)
        float temp_raw = temp_raw_to_celsius(adc_buffer[i]);
        sensor_data.adc.temp_raw_c[i] = temp_raw;

        // 이상치 필터 적용 (10°C 이상 급변 제거)
        float temp_outlier_filtered = apply_outlier_filter(i, temp_raw);

        // 이동평균 필터 적용
        float temp_filtered = apply_moving_average_filter(i, temp_outlier_filtered);
        sensor_data.adc.temp_c[i] = temp_filtered;

        // 센서 오류 검사 (필터링된 값으로)
        check_temp_sensor_fault(i, temp_filtered);
    }

    // 타임스탬프 및 카운터 업데이트
    sensor_data.adc.timestamp = HAL_GetTick();
    sensor_data.adc.update_count++;

    // [수정] Critical Section 종료 - 인터럽트 복원
    __set_PRIMASK(primask);
}

const SensorData_t* Sensor_GetData(void)
{
    return &sensor_data;
}

int32_t Sensor_GetCANChannel(uint8_t ch, uint16_t *pwr, uint16_t *biotorq, uint16_t *cap)
{
    if (ch >= SENSOR_FORCE_CH) {
        return -1;
    }

    if (pwr == NULL || biotorq == NULL || cap == NULL) {
        return -1;
    }

    if (osMutexAcquire(sensorDataMutexHandle, 10) == osOK)
    {
        *pwr     = sensor_data.can.pwr[ch];
        *biotorq = sensor_data.can.biotorq[ch];
        *cap     = sensor_data.can.cap[ch];

        osMutexRelease(sensorDataMutexHandle);
        return 0;
    }

    return -1;
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

    if (osMutexAcquire(sensorDataMutexHandle, 10) == osOK)
    {
        *temp_c = sensor_data.adc.temp_c[ch];

        osMutexRelease(sensorDataMutexHandle);
        return 0;
    }

    return -1;
}

/**
 * @brief 온도 읽기 (ISR-safe 버전, mutex 없음)
 * @note ISR 또는 빠른 읽기가 필요할 때 사용
 * @warning 데이터 일관성 보장 안됨
 */
int32_t Sensor_GetTemperature_ISR(uint8_t ch, float *temp_c)
{
    if (ch >= SENSOR_TEMP_CH || temp_c == NULL) {
        return -1;
    }

    if (sensor_data.adc.sensor_fault[ch]) {
        return -1;
    }

    // 직접 읽기 (float는 32비트 - 원자적 읽기)
    *temp_c = sensor_data.adc.temp_c[ch];
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
 * @details 범위: -40°C ~ 125°C
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

/**
 * @brief 이상치 필터 적용 (급격한 온도 변화 제거)
 * @param ch 온도 센서 채널 (0~5)
 * @param new_temp 새로운 온도 값 (°C)
 * @return 필터링된 온도 (°C) - 이상치이면 이전 값 유지
 * @details 이전 값 대비 10°C 이상 변화 시 노이즈로 판단하고 이전 값 반환
 */
static float apply_outlier_filter(uint8_t ch, float new_temp)
{
    if (ch >= SENSOR_TEMP_CH) {
        return new_temp;
    }

    // 첫 번째 측정값이거나 이전 값이 유효하지 않으면 그대로 사용
    if (temp_last_valid[ch] == 0.0f) {
        temp_last_valid[ch] = new_temp;
        return new_temp;
    }

    // 이전 값과의 차이 계산
    float delta = new_temp - temp_last_valid[ch];
    float abs_delta = (delta > 0) ? delta : -delta;  // 절댓값

    // 10°C 이상 급변하면 이상치로 판단
    if (abs_delta > TEMP_OUTLIER_THRESHOLD) {
        // 이상치 - 이전 값 유지
        char msg[MAX_LOG_MSG_LEN];
        snprintf(msg, sizeof(msg),
                 "TEMP_OUTLIER: CH%u = %.1f°C (prev=%.1f°C, delta=%.1f°C)\r\n",
                 ch, new_temp, temp_last_valid[ch], delta);
        LogFifo_Push(msg);

        return temp_last_valid[ch];  // 이전 유효 값 반환
    }

    // 정상 범위 내 변화 - 새 값 사용
    temp_last_valid[ch] = new_temp;
    return new_temp;
}

/**
 * @brief 이동평균 필터 적용
 * @param ch 온도 센서 채널 (0~5)
 * @param new_temp 새로운 온도 값 (°C)
 * @return 필터링된 온도 (°C)
 */
static float apply_moving_average_filter(uint8_t ch, float new_temp)
{
    if (ch >= SENSOR_TEMP_CH) {
        return new_temp;
    }

    // 새 값을 버퍼에 저장
    temp_filter_buffer[ch][temp_filter_index[ch]] = new_temp;

    // 인덱스 증가 (순환 버퍼)
    temp_filter_index[ch]++;
    if (temp_filter_index[ch] >= TEMP_FILTER_SIZE) {
        temp_filter_index[ch] = 0;
        temp_filter_filled[ch] = 1;  // 버퍼가 가득 찼음
    }

    // 평균 계산
    float sum = 0.0f;
    uint8_t count = temp_filter_filled[ch] ? TEMP_FILTER_SIZE : temp_filter_index[ch];

    for (uint8_t i = 0; i < count; i++) {
        sum += temp_filter_buffer[ch][i];
    }

    return (count > 0) ? (sum / (float)count) : new_temp;
}
