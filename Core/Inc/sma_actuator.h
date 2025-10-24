/*
 * sma_actuator.h - Shape Memory Alloy Actuator Control Library
 *
 * 형상기억합금(SMA) 소프트 액추에이터 제어 라이브러리
 * - 6채널 PWM 기반 전열 구동 제어
 * - 온도 피드백 제어
 * - 다중 제어 모드 지원 (오픈루프, 온도제어, 힘제어, 위치제어)
 *
 * Hardware:
 *   - PWM Output: TIM2_CH1/2, TIM3_CH1/2, TIM4_CH1/2 (6 channels)
 *   - Temp Sensor: ADC1 (6 channels) via sensors.c
 *   - Power Gating: 6-channel power switches
 */

#ifndef SMA_ACTUATOR_H
#define SMA_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/* ========== Configuration ========== */
#define SMA_MAX_CHANNELS        6       // 최대 액추에이터 채널 수

/* ========== Safety Limits ========== */
#define SMA_TEMP_MAX_SAFE       100.0f  // 안전 최대 온도 (°C)
#define SMA_TEMP_MAX_CRIT       120.0f  // 위험 온도 - 즉시 차단 (°C)
#define SMA_TEMP_MIN            0.0f    // 최소 목표 온도 (°C)
#define SMA_PWM_MAX             100.0f  // 최대 PWM Duty (%)
#define SMA_PWM_MIN             0.0f    // 최소 PWM Duty (%)

/* ========== Fan Control Settings ========== */
#define FAN_TEMP_START          40.0f   // 팬 시작 온도 (°C)
#define FAN_TEMP_MIN            30.0f   // 팬 최소 동작 온도 (°C)
#define FAN_TEMP_MAX            90.0f   // 팬 최대 속도 온도 (°C)
#define FAN_DUTY_MIN            30.0f   // 팬 최소 Duty (%)
#define FAN_DUTY_MAX            100.0f  // 팬 최대 Duty (%)
#define FAN_ENABLE_AUTO         1       // 자동 팬 제어 활성화 (0=비활성, 1=활성)

/* ========== Control Mode ========== */
typedef enum {
    SMA_MODE_DISABLED = 0,      // 비활성 (PWM 0%)
    SMA_MODE_OPEN_LOOP,         // 오픈루프 (PWM 직접 제어)
    SMA_MODE_TEMP_CONTROL,      // 온도 제어 (PID)
    SMA_MODE_FORCE_CONTROL,     // 힘 제어 (추후 구현)
    SMA_MODE_POSITION_CONTROL   // 위치 제어 (추후 구현)
} SMA_ControlMode_t;

/* ========== PID Controller Parameters ========== */
typedef struct {
    float kp;           // 비례 게인
    float ki;           // 적분 게인
    float kd;           // 미분 게인
    float integral;     // 적분 누적값
    float prev_error;   // 이전 오차
    float output_min;   // 출력 최소값 (%)
    float output_max;   // 출력 최대값 (%)
} SMA_PID_t;

/* ========== Channel State ========== */
typedef struct {
    SMA_ControlMode_t mode;     // 제어 모드
    float target_temp;          // 목표 온도 (°C)
    float current_temp;         // 현재 온도 (°C)
    float pwm_duty;             // 현재 PWM Duty (%)
    float target_force;         // 목표 힘 (추후 사용)
    float target_position;      // 목표 위치 (추후 사용)
    SMA_PID_t pid;              // PID 제어기
    uint8_t overtemp_flag;      // 과열 플래그 (0=정상, 1=과열)
    uint32_t last_update_ms;    // 마지막 업데이트 시간 (ms)
    float fan_duty;             // 팬 PWM Duty (%)
    uint8_t fan_auto_enable;    // 자동 팬 제어 활성화 (0=수동, 1=자동)
} SMA_Channel_t;

/* ========== Global Controller State ========== */
typedef struct {
    SMA_Channel_t channels[SMA_MAX_CHANNELS];   // 채널 상태
    uint32_t control_period_ms;                 // 제어 주기 (ms)
    uint8_t initialized;                        // 초기화 플래그
} SMA_Controller_t;

/* ========== Public API ========== */

/**
 * @brief SMA 액추에이터 컨트롤러 초기화
 * @param control_period_ms 제어 주기 (ms) - 권장: 10~100ms
 * @note 타이머 PWM 시작, PID 초기화, 안전 상태로 설정
 */
void SMA_Init(uint32_t control_period_ms);

/**
 * @brief 특정 채널의 제어 모드 설정
 * @param ch 채널 번호 (0~5)
 * @param mode 제어 모드
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetMode(uint8_t ch, SMA_ControlMode_t mode);

/**
 * @brief 오픈루프 PWM 설정 (SMA_MODE_OPEN_LOOP에서만 동작)
 * @param ch 채널 번호 (0~5)
 * @param duty_pct PWM Duty (0.0~100.0%)
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetPWM(uint8_t ch, float duty_pct);

/**
 * @brief 목표 온도 설정 (SMA_MODE_TEMP_CONTROL에서만 동작)
 * @param ch 채널 번호 (0~5)
 * @param temp_c 목표 온도 (°C)
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetTargetTemp(uint8_t ch, float temp_c);

/**
 * @brief 목표 힘 설정 (SMA_MODE_FORCE_CONTROL에서만 동작)
 * @param ch 채널 번호 (0~5)
 * @param force_n 목표 힘 (N 또는 센서 raw 값)
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetTargetForce(uint8_t ch, float force_n);

/**
 * @brief PID 게인 설정
 * @param ch 채널 번호 (0~5)
 * @param kp 비례 게인
 * @param ki 적분 게인
 * @param kd 미분 게인
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetPIDGains(uint8_t ch, float kp, float ki, float kd);

/**
 * @brief 현재 온도 읽기
 * @param ch 채널 번호 (0~5)
 * @param temp_c 온도 출력 포인터 (°C)
 * @return 0=성공, -1=실패
 */
int32_t SMA_GetTemp(uint8_t ch, float *temp_c);

/**
 * @brief 현재 PWM Duty 읽기
 * @param ch 채널 번호 (0~5)
 * @param duty_pct PWM Duty 출력 포인터 (%)
 * @return 0=성공, -1=실패
 */
int32_t SMA_GetPWM(uint8_t ch, float *duty_pct);

/**
 * @brief 제어기 업데이트 (주기적 호출 필요)
 * @note 타이머 인터럽트 또는 메인 루프에서 호출
 *       - 온도 센서 읽기
 *       - PID 계산
 *       - PWM 출력 업데이트
 *       - 과열 보호
 */
void SMA_Update(void);

/**
 * @brief 모든 채널 비상 정지
 * @note 모든 PWM을 0%로 설정하고 모드를 DISABLED로 변경
 */
void SMA_EmergencyStop(void);

/**
 * @brief 채널 상태 읽기 (디버그용)
 * @param ch 채널 번호 (0~5)
 * @return 채널 상태 포인터 (읽기 전용)
 */
const SMA_Channel_t* SMA_GetChannelState(uint8_t ch);

/**
 * @brief 과열 상태 확인
 * @param ch 채널 번호 (0~5)
 * @return 1=과열, 0=정상
 */
uint8_t SMA_IsOverTemp(uint8_t ch);

/**
 * @brief 팬 자동 제어 활성화/비활성화
 * @param ch 채널 번호 (0~5)
 * @param enable 1=자동 팬 제어, 0=수동 제어
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetFanAutoControl(uint8_t ch, uint8_t enable);

/**
 * @brief 팬 수동 제어 (자동 제어 비활성화 시에만 동작)
 * @param ch 채널 번호 (0~5)
 * @param duty_pct 팬 PWM Duty (0.0~100.0%)
 * @return 0=성공, -1=실패
 */
int32_t SMA_SetFanDuty(uint8_t ch, float duty_pct);

/**
 * @brief 현재 팬 Duty 읽기
 * @param ch 채널 번호 (0~5)
 * @param duty_pct 팬 Duty 출력 포인터 (%)
 * @return 0=성공, -1=실패
 */
int32_t SMA_GetFanDuty(uint8_t ch, float *duty_pct);

/* ========== Private Functions (내부 사용) ========== */

/**
 * @brief 하드웨어 PWM 출력 설정
 * @param ch 채널 번호 (0~5)
 * @param duty_pct PWM Duty (0.0~100.0%)
 */
void SMA_SetHardwarePWM(uint8_t ch, float duty_pct);

/**
 * @brief PID 제어기 계산
 * @param pid PID 구조체 포인터
 * @param setpoint 목표값
 * @param measurement 측정값
 * @param dt 시간 간격 (s)
 * @return PID 출력값
 */
float SMA_PID_Compute(SMA_PID_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief PID 제어기 리셋
 * @param pid PID 구조체 포인터
 */
void SMA_PID_Reset(SMA_PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* SMA_ACTUATOR_H */
