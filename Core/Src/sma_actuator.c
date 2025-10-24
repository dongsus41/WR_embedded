/*
 * sma_actuator.c - Shape Memory Alloy Actuator Control Library
 *
 * 형상기억합금(SMA) 소프트 액추에이터 제어 라이브러리 구현
 */

#include "sma_actuator.h"
#include "sensors.h"
#include "tim.h"
#include "debug.h"
#include <string.h>
#include <math.h>

/* ========== Private Variables ========== */
static SMA_Controller_t sma_ctrl;

/* ========== Timer/Channel Mapping ========== */
typedef struct {
    TIM_HandleTypeDef *htim;    // 타이머 핸들
    uint32_t channel;           // 타이머 채널
} SMA_PWM_Map_t;

static const SMA_PWM_Map_t pwm_map[SMA_MAX_CHANNELS] = {
    {&htim2, TIM_CHANNEL_1},    // CH0: TIM2_CH1 (PA15)
    {&htim2, TIM_CHANNEL_2},    // CH1: TIM2_CH2 (PB3)
    {&htim3, TIM_CHANNEL_1},    // CH2: TIM3_CH1 (PB4)
    {&htim3, TIM_CHANNEL_2},    // CH3: TIM3_CH2 (PB5)
    {&htim4, TIM_CHANNEL_1},    // CH4: TIM4_CH1 (PB6)
    {&htim4, TIM_CHANNEL_2}     // CH5: TIM4_CH2 (PB7)
};

/* ========== Default PID Gains ========== */
#define DEFAULT_KP      5.0f    // 비례 게인
#define DEFAULT_KI      0.1f    // 적분 게인
#define DEFAULT_KD      0.5f    // 미분 게인

/* ========== Private Function Prototypes ========== */
static void SMA_InitPWM(void);
static void SMA_InitChannels(void);
static void SMA_UpdateTempControl(uint8_t ch, float dt);
static void SMA_SafetyCheck(uint8_t ch);
static inline float SMA_Clamp(float value, float min, float max);

/* ========== Public Functions ========== */

void SMA_Init(uint32_t control_period_ms)
{
    // 구조체 초기화
    memset(&sma_ctrl, 0, sizeof(SMA_Controller_t));

    // 제어 주기 설정
    sma_ctrl.control_period_ms = control_period_ms;

    // 채널 초기화
    SMA_InitChannels();

    // PWM 초기화 및 시작
    SMA_InitPWM();

    sma_ctrl.initialized = 1;

    REPORT_INFO_MSG("SMA Actuator Controller Initialized");
}

int32_t SMA_SetMode(uint8_t ch, SMA_ControlMode_t mode)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 모드 변경 시 PID 리셋
    if (channel->mode != mode) {
        SMA_PID_Reset(&channel->pid);

        // DISABLED 모드로 변경 시 PWM 0%
        if (mode == SMA_MODE_DISABLED) {
            SMA_SetHardwarePWM(ch, 0.0f);
            channel->pwm_duty = 0.0f;
        }
    }

    channel->mode = mode;
    return 0;
}

int32_t SMA_SetPWM(uint8_t ch, float duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 오픈루프 모드에서만 허용
    if (channel->mode != SMA_MODE_OPEN_LOOP) {
        REPORT_ERROR_MSG("Not in OPEN_LOOP mode");
        return -1;
    }

    // 과열 체크
    if (channel->overtemp_flag) {
        REPORT_ERROR_MSG("Overtemp - PWM disabled");
        return -1;
    }

    // PWM 제한
    duty_pct = SMA_Clamp(duty_pct, SMA_PWM_MIN, SMA_PWM_MAX);

    channel->pwm_duty = duty_pct;
    SMA_SetHardwarePWM(ch, duty_pct);

    return 0;
}

int32_t SMA_SetTargetTemp(uint8_t ch, float temp_c)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    // 온도 범위 체크
    if (temp_c < SMA_TEMP_MIN || temp_c > SMA_TEMP_MAX_SAFE) {
        REPORT_ERROR_MSG("Target temp out of range");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];
    channel->target_temp = temp_c;

    // 온도 제어 모드로 자동 전환
    if (channel->mode != SMA_MODE_TEMP_CONTROL) {
        SMA_SetMode(ch, SMA_MODE_TEMP_CONTROL);
    }

    return 0;
}

int32_t SMA_SetPIDGains(uint8_t ch, float kp, float ki, float kd)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];
    channel->pid.kp = kp;
    channel->pid.ki = ki;
    channel->pid.kd = kd;

    // 적분 리셋
    channel->pid.integral = 0.0f;

    return 0;
}

int32_t SMA_GetTemp(uint8_t ch, float *temp_c)
{
    if (ch >= SMA_MAX_CHANNELS || temp_c == NULL) {
        return -1;
    }

    *temp_c = sma_ctrl.channels[ch].current_temp;
    return 0;
}

int32_t SMA_GetPWM(uint8_t ch, float *duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS || duty_pct == NULL) {
        return -1;
    }

    *duty_pct = sma_ctrl.channels[ch].pwm_duty;
    return 0;
}

void SMA_Update(void)
{
    if (!sma_ctrl.initialized) {
        return;
    }

    float dt = (float)sma_ctrl.control_period_ms / 1000.0f;  // 초 단위로 변환

    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_Channel_t *channel = &sma_ctrl.channels[ch];

        // 온도 센서 읽기
        float temp_c;
        if (Sensor_GetTemperature(ch, &temp_c) == 0) {
            channel->current_temp = temp_c;
        }

        // 안전 체크
        SMA_SafetyCheck(ch);

        // 모드별 제어
        switch (channel->mode) {
            case SMA_MODE_TEMP_CONTROL:
                SMA_UpdateTempControl(ch, dt);
                break;

            case SMA_MODE_FORCE_CONTROL:
                // 추후 구현
                break;

            case SMA_MODE_POSITION_CONTROL:
                // 추후 구현
                break;

            case SMA_MODE_OPEN_LOOP:
                // 오픈루프는 수동 제어, 업데이트 불필요
                break;

            case SMA_MODE_DISABLED:
            default:
                // PWM 0% 유지
                SMA_SetHardwarePWM(ch, 0.0f);
                channel->pwm_duty = 0.0f;
                break;
        }

        channel->last_update_ms = HAL_GetTick();
    }
}

void SMA_EmergencyStop(void)
{
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_SetMode(ch, SMA_MODE_DISABLED);
        SMA_SetHardwarePWM(ch, 0.0f);
        sma_ctrl.channels[ch].pwm_duty = 0.0f;
    }

    REPORT_ERROR_MSG("SMA Emergency Stop");
}

const SMA_Channel_t* SMA_GetChannelState(uint8_t ch)
{
    if (ch >= SMA_MAX_CHANNELS) {
        return NULL;
    }
    return &sma_ctrl.channels[ch];
}

uint8_t SMA_IsOverTemp(uint8_t ch)
{
    if (ch >= SMA_MAX_CHANNELS) {
        return 1;
    }
    return sma_ctrl.channels[ch].overtemp_flag;
}

/* ========== Private Functions ========== */

static void SMA_InitPWM(void)
{
    // 모든 타이머 PWM 시작
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        TIM_HandleTypeDef *htim = pwm_map[ch].htim;
        uint32_t channel = pwm_map[ch].channel;

        // PWM 시작
        if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) {
            REPORT_ERROR_MSG("PWM start failed");
        }

        // 초기 PWM 0%
        SMA_SetHardwarePWM(ch, 0.0f);
    }
}

static void SMA_InitChannels(void)
{
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_Channel_t *channel = &sma_ctrl.channels[ch];

        // 기본값 설정
        channel->mode = SMA_MODE_DISABLED;
        channel->target_temp = 0.0f;
        channel->current_temp = 0.0f;
        channel->pwm_duty = 0.0f;
        channel->overtemp_flag = 0;
        channel->last_update_ms = 0;

        // PID 초기화
        channel->pid.kp = DEFAULT_KP;
        channel->pid.ki = DEFAULT_KI;
        channel->pid.kd = DEFAULT_KD;
        channel->pid.integral = 0.0f;
        channel->pid.prev_error = 0.0f;
        channel->pid.output_min = SMA_PWM_MIN;
        channel->pid.output_max = SMA_PWM_MAX;
    }
}

static void SMA_UpdateTempControl(uint8_t ch, float dt)
{
    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 과열 시 제어 중지
    if (channel->overtemp_flag) {
        SMA_SetHardwarePWM(ch, 0.0f);
        channel->pwm_duty = 0.0f;
        return;
    }

    // PID 계산
    float pwm_output = SMA_PID_Compute(&channel->pid,
                                        channel->target_temp,
                                        channel->current_temp,
                                        dt);

    channel->pwm_duty = pwm_output;
    SMA_SetHardwarePWM(ch, pwm_output);
}

static void SMA_SafetyCheck(uint8_t ch)
{
    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 위험 온도 체크
    if (channel->current_temp >= SMA_TEMP_MAX_CRIT) {
        if (!channel->overtemp_flag) {
            channel->overtemp_flag = 1;
            SMA_SetHardwarePWM(ch, 0.0f);
            channel->pwm_duty = 0.0f;

            REPORT_ERROR_MSG("Critical overtemp - shutdown");
        }
    }
    // 안전 온도로 복귀
    else if (channel->current_temp < (SMA_TEMP_MAX_SAFE - 5.0f)) {
        if (channel->overtemp_flag) {
            channel->overtemp_flag = 0;
            REPORT_INFO_MSG("Temp normal - resume");
        }
    }
}

static inline float SMA_Clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void SMA_SetHardwarePWM(uint8_t ch, float duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS) {
        return;
    }

    // Duty cycle 제한
    duty_pct = SMA_Clamp(duty_pct, 0.0f, 100.0f);

    // 타이머 ARR 값 가져오기
    TIM_HandleTypeDef *htim = pwm_map[ch].htim;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    // CCR 값 계산 (0~ARR)
    uint32_t ccr = (uint32_t)((duty_pct * (float)arr) / 100.0f);

    // CCR 설정
    __HAL_TIM_SET_COMPARE(htim, pwm_map[ch].channel, ccr);
}

float SMA_PID_Compute(SMA_PID_t *pid, float setpoint, float measurement, float dt)
{
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    // 오차 계산
    float error = setpoint - measurement;

    // 비례항
    float p_term = pid->kp * error;

    // 적분항 (Anti-windup 포함)
    pid->integral += error * dt;

    // 적분 windup 방지
    float max_integral = 50.0f;  // 적분 누적 한계
    pid->integral = SMA_Clamp(pid->integral, -max_integral, max_integral);

    float i_term = pid->ki * pid->integral;

    // 미분항
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    // PID 출력
    float output = p_term + i_term + d_term;

    // 출력 제한
    output = SMA_Clamp(output, pid->output_min, pid->output_max);

    // 이전 오차 저장
    pid->prev_error = error;

    return output;
}

void SMA_PID_Reset(SMA_PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
