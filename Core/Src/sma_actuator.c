/*
 * sma_actuator.c - Shape Memory Alloy Actuator Control Library
 *
 * 형상기억합금(SMA) 소프트 액추에이터 제어 라이브러리 구현
 */

#include "sma_actuator.h"
#include "sensors.h"
#include "emc2303.h"
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
static void SMA_UpdateForceControl(uint8_t ch, float dt);
static void SMA_UpdateFanControl(uint8_t ch);
static float SMA_CalculateFanDuty(float temp);
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

int32_t SMA_SetTargetForce(uint8_t ch, float force_n)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    // 힘 범위 체크 (추후 실제 센서 범위에 맞게 수정)
    if (force_n < 0.0f) {
        REPORT_ERROR_MSG("Target force must be positive");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];
    channel->target_force = force_n;

    // 힘 제어 모드로 자동 전환
    if (channel->mode != SMA_MODE_FORCE_CONTROL) {
        SMA_SetMode(ch, SMA_MODE_FORCE_CONTROL);
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
                SMA_UpdateForceControl(ch, dt);
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

        // 팬 냉각 제어 (자동 모드일 경우)
        SMA_UpdateFanControl(ch);

        channel->last_update_ms = HAL_GetTick();
    }
}

void SMA_EmergencyStop(void)
{
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_SetMode(ch, SMA_MODE_DISABLED);
        SMA_SetHardwarePWM(ch, 0.0f);
        sma_ctrl.channels[ch].pwm_duty = 0.0f;

        // 팬도 정지
        sma_ctrl.channels[ch].fan_duty = 0.0f;
        Fan6_SetDuty(ch, 0.0f);
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

        // 팬 제어 초기화
        channel->fan_duty = 0.0f;
        channel->fan_auto_enable = FAN_ENABLE_AUTO;  // 기본값: 자동 팬 제어

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

static void SMA_UpdateForceControl(uint8_t ch, float dt)
{
    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 과열 시 제어 중지
    if (channel->overtemp_flag) {
        SMA_SetHardwarePWM(ch, 0.0f);
        channel->pwm_duty = 0.0f;
        return;
    }

    // TODO: 힘 센서 데이터 읽기 (센서 준비 후 구현)
    // 현재는 골자만 준비
    const SensorData_t *sensor = Sensor_GetData();
    if (sensor == NULL) {
        return;
    }

    // 힘 센서 값 읽기 (biotorq 필드 사용)
    // 채널 매핑은 추후 실제 센서 연결에 맞게 조정 필요
    float current_force = 0.0f;
    if (ch < SENSOR_MAX_CH) {
        // biotorq raw 값을 힘(N)으로 변환
        // TODO: 실제 센서 캘리브레이션 값 적용
        current_force = (float)sensor->can.biotorq[ch] / 100.0f;  // 임시 스케일링
    }

    // PID 계산
    float pwm_output = SMA_PID_Compute(&channel->pid,
                                        channel->target_force,
                                        current_force,
                                        dt);

    channel->pwm_duty = pwm_output;
    SMA_SetHardwarePWM(ch, pwm_output);

    // 디버그 메시지 (선택적)
    // REPORT_INFO_MSG("Force Control: CH%d Target=%.1f Current=%.1f PWM=%.1f",
    //                 ch, channel->target_force, current_force, pwm_output);
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

/* ========== Fan Control Functions ========== */

int32_t SMA_SetFanAutoControl(uint8_t ch, uint8_t enable)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    sma_ctrl.channels[ch].fan_auto_enable = enable ? 1 : 0;

    // 자동 모드 비활성화 시 팬 정지
    if (!enable) {
        sma_ctrl.channels[ch].fan_duty = 0.0f;
        Fan6_SetDuty(ch, 0.0f);
    }

    return 0;
}

int32_t SMA_SetFanDuty(uint8_t ch, float duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 자동 제어 활성화 시 수동 제어 불가
    if (channel->fan_auto_enable) {
        REPORT_ERROR_MSG("Auto fan control is enabled");
        return -1;
    }

    // Duty 제한
    duty_pct = SMA_Clamp(duty_pct, 0.0f, 100.0f);

    channel->fan_duty = duty_pct;
    Fan6_SetDuty(ch, duty_pct);

    return 0;
}

int32_t SMA_GetFanDuty(uint8_t ch, float *duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS || duty_pct == NULL) {
        return -1;
    }

    *duty_pct = sma_ctrl.channels[ch].fan_duty;
    return 0;
}

static void SMA_UpdateFanControl(uint8_t ch)
{
    if (ch >= SMA_MAX_CHANNELS) {
        return;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    // 자동 팬 제어가 비활성화되어 있으면 건너뛰기
    if (!channel->fan_auto_enable) {
        return;
    }

    // 모드별 팬 제어
    if (channel->mode == SMA_MODE_DISABLED) {
        // 구동기가 꺼져 있으면 팬도 정지
        channel->fan_duty = 0.0f;
        Fan6_SetDuty(ch, 0.0f);
    } else {
        // 온도 기반 팬 제어
        float fan_duty = SMA_CalculateFanDuty(channel->current_temp);
        channel->fan_duty = fan_duty;
        Fan6_SetDuty(ch, fan_duty);
    }
}

static float SMA_CalculateFanDuty(float temp)
{
    // 온도가 FAN_TEMP_MIN 이하면 팬 정지
    if (temp < FAN_TEMP_MIN) {
        return 0.0f;
    }

    // 온도가 FAN_TEMP_MAX 이상이면 팬 최대 속도
    if (temp >= FAN_TEMP_MAX) {
        return FAN_DUTY_MAX;
    }

    // 선형 보간: FAN_TEMP_MIN ~ FAN_TEMP_MAX 사이에서 FAN_DUTY_MIN ~ FAN_DUTY_MAX
    float temp_range = FAN_TEMP_MAX - FAN_TEMP_MIN;
    float duty_range = FAN_DUTY_MAX - FAN_DUTY_MIN;
    float normalized = (temp - FAN_TEMP_MIN) / temp_range;

    float duty = FAN_DUTY_MIN + (normalized * duty_range);

    return SMA_Clamp(duty, FAN_DUTY_MIN, FAN_DUTY_MAX);
}
