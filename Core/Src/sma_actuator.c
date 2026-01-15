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

#include "cmsis_os.h"

/* ========== Private Variables ========== */
static SMA_Controller_t sma_ctrl;

extern osMutexId_t smaChannelMutexHandle;
extern osMutexId_t i2cMutexHandle;

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
static void SMA_SafetyCheck(uint8_t ch);
static inline float SMA_Clamp(float value, float min, float max);
static float SMA_CalculateFanDuty(float current_temp, float target_temp);

/* ========== Public Functions ========== */

void SMA_Init(uint32_t control_period_ms)
{
    memset(&sma_ctrl, 0, sizeof(SMA_Controller_t));
    sma_ctrl.control_period_ms = control_period_ms;

    SMA_InitChannels();
    SMA_InitPWM();

    sma_ctrl.initialized = 1;

    REPORT_INFO_MSG("SMA Actuator Controller Initialized (RTOS)");
}

/**
 * @brief 제어 모드 설정
 * @note 외부에서 smaChannelMutex 획득 후 호출 권장
 */
int32_t SMA_SetMode(uint8_t ch, SMA_ControlMode_t mode)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    if (channel->mode != mode) {
        SMA_PID_Reset(&channel->pid);

        if (mode == SMA_MODE_DISABLED) {
            SMA_SetHardwarePWM(ch, 0.0f);
            channel->pwm_duty = 0.0f;
        }
    }

    channel->mode = mode;
    return 0;
}

/**
 * @brief 오픈루프 PWM 설정
 * @note 외부에서 smaChannelMutex 획득 후 호출 권장
 */
int32_t SMA_SetPWM(uint8_t ch, float duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    if (channel->mode != SMA_MODE_OPEN_LOOP) {
        REPORT_ERROR_MSG("Not in OPEN_LOOP mode");
        return -1;
    }

    if (channel->overtemp_flag) {
        REPORT_ERROR_MSG("Overtemp - PWM disabled");
        return -1;
    }

    duty_pct = SMA_Clamp(duty_pct, SMA_PWM_MIN, SMA_PWM_MAX);

    channel->pwm_duty = duty_pct;
    SMA_SetHardwarePWM(ch, duty_pct);

    return 0;
}

/**
 * @brief 목표 온도 설정
 * @note 외부에서 smaChannelMutex 획득 후 호출 권장
 */
int32_t SMA_SetTargetTemp(uint8_t ch, float temp_c)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    if (temp_c < SMA_TEMP_MIN || temp_c > SMA_TEMP_MAX_SAFE) {
        REPORT_ERROR_MSG("Target temp out of range");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];
    channel->target_temp = temp_c;

    if (channel->mode != SMA_MODE_TEMP_CONTROL) {
        SMA_SetMode(ch, SMA_MODE_TEMP_CONTROL);
    }

    return 0;
}

/**
 * @brief 목표 힘 설정
 * @note 외부에서 smaChannelMutex 획득 후 호출 권장
 */
int32_t SMA_SetTargetForce(uint8_t ch, float force_n)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    if (force_n < 0.0f) {
        REPORT_ERROR_MSG("Target force must be positive");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];
    channel->target_force = force_n;

    if (channel->mode != SMA_MODE_FORCE_CONTROL) {
        SMA_SetMode(ch, SMA_MODE_FORCE_CONTROL);
    }

    return 0;
}

/**
 * @brief PID 게인 설정
 * @note 외부에서 smaChannelMutex 획득 후 호출 권장
 */
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
    channel->pid.integral = 0.0f;

    return 0;
}

/**
 * @brief 현재 온도 읽기
 * @note Thread-safe (atomic read)
 */
int32_t SMA_GetTemp(uint8_t ch, float *temp_c)
{
    if (ch >= SMA_MAX_CHANNELS || temp_c == NULL) {
        return -1;
    }

    *temp_c = sma_ctrl.channels[ch].current_temp;
    return 0;
}

/**
 * @brief 현재 PWM Duty 읽기
 * @note Thread-safe (atomic read)
 */
int32_t SMA_GetPWM(uint8_t ch, float *duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS || duty_pct == NULL) {
        return -1;
    }

    *duty_pct = sma_ctrl.channels[ch].pwm_duty;
    return 0;
}

/**
 * @brief 제어기 업데이트 (PID + PWM)
 * @note ControlTask에서 호출, 외부에서 smaChannelMutex 획득 후 호출
 * @note 팬 제어는 별도로 SMA_UpdateFans() 호출 필요
 */
void SMA_Update(void)
{
    if (!sma_ctrl.initialized) {
        return;
    }

    float dt = (float)sma_ctrl.control_period_ms / 1000.0f;

    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_Channel_t *channel = &sma_ctrl.channels[ch];

        // 온도 센서 읽기 (ISR-safe 버전 사용)
        float temp_c;
        if (Sensor_GetTemperature_ISR(ch, &temp_c) == 0) {
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
                // 오픈루프는 수동 제어
                break;

            case SMA_MODE_DISABLED:
            default:
                SMA_SetHardwarePWM(ch, 0.0f);
                channel->pwm_duty = 0.0f;
                break;
        }

        channel->last_update_ms = HAL_GetTick();
    }
}

/**
 * @brief 팬 제어 업데이트 (I2C 통신 포함)
 * @note ControlTask에서 호출, 외부에서 i2cMutex 획득 후 호출
 * @note SMA_Update()와 분리하여 I2C mutex 관리 용이하게 함
 */
void SMA_UpdateFans(void)
{
    if (!sma_ctrl.initialized) {
        return;
    }

    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_Channel_t *channel = &sma_ctrl.channels[ch];

        // 자동 팬 제어가 비활성화되어 있으면 건너뛰기
        if (!channel->fan_auto_enable) {
            continue;
        }

        // 모드별 팬 제어
        if (channel->mode == SMA_MODE_DISABLED) {
            channel->fan_duty = 0.0f;
            Fan6_SetDuty(ch, 0.0f);
            continue;
        }

        float fan_duty = 0.0f;

        if (channel->mode == SMA_MODE_TEMP_CONTROL && channel->target_temp > 0.0f) {
            fan_duty = SMA_CalculateFanDuty(channel->current_temp, channel->target_temp);
        }

        channel->fan_duty = fan_duty;
        Fan6_SetDuty(ch, fan_duty);
    }
}

/**
 * @brief 비상 정지
 * @note i2cMutex 내부에서 획득 시도
 */
void SMA_EmergencyStop(void)
{
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        sma_ctrl.channels[ch].mode = SMA_MODE_DISABLED;
        SMA_SetHardwarePWM(ch, 0.0f);
        sma_ctrl.channels[ch].pwm_duty = 0.0f;
        sma_ctrl.channels[ch].fan_duty = 0.0f;
    }

    // 팬 정지 (I2C 접근 필요)
    if (osMutexAcquire(i2cMutexHandle, 100) == osOK) {
        for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
            Fan6_SetDuty(ch, 0.0f);
        }
        osMutexRelease(i2cMutexHandle);
    }

    REPORT_ERROR_MSG("SMA Emergency Stop");
}

/**
 * @brief 채널 상태 읽기 (읽기 전용)
 * @note Thread-safe (포인터 반환, 읽기만)
 */
const SMA_Channel_t* SMA_GetChannelState(uint8_t ch)
{
    if (ch >= SMA_MAX_CHANNELS) {
        return NULL;
    }
    return &sma_ctrl.channels[ch];
}

/**
 * @brief 과열 상태 확인
 * @note Thread-safe (atomic read)
 */
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
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        TIM_HandleTypeDef *htim = pwm_map[ch].htim;
        uint32_t channel = pwm_map[ch].channel;

        if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) {
            REPORT_ERROR_MSG("PWM start failed");
        }

        SMA_SetHardwarePWM(ch, 0.0f);
    }
}

static void SMA_InitChannels(void)
{
    for (uint8_t ch = 0; ch < SMA_MAX_CHANNELS; ch++) {
        SMA_Channel_t *channel = &sma_ctrl.channels[ch];

        channel->mode = SMA_MODE_DISABLED;
        channel->target_temp = 0.0f;
        channel->current_temp = 0.0f;
        channel->pwm_duty = 0.0f;
        channel->overtemp_flag = 0;
        channel->last_update_ms = 0;

        channel->fan_duty = 0.0f;
        channel->fan_auto_enable = FAN_ENABLE_AUTO;

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

    if (channel->overtemp_flag) {
        SMA_SetHardwarePWM(ch, 0.0f);
        channel->pwm_duty = 0.0f;
        return;
    }

    if (channel->target_temp <= 0.0f) {
        SMA_SetHardwarePWM(ch, 0.0f);
        channel->pwm_duty = 0.0f;
        return;
    }

    // 프리히트 구간 (목표의 40%까지 풀 듀티)
    float preheat_threshold = SMA_PREHEAT_RATIO * channel->target_temp;

    if (channel->current_temp < preheat_threshold) {
        SMA_PID_Reset(&channel->pid);
        float pwm_output = SMA_PWM_MAX;
        channel->pwm_duty = pwm_output;
        SMA_SetHardwarePWM(ch, pwm_output);
        return;
    }

    // PID 온도 제어
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

    if (channel->overtemp_flag) {
        SMA_SetHardwarePWM(ch, 0.0f);
        channel->pwm_duty = 0.0f;
        return;
    }

    const SensorData_t *sensor = Sensor_GetData();
    if (sensor == NULL) {
        return;
    }

    float current_force = 0.0f;
    if (ch < SENSOR_MAX_CH) {
        current_force = (float)sensor->can.biotorq[ch] / 100.0f;
    }

    float pwm_output = SMA_PID_Compute(&channel->pid,
                                        channel->target_force,
                                        current_force,
                                        dt);

    channel->pwm_duty = pwm_output;
    SMA_SetHardwarePWM(ch, pwm_output);
}

static void SMA_SafetyCheck(uint8_t ch)
{
    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    if (channel->current_temp >= SMA_TEMP_MAX_CRIT) {
        if (!channel->overtemp_flag) {
            channel->overtemp_flag = 1;
            SMA_SetHardwarePWM(ch, 0.0f);
            channel->pwm_duty = 0.0f;

            REPORT_ERROR_MSG("Critical overtemp - shutdown");
        }
    }
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

    duty_pct = SMA_Clamp(duty_pct, 0.0f, 100.0f);

    TIM_HandleTypeDef *htim = pwm_map[ch].htim;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)((duty_pct * (float)arr) / 100.0f);

    __HAL_TIM_SET_COMPARE(htim, pwm_map[ch].channel, ccr);
}

float SMA_PID_Compute(SMA_PID_t *pid, float setpoint, float measurement, float dt)
{
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    float p_term = pid->kp * error;

    pid->integral += error * dt;
    float max_integral = 50.0f;
    pid->integral = SMA_Clamp(pid->integral, -max_integral, max_integral);
    float i_term = pid->ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    float output = p_term + i_term + d_term;
    output = SMA_Clamp(output, pid->output_min, pid->output_max);

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

static float SMA_CalculateFanDuty(float current_temp, float target_temp)
{
    if (target_temp <= 0.0f) {
        return 0.0f;
    }

    float temp_diff = current_temp - target_temp;

    if (temp_diff <= FAN_TEMP_DIFF_OFF) {
        return 0.0f;
    }

    if (temp_diff >= FAN_TEMP_DIFF_FULL) {
        return FAN_DUTY_MAX;
    }

    float duty = temp_diff * FAN_P_GAIN;
    duty = SMA_Clamp(duty, 0.0f, FAN_DUTY_MAX);

    return duty;
}

/* ========== Fan Control Functions ========== */

int32_t SMA_SetFanAutoControl(uint8_t ch, uint8_t enable)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    sma_ctrl.channels[ch].fan_auto_enable = enable ? 1 : 0;

    if (!enable) {
        sma_ctrl.channels[ch].fan_duty = 0.0f;
        // 팬 정지는 외부에서 i2cMutex 획득 후 Fan6_SetDuty 호출
    }

    return 0;
}

/**
 * @brief 팬 수동 제어
 * @note 외부에서 i2cMutex 획득 후 호출 권장
 */
int32_t SMA_SetFanDuty(uint8_t ch, float duty_pct)
{
    if (ch >= SMA_MAX_CHANNELS) {
        REPORT_ERROR_MSG("Invalid channel");
        return -1;
    }

    SMA_Channel_t *channel = &sma_ctrl.channels[ch];

    if (channel->fan_auto_enable) {
        REPORT_ERROR_MSG("Auto fan control is enabled");
        return -1;
    }

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








