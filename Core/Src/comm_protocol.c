/*
 * comm_protocol.c - RTOS Version
 *
 * 변경사항:
 * - UART TX 시 uartTxMutex 사용
 * - Queue 기반 명령 수신 (ISR → Task)
 * - Comm_ProcessCommands() 제거 (CommandTask에서 직접 처리)
 */

#include "comm_protocol.h"
#include "common_defs.h"
#include "sma_actuator.h"
#include "sensors.h"
#include "emc2303.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* ========== External RTOS Objects ========== */
extern osMutexId_t uartTxMutexHandle;
extern osMutexId_t smaChannelMutexHandle;
extern osMutexId_t i2cMutexHandle;

/* ========== Private Variables ========== */
static UART_HandleTypeDef *comm_huart = NULL;
static TelemetryFrame_t telem_frame;

/* ========== Private Function Prototypes ========== */
static void Comm_BuildTelemetryFrame(TelemetryFrame_t *frame);
static int32_t Comm_TransmitFrame(const uint8_t *data, uint16_t length);

/* ========== Public Functions ========== */

void Comm_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        REPORT_ERROR_MSG("NULL UART handle");
        return;
    }

    comm_huart = huart;

    memset(&telem_frame, 0, sizeof(TelemetryFrame_t));

    REPORT_INFO_MSG("Comm Protocol Initialized (RTOS)");
}

/**
 * @brief 텔레메트리 프레임 생성 및 전송
 * @note TelemetryTask에서 호출, 외부에서 uartTxMutex 획득 후 호출 권장
 */
int32_t Comm_SendTelemetry(void)
{
    if (comm_huart == NULL) {
        return -1;
    }

    // 텔레메트리 프레임 생성
    Comm_BuildTelemetryFrame(&telem_frame);

    // CRC 계산 (CRC 필드 제외)
    uint16_t crc = Comm_CalculateCRC16((uint8_t*)&telem_frame,
                                        sizeof(TelemetryFrame_t) - 2);
    telem_frame.crc16 = crc;

    // UART로 전송
    return Comm_TransmitFrame((uint8_t*)&telem_frame, sizeof(TelemetryFrame_t));
}

/**
 * @brief 텔레메트리 전송 (mutex 포함 버전)
 * @note 단독으로 호출할 때 사용
 */
int32_t Comm_SendTelemetry_Safe(void)
{
    if (comm_huart == NULL) {
        return -1;
    }

    if (osMutexAcquire(uartTxMutexHandle, 50) == osOK) {
        int32_t result = Comm_SendTelemetry();
        osMutexRelease(uartTxMutexHandle);
        return result;
    }

    return -1;  // Mutex timeout
}

/**
 * @brief 명령 문자열 파싱
 * @note CommandTask에서 호출
 */
int32_t Comm_ParseCommand(const char *cmd_str, ParsedCommand_t *result)
{
    if (cmd_str == NULL || result == NULL) {
        return -1;
    }

    memset(result, 0, sizeof(ParsedCommand_t));
    result->valid = false;
    result->channel = 0xFF;

    char cmd_name[16] = {0};
    char ch_str[8] = {0};

    if (sscanf(cmd_str, "%15s", cmd_name) != 1) {
        return -1;
    }

    if (strcmp(cmd_name, "MODE") == 0) {
        char mode_str[16];
        int ch;

        if (sscanf(cmd_str, "MODE %d %15s %f", &ch, mode_str, &result->param1) == 3) {
            result->cmd_id = CMD_MODE;
            result->channel = (uint8_t)ch;

            if (strcmp(mode_str, "DISABLED") == 0 || strcmp(mode_str, "0") == 0) {
                result->param2 = CTRL_MODE_DISABLED;
            } else if (strcmp(mode_str, "OPEN") == 0 || strcmp(mode_str, "1") == 0) {
                result->param2 = CTRL_MODE_OPEN_LOOP;
            } else if (strcmp(mode_str, "TEMP") == 0 || strcmp(mode_str, "2") == 0) {
                result->param2 = CTRL_MODE_TEMP_CONTROL;
            } else if (strcmp(mode_str, "FORCE") == 0 || strcmp(mode_str, "3") == 0) {
                result->param2 = CTRL_MODE_FORCE_CONTROL;
            } else {
                return -1;
            }
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "PWM") == 0) {
        int ch;
        if (sscanf(cmd_str, "PWM %d %f", &ch, &result->param1) == 2) {
            result->cmd_id = CMD_PWM;
            result->channel = (uint8_t)ch;
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "PID") == 0) {
        int ch;
        if (sscanf(cmd_str, "PID %d %f %f %f", &ch,
                   &result->param1, &result->param2, &result->param3) == 4) {
            result->cmd_id = CMD_PID;
            result->channel = (uint8_t)ch;
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "STOP") == 0) {
        if (sscanf(cmd_str, "STOP %7s", ch_str) == 1) {
            result->cmd_id = CMD_STOP;
            if (strcmp(ch_str, "ALL") == 0) {
                result->channel = 0xFF;
            } else {
                result->channel = (uint8_t)atoi(ch_str);
            }
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "FAN") == 0) {
        int ch;
        if (sscanf(cmd_str, "FAN %d %f", &ch, &result->param1) == 2) {
            result->cmd_id = CMD_FAN;
            result->channel = (uint8_t)ch;
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "RESET") == 0) {
        if (sscanf(cmd_str, "RESET %7s", ch_str) == 1) {
            result->cmd_id = CMD_RESET;
            if (strcmp(ch_str, "ALL") == 0) {
                result->channel = 0xFF;
            } else {
                result->channel = (uint8_t)atoi(ch_str);
            }
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "STATUS") == 0) {
        result->cmd_id = CMD_STATUS;
        result->valid = true;
        return 0;
    }

    return -1;
}

/**
 * @brief 명령 실행 (RTOS 버전)
 * @note CommandTask에서 호출, 필요한 mutex를 내부에서 획득
 */
int32_t Comm_ExecuteCommand(const ParsedCommand_t *cmd)
{
    if (cmd == NULL || !cmd->valid) {
        return -1;
    }

    int32_t result = 0;

    switch (cmd->cmd_id) {
        case CMD_MODE:
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }

            // SMA 채널 mutex 획득
            if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                if (SMA_SetMode(cmd->channel, (SMA_ControlMode_t)cmd->param2) != 0) {
                    result = -1;
                } else {
                    if (cmd->param2 == CTRL_MODE_TEMP_CONTROL) {
                        SMA_SetTargetTemp(cmd->channel, cmd->param1);
                    } else if (cmd->param2 == CTRL_MODE_FORCE_CONTROL) {
                        SMA_SetTargetForce(cmd->channel, cmd->param1);
                    }
                }
                osMutexRelease(smaChannelMutexHandle);
            } else {
                result = -1;
            }
            break;

        case CMD_PWM:
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }

            if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                result = SMA_SetPWM(cmd->channel, cmd->param1);
                osMutexRelease(smaChannelMutexHandle);
            } else {
                result = -1;
            }
            break;

        case CMD_PID:
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }

            if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                result = SMA_SetPIDGains(cmd->channel, cmd->param1, cmd->param2, cmd->param3);
                osMutexRelease(smaChannelMutexHandle);
            } else {
                result = -1;
            }
            break;

        case CMD_STOP:
            // 비상정지는 내부에서 mutex 획득
            if (cmd->channel == 0xFF) {
                SMA_EmergencyStop();
            } else if (cmd->channel < COMM_MAX_CHANNELS) {
                if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                    SMA_SetMode(cmd->channel, SMA_MODE_DISABLED);
                    osMutexRelease(smaChannelMutexHandle);
                }
            } else {
                result = -1;
            }
            break;

        case CMD_FAN:
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }

            // I2C mutex 획득 (팬 제어)
            if (osMutexAcquire(i2cMutexHandle, 100) == osOK) {
                Fan6_SetDuty(cmd->channel, cmd->param1);
                osMutexRelease(i2cMutexHandle);
            } else {
                result = -1;
            }
            break;

        case CMD_RESET:
            if (cmd->channel == 0xFF) {
                if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                    for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
                        SMA_SetMode(i, SMA_MODE_DISABLED);
                    }
                    osMutexRelease(smaChannelMutexHandle);
                }
            } else if (cmd->channel < COMM_MAX_CHANNELS) {
                if (osMutexAcquire(smaChannelMutexHandle, 100) == osOK) {
                    SMA_SetMode(cmd->channel, SMA_MODE_DISABLED);
                    osMutexRelease(smaChannelMutexHandle);
                }
            } else {
                result = -1;
            }
            break;

        case CMD_STATUS:
            // 텔레메트리 즉시 전송
            Comm_SendTelemetry_Safe();
            break;

        default:
            result = -1;
            break;
    }

    return result;
}

/**
 * @brief 응답 메시지 전송
 * @note mutex 포함 버전
 */
void Comm_SendResponse(const char *format, ...)
{
    if (comm_huart == NULL || format == NULL) {
        return;
    }

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (osMutexAcquire(uartTxMutexHandle, 50) == osOK) {
        HAL_UART_Transmit(comm_huart, (uint8_t*)buffer, strlen(buffer), 100);
        osMutexRelease(uartTxMutexHandle);
    }
}

/**
 * @brief 응답 메시지 전송 (mutex 없음)
 * @note 외부에서 uartTxMutex 획득 후 호출
 */
void Comm_SendResponse_Unsafe(const char *format, ...)
{
    if (comm_huart == NULL || format == NULL) {
        return;
    }

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(comm_huart, (uint8_t*)buffer, strlen(buffer), 100);
}

/* ========== Private Functions ========== */

static void Comm_BuildTelemetryFrame(TelemetryFrame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    frame->header[0] = TELEM_HEADER_BYTE1;
    frame->header[1] = TELEM_HEADER_BYTE2;
    frame->timestamp_ms = HAL_GetTick();

    // 6채널 구동기 상태
    for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
        if (IS_CHANNEL_ENABLED(i)) {
            const SMA_Channel_t *ch = SMA_GetChannelState(i);
            if (ch != NULL) {
                frame->actuator[i].current_temp = ch->current_temp;
                frame->actuator[i].target_value = (ch->mode == SMA_MODE_TEMP_CONTROL) ?
                                                   ch->target_temp : ch->target_force;
                frame->actuator[i].pwm_duty = ch->pwm_duty;
                frame->actuator[i].control_mode = (uint8_t)ch->mode;
                frame->actuator[i].fault_flag = ch->overtemp_flag;
            }
        } else {
            memset(&frame->actuator[i], 0, sizeof(ActuatorStatus_t));
        }
    }

    // 센서 데이터
#if CAN_SENSORS_ENABLE
    const SensorData_t *sensor = Sensor_GetData();
    if (sensor != NULL) {
        // 힘센서 (4채널)
        for (uint8_t i = 0; i < SENSOR_FORCE_CH; i++) {
            frame->force_sensor[i] = sensor->can.pwr[i];
        }
        // 변위센서 (5채널, ID 0x101~0x105 = 인덱스 1~5)
        for (uint8_t i = 0; i < SENSOR_DISP_CH; i++) {
            frame->displacement[i] = sensor->can.displacement[i + 1];
        }
    }
#else
    memset(frame->force_sensor, 0, sizeof(frame->force_sensor));
    memset(frame->displacement, 0, sizeof(frame->displacement));
#endif

    // 팬 상태
    for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
        if (IS_CHANNEL_ENABLED(i)) {
            const SMA_Channel_t *ch = SMA_GetChannelState(i);
            if (ch != NULL) {
                frame->fan_duty[i] = (uint8_t)(ch->fan_duty + 0.5f);
            } else {
                frame->fan_duty[i] = 0;
            }
        } else {
            frame->fan_duty[i] = 0;
        }
    }

    frame->system_state = 0;
}

static int32_t Comm_TransmitFrame(const uint8_t *data, uint16_t length)
{
    if (comm_huart == NULL || data == NULL) {
        return -1;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(comm_huart, (uint8_t*)data,
                                                   length, 100);

    return (status == HAL_OK) ? 0 : -1;
}

uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}
