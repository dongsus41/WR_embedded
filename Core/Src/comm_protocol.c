/*
 * comm_protocol.c - USART Communication Protocol Implementation
 */

#include "comm_protocol.h"
#include "sma_actuator.h"
#include "sensors.h"
#include "emc2303.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* ========== Private Variables ========== */
static UART_HandleTypeDef *comm_huart = NULL;
static TelemetryFrame_t telem_frame;

// 명령 수신 버퍼 (링 버퍼)
static char rx_buffer[COMM_CMD_BUFFER_SIZE];
static uint16_t rx_write_idx = 0;
static char cmd_buffer[COMM_CMD_BUFFER_SIZE];
static volatile bool cmd_ready = false;

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

    // 버퍼 초기화
    memset(&telem_frame, 0, sizeof(TelemetryFrame_t));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    rx_write_idx = 0;
    cmd_ready = false;

    // UART 수신 인터럽트 활성화 (단일 바이트 수신)
    // 참고: main.c에서 HAL_UART_Receive_IT 호출 필요
}

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

void Comm_ProcessCommands(void)
{
    // 명령이 준비되었는지 확인
    if (!cmd_ready) {
        return;
    }

    // 명령 파싱
    ParsedCommand_t parsed_cmd;
    if (Comm_ParseCommand(cmd_buffer, &parsed_cmd) == 0) {
        // 명령 실행
        if (Comm_ExecuteCommand(&parsed_cmd) == 0) {
            Comm_SendResponse("OK\r\n");
        } else {
            Comm_SendResponse("ERROR: Execution failed\r\n");
        }
    } else {
        Comm_SendResponse("ERROR: Invalid command\r\n");
    }

    // 버퍼 클리어
    cmd_ready = false;
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
}

void Comm_RxByteCallback(uint8_t data)
{
    // 명령 종료 문자 감지 ('\r' 또는 '\n')
    if (data == '\r' || data == '\n') {
        if (rx_write_idx > 0) {
            // 명령 버퍼에 복사
            memcpy(cmd_buffer, rx_buffer, rx_write_idx);
            cmd_buffer[rx_write_idx] = '\0';
            cmd_ready = true;

            // 수신 버퍼 리셋
            rx_write_idx = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }
        return;
    }

    // 버퍼 오버플로 방지
    if (rx_write_idx >= (COMM_CMD_BUFFER_SIZE - 1)) {
        rx_write_idx = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
        return;
    }

    // 문자 추가
    rx_buffer[rx_write_idx++] = data;
}

int32_t Comm_ParseCommand(const char *cmd_str, ParsedCommand_t *result)
{
    if (cmd_str == NULL || result == NULL) {
        return -1;
    }

    // 결과 초기화
    memset(result, 0, sizeof(ParsedCommand_t));
    result->valid = false;
    result->channel = 0xFF; // ALL

    char cmd_name[16] = {0};
    char ch_str[8] = {0};

    // 명령어 추출
    if (sscanf(cmd_str, "%15s", cmd_name) != 1) {
        return -1;
    }

    // 명령어 파싱
    if (strcmp(cmd_name, "MODE") == 0) {
        // MODE <ch> <mode> <target>
        // 예: MODE 0 TEMP 60.0 또는 MODE 1 FORCE 50.0
        char mode_str[16];
        int ch;

        if (sscanf(cmd_str, "MODE %d %15s %f", &ch, mode_str, &result->param1) == 3) {
            result->cmd_id = CMD_MODE;
            result->channel = (uint8_t)ch;

            // 모드 문자열을 숫자로 변환
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
        // PWM <ch> <duty>
        int ch;
        if (sscanf(cmd_str, "PWM %d %f", &ch, &result->param1) == 2) {
            result->cmd_id = CMD_PWM;
            result->channel = (uint8_t)ch;
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "PID") == 0) {
        // PID <ch> <kp> <ki> <kd>
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
        // STOP <ch|ALL>
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
        // FAN <ch> <duty>
        int ch;
        if (sscanf(cmd_str, "FAN %d %f", &ch, &result->param1) == 2) {
            result->cmd_id = CMD_FAN;
            result->channel = (uint8_t)ch;
            result->valid = true;
            return 0;
        }
    }
    else if (strcmp(cmd_name, "RESET") == 0) {
        // RESET <ch|ALL>
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

int32_t Comm_ExecuteCommand(const ParsedCommand_t *cmd)
{
    if (cmd == NULL || !cmd->valid) {
        return -1;
    }

    switch (cmd->cmd_id) {
        case CMD_MODE:
            // 제어 모드 변경
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }

            // 모드 설정
            if (SMA_SetMode(cmd->channel, (SMA_ControlMode_t)cmd->param2) != 0) {
                return -1;
            }

            // 목표값 설정
            if (cmd->param2 == CTRL_MODE_TEMP_CONTROL) {
                SMA_SetTargetTemp(cmd->channel, cmd->param1);
            } else if (cmd->param2 == CTRL_MODE_FORCE_CONTROL) {
                SMA_SetTargetForce(cmd->channel, cmd->param1);
            }
            break;

        case CMD_PWM:
            // PWM 직접 제어
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }
            return SMA_SetPWM(cmd->channel, cmd->param1);

        case CMD_PID:
            // PID 게인 설정
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }
            return SMA_SetPIDGains(cmd->channel, cmd->param1, cmd->param2, cmd->param3);

        case CMD_STOP:
            // 비상정지
            if (cmd->channel == 0xFF) {
                SMA_EmergencyStop();
            } else if (cmd->channel < COMM_MAX_CHANNELS) {
                SMA_SetMode(cmd->channel, SMA_MODE_DISABLED);
            } else {
                return -1;
            }
            break;

        case CMD_FAN:
            // 팬 제어
            if (cmd->channel >= COMM_MAX_CHANNELS) {
                return -1;
            }
            Fan6_SetDuty(cmd->channel, cmd->param1);
            break;

        case CMD_RESET:
            // 리셋 (PID 리셋)
            if (cmd->channel == 0xFF) {
                for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
                    SMA_SetMode(i, SMA_MODE_DISABLED);
                }
            } else if (cmd->channel < COMM_MAX_CHANNELS) {
                SMA_SetMode(cmd->channel, SMA_MODE_DISABLED);
            } else {
                return -1;
            }
            break;

        case CMD_STATUS:
            // 상태 요청 (텔레메트리 즉시 전송)
            Comm_SendTelemetry();
            break;

        default:
            return -1;
    }

    return 0;
}

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

    HAL_UART_Transmit(comm_huart, (uint8_t*)buffer, strlen(buffer), 100);
}

/* ========== Private Functions ========== */

static void Comm_BuildTelemetryFrame(TelemetryFrame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    // 헤더
    frame->header[0] = TELEM_HEADER_BYTE1;
    frame->header[1] = TELEM_HEADER_BYTE2;
    frame->timestamp_ms = HAL_GetTick();

    // 6채널 구동기 상태
    for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
        const SMA_Channel_t *ch = SMA_GetChannelState(i);
        if (ch != NULL) {
            frame->actuator[i].current_temp = ch->current_temp;
            frame->actuator[i].target_value = (ch->mode == SMA_MODE_TEMP_CONTROL) ?
                                               ch->target_temp : ch->target_force;
            frame->actuator[i].pwm_duty = ch->pwm_duty;
            frame->actuator[i].control_mode = (uint8_t)ch->mode;
            frame->actuator[i].fault_flag = ch->overtemp_flag;
        }
    }

    // 센서 데이터
    const SensorData_t *sensor = Sensor_GetData();
    if (sensor != NULL) {
        // 힘센서 (4채널)
        for (uint8_t i = 0; i < 4; i++) {
            frame->force_sensor[i] = sensor->can.biotorq[i];
        }

        // 변위센서 (추후 확장)
        frame->displacement[0] = 0;
        frame->displacement[1] = 0;
    }

    // 팬 상태 (SMA 채널에서 읽기)
    for (uint8_t i = 0; i < COMM_MAX_CHANNELS; i++) {
        const SMA_Channel_t *ch = SMA_GetChannelState(i);
        if (ch != NULL) {
            frame->fan_duty[i] = (uint8_t)(ch->fan_duty + 0.5f);  // 반올림하여 uint8_t로 변환
        } else {
            frame->fan_duty[i] = 0;
        }
    }

    // 시스템 상태
    frame->system_state = 0; // TODO: 시스템 상태 정의
}

static int32_t Comm_TransmitFrame(const uint8_t *data, uint16_t length)
{
    if (comm_huart == NULL || data == NULL) {
        return -1;
    }

    // DMA 사용 가능하면 HAL_UART_Transmit_DMA 사용
    HAL_StatusTypeDef status = HAL_UART_Transmit(comm_huart, (uint8_t*)data,
                                                   length, 100);

    return (status == HAL_OK) ? 0 : -1;
}

uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length)
{
    // CRC-16-CCITT (polynomial 0x1021)
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
