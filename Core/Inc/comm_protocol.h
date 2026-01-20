/*
 * comm_protocol.h - USART Communication Protocol
 *
 * 웨어러블 로봇 제어 시스템 통신 프로토콜
 * - 텔레메트리 데이터 송신 (바이너리 프레임)
 * - 명령 수신 및 파싱 (텍스트 기반)
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "sensors.h"

/* ========== Configuration ========== */
#define COMM_TELEMETRY_PERIOD_MS    10      // 텔레메트리 전송 주기 (100Hz)
#define COMM_CMD_BUFFER_SIZE        128     // 명령 버퍼 크기
#define COMM_MAX_CHANNELS           6       // 최대 채널 수

/* ========== Telemetry Frame ========== */
#define TELEM_HEADER_BYTE1          0xAA
#define TELEM_HEADER_BYTE2          0x55
#define TELEM_FRAME_SIZE            130      // 텔레메트리 프레임 총 크기

/* 채널별 구동기 상태 */
typedef struct __attribute__((packed)) {
    float    current_temp;      // 현재 온도 (°C)
    float    target_value;      // 목표값 (온도 또는 힘)
    float    pwm_duty;          // PWM Duty (%)
    uint8_t  control_mode;      // 제어 모드 (0=DISABLED, 1=OPEN, 2=TEMP, 3=FORCE)
    uint8_t  fault_flag;        // 에러 플래그 (0=정상, 1=과열)
    uint8_t  reserved[2];       // 정렬을 위한 예약 (4바이트 정렬)
} ActuatorStatus_t;             // 16 bytes per channel

/* 텔레메트리 프레임 구조체 */
typedef struct __attribute__((packed)) {
    // Header (6 bytes)
    uint8_t  header[2];         // 0xAA, 0x55
    uint32_t timestamp_ms;      // 타임스탬프 (ms)

    // 6채널 구동기 상태 (6 x 16 = 96 bytes)
    ActuatorStatus_t actuator[COMM_MAX_CHANNELS];

    // 센서 데이터 (18 bytes)
    uint16_t force_sensor[SENSOR_FORCE_CH];     // 힘센서 4개
    uint16_t displacement[SENSOR_DISP_CH];      // 변위센서 5개 

    // 팬 상태 (6 bytes)
    uint8_t  fan_duty[6];       // 팬 PWM Duty (%)

    // 시스템 상태 (2 bytes)
    uint8_t  system_state;      // 시스템 상태
    uint8_t  reserved;          // 예약

    // CRC (2 bytes)
    uint16_t crc16;             // CRC-16 체크섬
} TelemetryFrame_t;             // Total: 130 bytes (2+4+96+8+10+6+1+1+2)

/* ========== Command Protocol ========== */

/* 명령 ID */
typedef enum {
    CMD_MODE = 0,       // 제어 모드 변경: MODE <ch> <mode> <target>
    CMD_PWM,            // PWM 직접 제어: PWM <ch> <duty>
    CMD_PID,            // PID 게인 설정: PID <ch> <kp> <ki> <kd>
    CMD_STOP,           // 비상정지: STOP <ch|ALL>
    CMD_FAN,            // 팬 제어: FAN <ch> <duty>
    CMD_RESET,          // 리셋: RESET <ch|ALL>
    CMD_STATUS,         // 상태 요청: STATUS
    CMD_UNKNOWN
} CommandID_t;

/* 제어 모드 (텔레메트리와 공통) */
typedef enum {
    CTRL_MODE_DISABLED = 0,
    CTRL_MODE_OPEN_LOOP = 1,
    CTRL_MODE_TEMP_CONTROL = 2,
    CTRL_MODE_FORCE_CONTROL = 3,
    CTRL_MODE_POSITION_CONTROL = 4
} ControlMode_t;

/* 명령 파싱 결과 */
typedef struct {
    CommandID_t cmd_id;         // 명령 ID
    uint8_t     channel;        // 채널 번호 (0-5, 0xFF=ALL)
    float       param1;         // 파라미터 1
    float       param2;         // 파라미터 2
    float       param3;         // 파라미터 3
    bool        valid;          // 파싱 성공 여부
} ParsedCommand_t;

/* ========== Public Functions ========== */

/**
 * @brief 통신 프로토콜 초기화
 * @param huart UART 핸들 포인터
 */
void Comm_Init(UART_HandleTypeDef *huart);

/**
 * @brief 텔레메트리 프레임 생성 및 전송
 * @note 주기적으로 호출 (50ms 권장)
 * @return 0=성공, -1=실패
 */
int32_t Comm_SendTelemetry(void);

/**
 * @brief 수신된 명령 처리
 * @note UART 인터럽트 또는 메인 루프에서 호출
 */
void Comm_ProcessCommands(void);

/**
 * @brief 단일 바이트 수신 (UART 인터럽트 콜백에서 호출)
 * @param data 수신된 바이트
 */
void Comm_RxByteCallback(uint8_t data);

/**
 * @brief 명령 문자열 파싱
 * @param cmd_str 명령 문자열 (null-terminated)
 * @param result 파싱 결과 출력
 * @return 0=성공, -1=실패
 */
int32_t Comm_ParseCommand(const char *cmd_str, ParsedCommand_t *result);

/**
 * @brief 명령 실행
 * @param cmd 파싱된 명령
 * @return 0=성공, -1=실패
 */
int32_t Comm_ExecuteCommand(const ParsedCommand_t *cmd);

/**
 * @brief 응답 메시지 전송
 * @param format printf 형식 문자열
 */
void Comm_SendResponse(const char *format, ...);

/* ========== Private Functions ========== */

/**
 * @brief CRC-16 계산 (CCITT)
 * @param data 데이터 버퍼
 * @param length 데이터 길이
 * @return CRC-16 값
 */
uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length);

/**
 * @brief 텔레메트리 전송 (mutex 자동 획득)
 * @return 0=성공, -1=실패
 */
int32_t Comm_SendTelemetry_Safe(void);

/**
 * @brief 응답 전송 (mutex 없이)
 * @note 외부에서 uartTxMutex 획득 필요
 */
void Comm_SendResponse_Unsafe(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */
