# CLAUDE.md

이 파일은 Claude Code (claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

STM32H7 기반 6채널 SMA(형상기억합금) 소프트 로봇 액추에이터 제어 시스템. 웨어러블 로봇용 FreeRTOS 실시간 OS와 다중 프로토콜 센서 통합 (CAN, ADC, UART).

**MCU**: STM32H725RGVx (Cortex-M7 @ 560 MHz)
**RTOS**: FreeRTOS v10.3.1 + CMSIS-RTOS V2 API

## 빌드 명령어

```bash
# 빌드
make -C Debug all

# 클린
make -C Debug clean
```

**출력**: `Debug/260110_softrobot_tempfan_v3.3.elf`

ST-LINK 디버거로 STM32CubeIDE에서 플래시.

## 아키텍처

### RTOS 태스크 (`Core/Src/freertos.c`)

| 태스크 | 우선순위 | 주기 | 역할 |
|--------|----------|------|------|
| ControlTask | 56 (Realtime) | 10ms | SMA PWM 제어 + PID 루프 |
| TelemetryTask | 40 (High) | 12ms | 바이너리 텔레메트리 전송 |
| CommandTask | 24 (Normal) | 이벤트 | UART 텍스트 명령 파싱 |
| NRF70_Task | 25 | 1ms | WiFi 모듈 (비활성) |

### 핵심 모듈

- **`sma_actuator.c/h`**: 6채널 SMA 제어, PID, 안전 제한 (100°C 최대, 120°C 자동 차단)
- **`sensors.c/h`**: 센서 통합 (CAN 힘/변위 + ADC 온도)
- **`comm_protocol.c/h`**: 바이너리 텔레메트리 (130바이트 @ 83Hz) + 텍스트 명령 파서
- **`emc2303.c/h`**: I2C 6채널 팬 컨트롤러 (EMC2303 2개)
- **`common_defs.h`**: 채널 활성화 플래그, CAN ID, GPIO 정의

### 하드웨어 인터페이스

- **ADC1**: 6채널 온도 센서 (TMP235), DMA 순환 버퍼
- **FDCAN1**: 변위 센서 (CAN ID 0x100-0x10F)
- **FDCAN2**: 전력/정전용량 센서 (CAN ID 0x001-0x003)
- **USART3**: 115200 baud 텔레메트리 + 명령
- **I2C4**: EMC2303 팬 컨트롤러 @ 0x2E, 0x2C
- **TIM1/TIM2**: SMA 히터 PWM 출력 (20 kHz)

### 동기화

뮤텍스: `sensorDataMutex`, `smaChannelMutex`, `uartTxMutex`, `i2cMutex`

메시지 큐: `uartRxQueue` (512바이트), `cmdQueue` (128바이트 × 16)

## 코드 컨벤션

- **함수**: `Module_Action()` (예: `Sensor_GetTemperature()`)
- **타입**: `TitleCase_t` (예: `SMA_ControlMode_t`)
- **매크로**: `UPPER_CASE`
- **반환값**: 0 = 성공, -1 = 실패

채널 활성화/비활성화는 `common_defs.h`에서:
```c
#define CH0_ENABLE 0  // Forearm L (비활성)
#define CH4_ENABLE 1  // Waist L (활성)
```

## 주요 파일

- `Core/Inc/main.h` - 시스템 상태 구조체
- `Core/Src/freertos.c` - 태스크 정의 및 RTOS 초기화
- `Core/Src/sma_actuator.c` - 제어 알고리즘 및 PID
- `Core/Inc/FreeRTOSConfig.h` - RTOS 커널 설정
- `STM32H725RGVX_FLASH.ld` - 링커 스크립트
- `260110_softrobot_main_v3.3.ioc` - STM32CubeMX 설정

## 알려진 이슈

자세한 내용은 `docs/RTOS_Issues_and_Improvements.md` 참조.

**Critical**: TelemetryTask가 blocking UART TX 사용 (`HAL_UART_Transmit`), CPU ~94% 점유. 권장 수정: DMA 기반 전송.

**High**: ControlTask와 TelemetryTask 간 뮤텍스 우선순위 역전 위험.

## 명령 프로토콜

UART 텍스트 명령 (개행 종료):
```
MODE <ch> <mode> <target>  - 제어 모드 설정 (DISABLED/OPEN/TEMP/FORCE/POS)
PWM <ch> <duty>            - 오픈 루프 PWM (0-100%)
PID <ch> <kp> <ki> <kd>    - PID 튜닝
STOP <ch|ALL>              - 긴급 정지
FAN <ch> <duty>            - 수동 팬 제어
STATUS                     - 상태 요청
```

## 작업 추적

개발 작업 진행 시 `docs/TASK_*.md` 파일을 생성하여 체크리스트로 진행 상황 추적.
예: `docs/TASK_RTOS_DMA.md` - RTOS DMA 전환 작업용 PRD 및 체크리스트.

## 서브모듈

`nrf70-bm/` - Nordic nRF70 WiFi 드라이버 (별도 빌드 시스템, 현재 메인 앱에서 비활성).
