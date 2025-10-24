# SMA Actuator Control Library

형상기억합금(Shape Memory Alloy) 소프트 액추에이터 제어 라이브러리

## 개요

이 라이브러리는 STM32H7 기반 시스템에서 6채널 SMA 소프트 액추에이터를 제어하기 위한 통합 솔루션입니다.

### 주요 기능

- **6채널 PWM 제어**: TIM2/TIM3/TIM4를 이용한 전열 구동 제어
- **온도 피드백 제어**: ADC를 통한 실시간 온도 모니터링
- **다중 제어 모드**:
  - 오픈루프 (Open-loop): PWM 직접 제어
  - 온도 제어 (Temperature Control): PID 기반 자동 온도 제어
  - 힘 제어 (Force Control): 추후 구현 예정
  - 위치 제어 (Position Control): 추후 구현 예정
- **안전 기능**: 과열 방지, 위험 온도 자동 차단

## 하드웨어 구성

### PWM 출력 핀 매핑

| 채널 | 타이머 | GPIO 핀 | 용도 |
|------|--------|---------|------|
| CH0  | TIM2_CH1 | PA15 | SMA 액추에이터 #0 |
| CH1  | TIM2_CH2 | PB3  | SMA 액추에이터 #1 |
| CH2  | TIM3_CH1 | PB4  | SMA 액추에이터 #2 |
| CH3  | TIM3_CH2 | PB5  | SMA 액추에이터 #3 |
| CH4  | TIM4_CH1 | PB6  | SMA 액추에이터 #4 |
| CH5  | TIM4_CH2 | PB7  | SMA 액추에이터 #5 |

### 온도 센서 핀 매핑

| 채널 | ADC 채널 | GPIO 핀 | 센서 |
|------|----------|---------|------|
| 0 | ADC1_INP10 | PC0 | TMP235 #0 |
| 1 | ADC1_INP11 | PC1 | TMP235 #1 |
| 2 | ADC1_INP17 | PA1 | TMP235 #2 |
| 3 | ADC1_INP14 | PA2 | TMP235 #3 |
| 4 | ADC1_INP15 | PA3 | TMP235 #4 |
| 5 | ADC1_INP18 | PA4 | TMP235 #5 |

## 사용법

### 1. 초기화

```c
#include "sma_actuator.h"

int main(void)
{
    // 시스템 초기화...

    // SMA 컨트롤러 초기화 (제어 주기: 10ms)
    SMA_Init(10);

    // ...
}
```

### 2. 오픈루프 제어 예제

```c
// 채널 0을 오픈루프 모드로 설정
SMA_SetMode(0, SMA_MODE_OPEN_LOOP);

// PWM Duty를 50%로 설정
SMA_SetPWM(0, 50.0f);
```

### 3. 온도 제어 예제

```c
// 채널 1을 온도 제어 모드로 설정
SMA_SetMode(1, SMA_MODE_TEMP_CONTROL);

// 목표 온도를 60°C로 설정
SMA_SetTargetTemp(1, 60.0f);

// PID 게인 조정 (선택 사항)
SMA_SetPIDGains(1, 5.0f, 0.1f, 0.5f);
```

### 4. 제어 루프 업데이트

타이머 인터럽트 또는 메인 루프에서 주기적으로 호출:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        // SMA 제어기 업데이트 (10ms 주기)
        SMA_Update();
    }
}
```

### 5. 상태 읽기

```c
// 온도 읽기
float temp;
SMA_GetTemp(0, &temp);
printf("CH0 Temperature: %.1f°C\n", temp);

// PWM Duty 읽기
float duty;
SMA_GetPWM(0, &duty);
printf("CH0 PWM Duty: %.1f%%\n", duty);

// 과열 상태 확인
if (SMA_IsOverTemp(0)) {
    printf("WARNING: CH0 Over Temperature!\n");
}
```

### 6. 비상 정지

```c
// 모든 채널 즉시 정지
SMA_EmergencyStop();
```

## API 레퍼런스

### 초기화 및 설정

- `void SMA_Init(uint32_t control_period_ms)` - 컨트롤러 초기화
- `int32_t SMA_SetMode(uint8_t ch, SMA_ControlMode_t mode)` - 제어 모드 설정
- `int32_t SMA_SetPIDGains(uint8_t ch, float kp, float ki, float kd)` - PID 게인 설정

### 제어

- `int32_t SMA_SetPWM(uint8_t ch, float duty_pct)` - PWM 직접 제어 (오픈루프)
- `int32_t SMA_SetTargetTemp(uint8_t ch, float temp_c)` - 목표 온도 설정
- `void SMA_Update(void)` - 제어 루프 업데이트 (주기 호출 필요)
- `void SMA_EmergencyStop(void)` - 비상 정지

### 상태 읽기

- `int32_t SMA_GetTemp(uint8_t ch, float *temp_c)` - 현재 온도 읽기
- `int32_t SMA_GetPWM(uint8_t ch, float *duty_pct)` - 현재 PWM Duty 읽기
- `uint8_t SMA_IsOverTemp(uint8_t ch)` - 과열 상태 확인
- `const SMA_Channel_t* SMA_GetChannelState(uint8_t ch)` - 채널 상태 구조체 읽기

## 제어 모드

### SMA_MODE_DISABLED
- 채널 비활성화
- PWM 0% 출력

### SMA_MODE_OPEN_LOOP
- 오픈루프 PWM 제어
- 사용자가 직접 PWM Duty 설정
- 온도 피드백 없음

### SMA_MODE_TEMP_CONTROL
- PID 기반 온도 제어
- 자동으로 PWM을 조절하여 목표 온도 유지
- 과열 보호 기능 포함

### SMA_MODE_FORCE_CONTROL (추후 구현)
- 힘 센서 피드백 기반 제어
- 목표 힘 유지

### SMA_MODE_POSITION_CONTROL (추후 구현)
- 위치 센서 피드백 기반 제어
- 목표 위치 유지

## 안전 기능

### 과열 보호

- **경고 온도**: 80°C (TEMP_WARN_THRESHOLD)
- **안전 최대 온도**: 100°C (SMA_TEMP_MAX_SAFE)
- **위험 온도**: 120°C (SMA_TEMP_MAX_CRIT)

위험 온도 도달 시 자동으로 해당 채널 PWM을 0%로 설정하고 과열 플래그 설정

### PWM 한계

- **최소**: 0%
- **최대**: 100%

## PID 제어기

### 기본 게인 (Default Gains)

- **Kp**: 5.0 (비례 게인)
- **Ki**: 0.1 (적분 게인)
- **Kd**: 0.5 (미분 게인)

### Anti-windup

적분 누적 한계: ±50.0

## 파일 구조

```
Core/
├── Inc/
│   └── sma_actuator.h      # 헤더 파일
└── Src/
    └── sma_actuator.c      # 구현 파일
```

## 의존성

- STM32 HAL 라이브러리
- sensors.h/sensors.c (온도 센서 모듈)
- tim.h (타이머 설정)
- debug.h (에러 리포팅)

## 빌드

CMake 프로젝트에 자동으로 포함됩니다.

```bash
cmake -B build
cmake --build build
```

## 주의사항

1. **제어 주기**: `SMA_Update()`는 초기화 시 설정한 주기(예: 10ms)로 정확히 호출되어야 합니다.
2. **온도 센서**: 온도 센서가 올바르게 연결되고 `Sensor_Init()`로 초기화되어야 합니다.
3. **과열 방지**: 과열 플래그가 설정되면 온도가 안전 수준(SMA_TEMP_MAX_SAFE - 5°C) 이하로 떨어질 때까지 해당 채널은 차단됩니다.
4. **PID 튜닝**: 시스템에 따라 PID 게인을 조정해야 할 수 있습니다.

## 추후 개발 계획

- [ ] 힘 제어 모드 구현
- [ ] 위치 제어 모드 구현
- [ ] 상위 제어기 통신 인터페이스 (CAN/UART)
- [ ] 데이터 로깅 기능
- [ ] 캘리브레이션 기능

## 라이선스

STMicroelectronics 소프트웨어 라이선스 참조

---

**작성일**: 2025-10-24
**버전**: 1.0
**작성자**: Claude Code
