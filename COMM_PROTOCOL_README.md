# USART Communication Protocol

웨어러블 로봇 제어 시스템 USART 통신 프로토콜

## 개요

이 프로토콜은 STM32H7 기반 웨어러블 로봇 제어 시스템과 상위 제어기(PC/모바일) 간의 양방향 통신을 제공합니다.

### 통신 구성

- **인터페이스**: USART3
- **보레이트**: 115200 bps (설정 가능)
- **데이터 형식**: 8N1 (8 data bits, No parity, 1 stop bit)
- **통신 방향**:
  - **송신 (텔레메트리)**: 96바이트 바이너리 프레임, 50ms 주기 (20Hz)
  - **수신 (명령)**: 텍스트 기반, 비동기

---

## 텔레메트리 프레임 (송신)

### 프레임 구조

총 96바이트 바이너리 프레임, CRC-16 체크섬 포함

```
+--------+------------+------------------+----------------+----------+--------+--------+
| Header | Timestamp  | Actuator Status  | Sensor Data    | Fan Duty | System | CRC-16 |
| 2bytes | 4bytes     | 72bytes (6ch)    | 12bytes        | 6bytes   | 2bytes | 2bytes |
+--------+------------+------------------+----------------+----------+--------+--------+
```

### 상세 필드

#### Header (2 bytes)
- Byte 0: `0xAA`
- Byte 1: `0x55`

#### Timestamp (4 bytes)
- `uint32_t`: 시스템 타임스탬프 (ms)

#### Actuator Status (72 bytes = 6 channels × 12 bytes)

각 채널당:
```c
struct ActuatorStatus {
    float   current_temp;    // 현재 온도 (°C)
    float   target_value;    // 목표값 (온도 또는 힘)
    float   pwm_duty;        // PWM Duty (%)
    uint8_t control_mode;    // 제어 모드 (0~4)
    uint8_t fault_flag;      // 에러 플래그
    uint8_t reserved[2];     // 예약 (정렬용)
};  // 12 bytes
```

**제어 모드 (control_mode)**:
- `0` = DISABLED (비활성)
- `1` = OPEN_LOOP (오픈루프)
- `2` = TEMP_CONTROL (온도제어)
- `3` = FORCE_CONTROL (힘제어)
- `4` = POSITION_CONTROL (위치제어, 추후)

**에러 플래그 (fault_flag)**:
- `0` = 정상
- `1` = 과열

#### Sensor Data (12 bytes)
```c
struct SensorData {
    uint16_t force_sensor[4];       // 힘센서 4개 (8 bytes)
    uint16_t displacement[2];       // 변위센서 2개 (4 bytes, 추후)
};  // 12 bytes
```

#### Fan Duty (6 bytes)
- `uint8_t fan_duty[6]`: 팬 PWM Duty (%) × 6채널

#### System State (2 bytes)
- `uint8_t system_state`: 시스템 상태
- `uint8_t reserved`: 예약

#### CRC-16 (2 bytes)
- CRC-16-CCITT (polynomial 0x1021)
- 헤더부터 system_state까지 계산

### 파싱 예제 (Python)

```python
import struct

def parse_telemetry(data):
    if len(data) != 96:
        return None

    # 헤더 확인
    if data[0] != 0xAA or data[1] != 0x55:
        return None

    # CRC 검증
    crc_calc = calculate_crc16(data[:-2])
    crc_recv = struct.unpack('<H', data[-2:])[0]
    if crc_calc != crc_recv:
        return None

    offset = 2
    timestamp = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4

    actuators = []
    for i in range(6):
        temp, target, duty, mode, fault = struct.unpack('<fffBBxx', data[offset:offset+12])
        actuators.append({
            'temp': temp,
            'target': target,
            'duty': duty,
            'mode': mode,
            'fault': fault
        })
        offset += 12

    forces = struct.unpack('<4H', data[offset:offset+8])
    offset += 8

    displacements = struct.unpack('<2H', data[offset:offset+4])
    offset += 4

    fans = struct.unpack('<6B', data[offset:offset+6])
    offset += 6

    sys_state = data[offset]

    return {
        'timestamp': timestamp,
        'actuators': actuators,
        'forces': forces,
        'displacements': displacements,
        'fans': fans,
        'system_state': sys_state
    }
```

---

## 명령 프로토콜 (수신)

### 명령 형식

텍스트 기반, ASCII 형식, `\r` 또는 `\n`으로 종료

```
<COMMAND> <PARAM1> <PARAM2> ... \r\n
```

### 지원 명령

#### 1. MODE - 제어 모드 변경

```
MODE <channel> <mode> <target>
```

**파라미터**:
- `<channel>`: 채널 번호 (0-5)
- `<mode>`: 제어 모드
  - `DISABLED` 또는 `0`: 비활성
  - `OPEN` 또는 `1`: 오픈루프
  - `TEMP` 또는 `2`: 온도제어
  - `FORCE` 또는 `3`: 힘제어
- `<target>`: 목표값 (온도는 °C, 힘은 N)

**예제**:
```
MODE 0 TEMP 60.0       # CH0을 온도제어 모드, 60°C 목표
MODE 1 FORCE 50.0      # CH1을 힘제어 모드, 50N 목표
MODE 2 DISABLED 0      # CH2 비활성
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 2. PWM - PWM 직접 제어 (오픈루프)

```
PWM <channel> <duty>
```

**파라미터**:
- `<channel>`: 채널 번호 (0-5)
- `<duty>`: PWM Duty (0.0-100.0%)

**예제**:
```
PWM 0 75.5             # CH0을 75.5% PWM
PWM 3 0                # CH3을 0% (정지)
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 3. PID - PID 게인 설정

```
PID <channel> <kp> <ki> <kd>
```

**파라미터**:
- `<channel>`: 채널 번호 (0-5)
- `<kp>`: 비례 게인
- `<ki>`: 적분 게인
- `<kd>`: 미분 게인

**예제**:
```
PID 0 5.0 0.1 0.5      # CH0 PID 게인 설정
PID 1 3.0 0.05 0.3     # CH1 PID 게인 설정 (힘 제어용)
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 4. STOP - 비상정지

```
STOP <channel|ALL>
```

**파라미터**:
- `<channel>`: 채널 번호 (0-5)
- `ALL`: 전체 채널

**예제**:
```
STOP 0                 # CH0 정지
STOP ALL               # 전체 채널 정지
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 5. FAN - 팬 제어

```
FAN <channel> <duty>
```

**파라미터**:
- `<channel>`: 팬 채널 (0-5)
- `<duty>`: 팬 PWM Duty (0.0-100.0%)

**예제**:
```
FAN 0 80               # 팬 CH0을 80%로
FAN 2 0                # 팬 CH2 정지
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 6. RESET - 채널 리셋

```
RESET <channel|ALL>
```

**파라미터**:
- `<channel>`: 채널 번호 (0-5)
- `ALL`: 전체 채널

**예제**:
```
RESET 0                # CH0 리셋
RESET ALL              # 전체 채널 리셋
```

**응답**: `OK\r\n` 또는 `ERROR: ...\r\n`

---

#### 7. STATUS - 상태 즉시 요청

```
STATUS
```

**예제**:
```
STATUS                 # 텔레메트리 프레임 즉시 전송
```

**응답**: 텔레메트리 프레임 96바이트 전송

---

## 사용 예제

### 시나리오 1: 온도 제어 시작

```
# 1. 채널 0을 온도 제어 모드로 설정, 목표 60°C
MODE 0 TEMP 60.0
> OK

# 2. PID 게인 조정 (선택 사항)
PID 0 5.0 0.1 0.5
> OK

# 3. 팬 가동 (냉각용)
FAN 0 50
> OK
```

### 시나리오 2: 힘 제어 시작

```
# 1. 채널 1을 힘 제어 모드로 설정, 목표 50N
MODE 1 FORCE 50.0
> OK

# 2. 힘 제어용 PID 게인 설정
PID 1 3.0 0.05 0.3
> OK
```

### 시나리오 3: 비상정지

```
# 전체 채널 즉시 정지
STOP ALL
> OK
```

### 시나리오 4: 상태 확인

```
# 현재 상태 요청
STATUS
> [96 bytes binary telemetry frame]
```

---

## 오류 처리

### 명령 파싱 실패

```
INVALID COMMAND
> ERROR: Invalid command
```

### 채널 범위 초과

```
MODE 10 TEMP 60
> ERROR: Execution failed
```

### 파라미터 오류

```
MODE 0 UNKNOWN 60
> ERROR: Invalid command
```

---

## 구현 세부사항

### 수신 버퍼

- 크기: 128바이트
- 명령 종료: `\r` 또는 `\n`
- 오버플로 시 자동 리셋

### 송신 타이밍

- 텔레메트리: TIM8 인터럽트 (50ms 주기)
- 응답: 명령 수신 즉시

### CRC-16 계산

```c
uint16_t calculate_crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}
```

---

## 파일 구조

```
Core/
├── Inc/
│   └── comm_protocol.h      # 통신 프로토콜 헤더
└── Src/
    └── comm_protocol.c      # 통신 프로토콜 구현
```

---

## 주의사항

1. **텔레메트리 프레임**: 바이너리 데이터이므로 텍스트 모드로 읽으면 안 됨
2. **명령 종료**: 반드시 `\r` 또는 `\n`으로 종료
3. **타이밍**: 텔레메트리는 50ms 주기로 자동 전송되므로 UART 버퍼 오버플로 주의
4. **CRC 검증**: 텔레메트리 데이터 신뢰성을 위해 CRC 검증 권장

---

**작성일**: 2025-10-24
**버전**: 1.0
**작성자**: Claude Code
