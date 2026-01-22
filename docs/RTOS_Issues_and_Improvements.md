# RTOS 아키텍처 분석 및 개선 방안

## 문서 목적

본 문서는 현재 WR_embedded 프로젝트의 FreeRTOS 아키텍처를 면밀히 분석하여 잠재적 문제점을 식별하고, 각 문제에 대한 구체적인 개선 방안을 제시합니다. 이는 향후 시스템 안정성과 성능을 향상시키기 위한 로드맵으로 활용됩니다.

---

## 목차

1. [현재 아키텍처 요약](#현재-아키텍처-요약)
2. [식별된 문제점](#식별된-문제점)
3. [개선 방안](#개선-방안)
4. [우선순위별 구현 로드맵](#우선순위별-구현-로드맵)
5. [예상 효과](#예상-효과)

---

## 현재 아키텍처 요약

### 태스크 구성

| 태스크 | 우선순위 | 스택 크기 | 주기 | 실행 시간 | CPU 사용률 |
|--------|---------|----------|------|----------|-----------|
| ControlTask | Realtime (56) | 4096 bytes | 10ms | ~3ms | 30% |
| TelemetryTask | High (40) | 2048 bytes | 12ms | **~11.3ms** ⚠️ | **94%** ⚠️ |
| CommandTask | Normal (24) | 2048 bytes | Event | ~0.5ms | <5% |
| NRF70_Task | Normal1 (25) | 1024 bytes | 1ms | 0ms (비활성) | 0% |

### 동기화 객체

| 뮤텍스 | 보호 대상 | 사용 태스크 | 타임아웃 |
|--------|---------|----------|---------|
| sensorDataMutex | sensor_data | ControlTask, TelemetryTask | **5ms** |
| smaChannelMutex | sma_channels | ControlTask, CommandTask | **5ms** |
| uartTxMutex | UART TX | TelemetryTask, CommandTask | **50ms** ⚠️ |
| i2cMutex | I2C 버스 | ControlTask | **5ms** |

### 통신 방식

- **UART TX**: `HAL_UART_Transmit()` - **Blocking 방식** ⚠️
- **UART RX**: `HAL_UART_Receive_IT()` - Interrupt 방식 ✅
- **CAN RX**: FDCAN Interrupt - ISR 직접 처리 ✅
- **ADC**: DMA - ISR 직접 처리 ✅

---

## 식별된 문제점

### 🚨 1. UART Blocking으로 인한 CPU 독점 (Critical)

**문제 상황**:
```c
// Core/Src/comm_protocol.c:437
HAL_UART_Transmit(comm_huart, data, 130, 100);  // Blocking!
```

**영향**:
- TelemetryTask가 **11.3ms 동안 CPU를 점유**
- 12ms 주기 중 94% CPU 사용 (UART 대역폭과 동일)
- 낮은 우선순위 태스크(CommandTask, WiFi) 실행 기회 박탈
- RTOS 도입 효과 반감 (CPU 효율성 개선 없음)

**타이밍 분석**:
```
시간 (ms):  0         11.3      12        23.3      24
            │         │         │         │         │
Telemetry   ███████████████████─███████████████████─  (Blocking)
                     ▲                    ▲
                     │                    │
                  CPU 독점            CPU 독점

WiFi/Cmd    ─────────────────────█─────────────────█  (극히 짧은 시간만 실행)
```

**선점형 스케줄링과의 모순**:
- ControlTask는 선점 가능 (우선순위 > TelemetryTask) ✅
- 하지만 Telemetry 종료 후 남은 시간(0.7ms)에만 다른 작업 가능 ❌
- **RTOS의 멀티태스킹 이점을 활용하지 못함**

---

### ⚠️ 2. 우선순위 역전 (Priority Inversion) 위험 (High)

**문제 시나리오**:

```
1. TelemetryTask(High)가 smaChannelMutex 획득
2. TelemetryTask가 텔레메트리 전송 중 (11.3ms blocking)
3. ControlTask(Realtime) 주기 도래 (10ms)
4. ControlTask가 smaChannelMutex 요청
   → TelemetryTask가 mutex 반환할 때까지 대기 (최대 11.3ms!)
5. ControlTask의 10ms 데드라인 위반 가능
```

**타이밍 다이어그램**:
```
시간 (ms):  0    5    10   15   20
            │    │    │    │    │
Control     ███──────────[대기]──███  ← 10ms 주기 깨짐!
                         ▲
                         │ smaChannelMutex 대기
Telemetry   ─────███████████████─────  (mutex 점유 중)
                 └ smaChannelMutex 획득
```

**현재 완화 장치**:
- 뮤텍스 타임아웃 5ms 설정
- 하지만 TelemetryTask가 11.3ms 동안 mutex를 점유할 수 있음

**FreeRTOS 우선순위 상속 (Priority Inheritance)**:
- FreeRTOS 뮤텍스는 우선순위 상속 지원
- TelemetryTask가 mutex를 잡으면 일시적으로 Realtime 우선순위로 승격
- **하지만 Blocking UART로 인해 효과 제한적**

---

### ⚠️ 3. 뮤텍스 타임아웃 설정 불일치 (Medium)

**문제**:
```c
// ControlTask, TelemetryTask
osMutexAcquire(sensorDataMutex, 5);   // 5ms
osMutexAcquire(smaChannelMutex, 5);   // 5ms
osMutexAcquire(i2cMutex, 5);          // 5ms

// TelemetryTask, CommandTask
osMutexAcquire(uartTxMutex, 50);      // 50ms ⚠️
```

**분석**:
1. **5ms 타임아웃**:
   - ControlTask의 10ms 주기를 고려한 설정
   - 적절한 값

2. **50ms 타임아웃** (uartTxMutex):
   - UART 전송 시간(11.3ms) × 4배 이상
   - **과도하게 긴 타임아웃**
   - TelemetryTask가 대기 중일 때 최대 50ms 블로킹 가능

**권장 값**:
- uartTxMutex: **15ms** (전송 시간 11.3ms + 여유 30%)

---

### ⚠️ 4. 타이밍 충돌 가능성 (Medium)

**충돌 패턴**:
```
ControlTask 주기: 10ms
TelemetryTask 주기: 12ms
최소공배수(LCM): 60ms

충돌 시점: 0ms, 60ms, 120ms, 180ms, ...
```

**60ms 시점 분석**:
```
시간 (ms):  0    10   20   30   40   50   60   70   80
            │    │    │    │    │    │    │    │    │
Control     ●────●────●────●────●────●────●────●────  (10ms 주기)
                                         ▲
                                         │ 충돌!
Telemetry   ●────────●────────●────────●────────●──  (12ms 주기)
```

**충돌 시 동작**:
1. 두 태스크 동시에 Ready 상태
2. ControlTask(Realtime) 먼저 실행 (3ms)
3. TelemetryTask(High) 실행 (11.3ms)
4. **총 14.3ms → 다음 Control 주기(10ms)까지 4.3ms 지연**

**영향**:
- 60ms마다 ControlTask 지터 발생 (최대 4.3ms)
- 제어 성능 저하 가능성 (PID 제어)

**완화 방법**:
- TelemetryTask 주기를 **10ms의 배수**로 조정 (10ms, 20ms)
- 또는 **DMA로 전환**하여 Telemetry 실행 시간 단축

---

### ⚠️ 5. 스택 오버플로우 위험 (Medium)

**현재 스택 크기**:
```c
ControlTask:   4096 bytes (4KB)
TelemetryTask: 2048 bytes (2KB)
CommandTask:   2048 bytes (2KB)
NRF70_Task:    1024 bytes (1KB)
```

**스택 사용 분석**:

1. **ControlTask (4KB)**:
   ```c
   SMA_Update() 호출 스택:
   - 로컬 변수: ~100 bytes
   - SMA_GetChannelState(): ~200 bytes (구조체 복사)
   - PID 계산: ~100 bytes
   - 뮤텍스 대기 스택: ~200 bytes
   총 추정: ~600 bytes ✅ 안전
   ```

2. **TelemetryTask (2KB)**:
   ```c
   Comm_SendTelemetry_Safe() 호출 스택:
   - TelemetryFrame_t frame (130 bytes)
   - CRC 계산 버퍼: ~200 bytes
   - HAL_UART_Transmit 스택: ~300 bytes
   총 추정: ~630 bytes ✅ 안전
   ```

3. **CommandTask (2KB)**:
   ```c
   Comm_ParseCommand() 호출 스택:
   - cmd_rx_buffer[128]
   - ParsedCommand_t (32 bytes)
   - sscanf 스택: ~500 bytes
   총 추정: ~660 bytes ✅ 안전
   ```

**검증 방법**:
```c
// FreeRTOSConfig.h
#define configCHECK_FOR_STACK_OVERFLOW  2  // ✅ 이미 활성화

// 런타임 검사
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // 현재 무한 루프로 구현됨
    // LED 또는 UART 로그 추가 권장
}
```

**권장 사항**:
- 현재 스택 크기는 **안전한 수준**
- 향후 기능 추가 시 재검토 필요

---

### ⚠️ 6. ISR 작업량 (Medium)

**현재 ISR 작업**:

1. **UART RX ISR**:
   ```c
   void RTOS_UART_RxCallback(uint8_t byte) {
       osMessageQueuePut(uartRxQueue, &byte, 0, 0);  // ~5μs
   }
   ```
   ✅ 매우 가벼움 (논블로킹)

2. **FDCAN RX ISR**:
   ```c
   void RTOS_FDCAN_RxCallback(...) {
       Sensor_UpdateCAN(...);  // ⚠️ ~50μs
       // - ID 파싱
       // - 16비트 변환 (u16le)
       // - 구조체 업데이트
       // - 타임스탬프 갱신
   }
   ```
   ⚠️ 중간 정도 작업량

3. **ADC Complete ISR**:
   ```c
   void RTOS_ADC_ConvCpltCallback(void) {
       Sensor_UpdateADC(buf_adc1);  // ⚠️ ~100μs
       // - 6채널 ADC 읽기
       // - 온도 변환 (float 연산)
       // - 이상치 필터 (float 비교)
       // - 이동평균 필터 (4샘플)
       // - 오류 검사
   }
   ```
   ⚠️ 가장 무거운 ISR

**문제점**:
- ISR에서 **부동소수점 연산** (온도 변환, 필터)
- ISR에서 **메모리 복사** (구조체 업데이트)
- ISR 지연 시간 증가 → 다른 인터럽트 응답성 저하

**영향**:
- 높은 우선순위 인터럽트(CAN, ADC)가 UART 인터럽트를 지연
- ControlTask ISR 응답성 저하 가능

---

### ⚠️ 7. 메모리 단편화 위험 (Low)

**동적 메모리 할당**:
```c
// FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE  (65536)  // 64KB

// 할당 내역
CommandTask 스택:   2048 bytes
ControlTask 스택:   4096 bytes
TelemetryTask 스택: 2048 bytes
NRF70_Task 스택:    1024 bytes
uartRxQueue:        512 bytes
cmdQueue:           2048 bytes (16 × 128)
뮤텍스 4개:         ~200 bytes
-------------------------------------
총 사용:            ~12KB / 64KB (19%)
```

**분석**:
- 현재 메모리 사용률 **19%** ✅ 안전
- FreeRTOS Heap4 사용 (병합 가능한 힙)
- 단편화 위험 **낮음**

**모니터링 방법**:
```c
size_t free_heap = xPortGetFreeHeapSize();
size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
```

---

### ⚠️ 8. CRC-16 계산 오버헤드 (Low)

**현재 구현**:
```c
// Core/Src/comm_protocol.c
uint16_t Comm_CalculateCRC16(const uint8_t *data, uint16_t length) {
    // 소프트웨어 CRC (128 bytes)
    // 예상 시간: ~200μs (비트 연산)
}
```

**TelemetryTask 실행 시간 분해**:
```
1. Comm_BuildTelemetryFrame():  ~500μs (구조체 복사)
2. Comm_CalculateCRC16():       ~200μs (CRC 계산)
3. HAL_UART_Transmit():      ~11300μs (Blocking!)
------------------------------------------------------
총:                          ~12000μs (12ms)
```

**개선 가능성**:
- STM32H7의 **하드웨어 CRC 유닛** 사용
- 계산 시간: 200μs → **10μs** (20배 개선)
- 하지만 전체 12ms 중 200μs는 **1.7%**에 불과
- **우선순위: 낮음**

---

## 개선 방안

### 🎯 개선안 1: UART DMA 전환 (최우선, Critical)

**목표**: TelemetryTask CPU 점유 시간을 11.3ms → **1.5ms**로 단축

#### 구현 방법

**Before (Blocking)**:
```c
// Core/Src/comm_protocol.c
static int32_t Comm_TransmitFrame(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(comm_huart, data, length, 100);
    return (status == HAL_OK) ? 0 : -1;
}
```

**After (Non-blocking DMA)**:
```c
// 1. DMA 전송 시작
static int32_t Comm_TransmitFrame(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(comm_huart, data, length);
    return (status == HAL_OK) ? 0 : -1;
}

// 2. 완료 콜백 (ISR)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        // 전송 완료 시그널 (세마포어)
        osSemaphoreRelease(uartTxCompleteSemaphoreHandle);
    }
}

// 3. TelemetryTask 수정
void StartTelemetryTask(void *argument)
{
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(12));

        // 프레임 준비 (~1.5ms)
        Comm_BuildTelemetryFrame(&frame);
        Comm_CalculateCRC16(...);

        // DMA 전송 시작 (즉시 반환)
        Comm_TransmitFrame_DMA(&frame);

        // 다음 주기까지 대기하거나 다른 작업 가능
        // (CPU는 이제 WiFi/Command 태스크에 양보)
    }
}
```

#### 하드웨어 설정 (STM32CubeMX)

```
USART3 설정:
- Mode: Asynchronous
- DMA Settings:
  - DMA Request: USART3_TX
  - Channel: DMA1 Stream 3
  - Direction: Memory to Peripheral
  - Priority: Medium
  - Mode: Normal (not Circular)
- NVIC Settings:
  - DMA interrupt enabled
  - USART3 global interrupt enabled
```

#### 예상 효과

**CPU 사용률 변화**:
```
Before: TelemetryTask = 94% (11.3ms / 12ms)
After:  TelemetryTask = 12% (1.5ms / 12ms)
-----------------------------------------------
CPU 절감: 82% ✅
```

**타이밍 개선**:
```
시간 (ms):  0    1.5  3    5    7    9    11   13   15
            │    │    │    │    │    │    │    │    │
Control     ███──────███──────███──────███──────███─
Telemetry   ─█───────────█───────────────█──────────  (DMA 사용)
            └1.5ms      └1.5ms          └1.5ms

WiFi/Cmd    ──███████████████████████████████████───  (충분한 실행 시간!)
```

**위험 요소**:
- DMA 채널 충돌 (다른 DMA 사용 확인 필요)
- 전송 완료 전 버퍼 재사용 방지 (이중 버퍼링)

#### 구현 코드 (상세)

```c
// freertos.c에 세마포어 추가
osSemaphoreId_t uartTxCompleteSemaphoreHandle;

void MX_FREERTOS_Init(void) {
    // 바이너리 세마포어 생성
    uartTxCompleteSemaphoreHandle = osSemaphoreNew(1, 0, NULL);
}

// comm_protocol.c 수정
static volatile uint8_t tx_in_progress = 0;

static int32_t Comm_TransmitFrame_DMA(const uint8_t *data, uint16_t length)
{
    // 이전 전송 완료 대기 (타임아웃 15ms)
    if (osSemaphoreAcquire(uartTxCompleteSemaphoreHandle, 15) != osOK) {
        return -1;  // 타임아웃
    }

    tx_in_progress = 1;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(comm_huart, (uint8_t*)data, length);

    if (status != HAL_OK) {
        tx_in_progress = 0;
        osSemaphoreRelease(uartTxCompleteSemaphoreHandle);
        return -1;
    }

    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3 && tx_in_progress) {
        tx_in_progress = 0;
        osSemaphoreRelease(uartTxCompleteSemaphoreHandle);
    }
}
```

---

### 🎯 개선안 2: 우선순위 역전 방지 (High)

**방법 1: 우선순위 재조정**

```
현재:
ControlTask:   Realtime (56)
TelemetryTask: High (40)      ← 차이 16단계
CommandTask:   Normal (24)

개선:
ControlTask:   Realtime (56)
TelemetryTask: Normal3 (27)   ← 차이 29단계 (더 벌림)
CommandTask:   Normal (24)
```

**효과**:
- ControlTask와 TelemetryTask 우선순위 격차 확대
- 우선순위 상속 발생 시 영향 감소

**단점**:
- DMA 미사용 시 여전히 근본 문제 해결 안됨

---

**방법 2: 뮤텍스 → 세마포어 변경 (smaChannelMutex)**

```c
// 현재 (뮤텍스)
osMutexAcquire(smaChannelMutex, 5);
// 작업 수행
osMutexRelease(smaChannelMutex);

// 개선 (바이너리 세마포어)
osSemaphoreAcquire(smaChannelSemaphore, 5);
// 작업 수행
osSemaphoreRelease(smaChannelSemaphore);
```

**뮤텍스 vs 세마포어**:
| 특성 | 뮤텍스 | 세마포어 |
|------|--------|----------|
| **우선순위 상속** | ✅ 지원 | ❌ 미지원 |
| **재귀 잠금** | ✅ 가능 | ❌ 불가능 |
| **소유권** | ✅ 태스크 소유 | ❌ 소유권 없음 |
| **용도** | 상호 배제 | 이벤트 신호 |

**권장**:
- **뮤텍스 유지** (우선순위 상속이 도움됨)
- DMA 전환으로 근본 문제 해결

---

### 🎯 개선안 3: ISR 작업량 최적화 (Medium)

**문제**: `Sensor_UpdateADC()`가 ISR에서 부동소수점 연산 수행

**개선 방법**: ISR → 태스크로 작업 이동

#### Before (ISR에서 직접 처리)
```c
void RTOS_ADC_ConvCpltCallback(void) {
    Sensor_UpdateADC(buf_adc1);  // 100μs (float 연산)
}
```

#### After (태스크로 이동)
```c
// 1. ISR: 세마포어만 전송
void RTOS_ADC_ConvCpltCallback(void) {
    osSemaphoreRelease(adcDataReadySemaphore);  // 5μs
}

// 2. 새 태스크: SensorProcessTask (우선순위: High)
void StartSensorProcessTask(void *argument)
{
    for(;;)
    {
        // ADC 완료 대기
        osSemaphoreAcquire(adcDataReadySemaphore, osWaitForever);

        // 태스크 컨텍스트에서 처리 (ISR 부담 감소)
        Sensor_UpdateADC(buf_adc1);
    }
}
```

**효과**:
- ISR 실행 시간: 100μs → **5μs** (20배 개선)
- 다른 인터럽트 응답성 향상
- 부동소수점 연산을 태스크 스케줄러가 관리

**단점**:
- 태스크 전환 오버헤드 (~10μs)
- 센서 데이터 지연 (~20μs)
- **우선순위: Medium** (현재도 동작은 정상)

---

### 🎯 개선안 4: 타이밍 충돌 회피 (Medium)

**방법 1: TelemetryTask 주기 변경**

```c
// 현재
#define TELEMETRY_TASK_PERIOD_MS    12  // 60ms마다 충돌

// 개선안 A: 10ms의 배수
#define TELEMETRY_TASK_PERIOD_MS    10  // 100Hz (충돌 없음)
// 단점: UART 대역폭 초과 (113%)

// 개선안 B: 20ms
#define TELEMETRY_TASK_PERIOD_MS    20  // 50Hz (충돌 없음)
// 장점: 대역폭 56% (안전), 충돌 없음
// 단점: 텔레메트리 주파수 감소 (80Hz → 50Hz)
```

**권장**: DMA 전환 후 10ms로 변경
- TelemetryTask 실행 시간: 11.3ms → 1.5ms
- 10ms 주기 사용 가능 (대역폭 15%)

---

**방법 2: 페이싱 (Phasing)**

```c
// ControlTask: 0ms, 10ms, 20ms, 30ms, ...
// TelemetryTask: 6ms, 18ms, 30ms, ... (6ms 오프셋)

void StartTelemetryTask(void *argument)
{
    // 초기 지연 추가
    osDelay(6);  // 6ms 오프셋

    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(12));
        // ...
    }
}
```

**효과**: 충돌 빈도 감소

---

### 🎯 개선안 5: 뮤텍스 타임아웃 표준화 (Low)

```c
// 표준 타임아웃 정의
#define MUTEX_TIMEOUT_SHORT   5   // 빠른 작업 (센서 읽기)
#define MUTEX_TIMEOUT_MEDIUM  15  // 중간 작업 (UART DMA)
#define MUTEX_TIMEOUT_LONG    50  // 긴 작업 (예약)

// 적용
osMutexAcquire(sensorDataMutex, MUTEX_TIMEOUT_SHORT);
osMutexAcquire(smaChannelMutex, MUTEX_TIMEOUT_SHORT);
osMutexAcquire(i2cMutex, MUTEX_TIMEOUT_SHORT);
osMutexAcquire(uartTxMutex, MUTEX_TIMEOUT_MEDIUM);  // 15ms로 단축
```

---

### 🎯 개선안 6: 하드웨어 CRC 사용 (Low)

**STM32H7 하드웨어 CRC 유닛**:
```c
// HAL 초기화
CRC_HandleTypeDef hcrc;
hcrc.Instance = CRC;
hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
hcrc.Init.GeneratingPolynomial = 0x1021;  // CRC-16-CCITT
HAL_CRC_Init(&hcrc);

// CRC 계산
uint16_t Comm_CalculateCRC16_HW(const uint8_t *data, uint16_t length)
{
    return (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t*)data, length/4);
}
```

**효과**: 200μs → 10μs (미미한 개선)

---

## 우선순위별 구현 로드맵

### Phase 1: Critical Issues (즉시 구현, 1-2주)

1. **UART DMA 전환** 🚨
   - 예상 작업: 3일
   - 난이도: 중
   - 효과: CPU 사용률 82% 감소
   - 파일 수정:
     - `Core/Src/comm_protocol.c`
     - `Core/Src/freertos.c` (세마포어 추가)
     - `Core/Inc/main.h` (콜백 추가)
     - STM32CubeMX (.ioc 파일)

2. **검증 및 테스트**
   - 예상 작업: 2일
   - 텔레메트리 무결성 확인
   - 타이밍 측정 (오실로스코프)
   - 스트레스 테스트

### Phase 2: High Priority (1개월 이내)

3. **타이밍 충돌 해결**
   - TelemetryTask 주기를 12ms → **10ms**로 변경
   - DMA 전환 후 가능

4. **우선순위 재조정**
   - TelemetryTask: High → Normal3
   - 검증 및 테스트

5. **뮤텍스 타임아웃 표준화**
   - 상수 정의 및 일괄 적용

### Phase 3: Medium Priority (3개월 이내)

6. **ISR 작업량 최적화**
   - SensorProcessTask 추가
   - ADC 처리를 ISR → Task로 이동

7. **모니터링 기능 추가**
   - 스택 사용량 로깅
   - CPU 사용률 측정
   - 뮤텍스 대기 시간 통계

### Phase 4: Low Priority (필요 시)

8. **하드웨어 CRC 전환**
9. **추가 최적화**

---

## 예상 효과

### CPU 사용률 개선 (Phase 1 완료 후)

| 태스크 | Before | After | 개선 |
|--------|--------|-------|------|
| ControlTask | 30% | 30% | - |
| TelemetryTask | **94%** | **12%** | **-82%** ✅ |
| CommandTask | <5% | <5% | - |
| Idle | ~0% | **~53%** | **+53%** ✅ |

**총 CPU 여유**: 0% → **53%** 🎉

### 시스템 응답성 개선

| 항목 | Before | After |
|------|--------|-------|
| WiFi/Command 실행 기회 | 0.7ms/12ms (6%) | 10.5ms/12ms (88%) |
| ControlTask 지터 | 0-4.3ms | 0-0.5ms |
| 우선순위 역전 위험 | High | Low |

### 확장 가능성

**새로운 기능 추가 가능**:
- WiFi 통신 (nRF7002) 활성화
- 추가 센서 통합 (HX711 로드셀)
- 데이터 로깅 (SD 카드)
- GUI 업데이트

---

## 검증 계획

### 1. 기능 검증

- [ ] 텔레메트리 데이터 무결성 (CRC 체크)
- [ ] 명령 응답 정상 동작
- [ ] 제어 루프 안정성 (10ms 주기 유지)
- [ ] CAN 센서 수신 정상

### 2. 성능 측정

**측정 항목**:
```c
// 추가할 디버그 코드
uint32_t control_start = HAL_GetTick();
SMA_Update();
uint32_t control_end = HAL_GetTick();
uint32_t control_duration = control_end - control_start;

// UART 로그
printf("Control: %lu ms, Telemetry: %lu ms\n",
       control_duration, telemetry_duration);
```

**오실로스코프 측정**:
- GPIO 토글로 태스크 실행 시간 측정
- 타이밍 지터 확인

### 3. 스트레스 테스트

- [ ] 24시간 연속 동작 테스트
- [ ] 고부하 상황 (모든 채널 활성)
- [ ] 온도 사이클 테스트
- [ ] 뮤텍스 경합 시뮬레이션

---

## 참고 자료

### FreeRTOS 문서
- [Priority Inversion](https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2006/freertos_Priority_Inversion_1474145.html)
- [Mutexes](https://www.freertos.org/RTOS-mutex-example.html)
- [DMA with FreeRTOS](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/DMA_Capable_IO.html)

### STM32 HAL
- [STM32H7 DMA Documentation](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) (Chapter 11)
- [HAL UART DMA Example](https://github.com/STMicroelectronics/STM32CubeH7/tree/master/Projects/NUCLEO-H743ZI/Examples/UART/UART_TwoBoards_ComDMA)

---

**문서 버전**: 1.0
**작성일**: 2026-01-22
**작성자**: Claude (AI Assistant)
**리뷰 필요**: Phase 1 구현 전 검토 권장
