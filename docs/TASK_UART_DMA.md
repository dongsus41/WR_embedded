# UART DMA 전환 PRD

## 개요
- **작업 ID**: TASK_UART_DMA
- **시작일**: 2026-01-22
- **상태**: [ ] 진행중 / [x] 완료 / [ ] 보류

## 목표
TelemetryTask의 Blocking UART 전송을 DMA 방식으로 전환하여 CPU 점유율을 **94% → 12%**로 감소시킨다.

**예상 효과**: CPU 여유 시간 0% → **53%** 확보

## 관련 파일
- `Core/Src/comm_protocol.c` - DMA 전송 함수 추가
- `Core/Inc/comm_protocol.h` - 새 함수 프로토타입
- `Core/Src/freertos.c` - 전송 완료 세마포어 추가
- `Core/Src/main.c` - HAL_UART_TxCpltCallback 추가
- `260110_softrobot_main_v3.3.ioc` - CubeMX DMA 설정

---

## 체크리스트

### 1단계: CubeMX 설정
- [x] USART3 TX DMA 채널 추가 (DMA1 Stream 사용)
- [x] DMA 인터럽트 활성화
- [x] USART3 글로벌 인터럽트 확인
- [x] 코드 재생성

### 2단계: freertos.c 수정
- [x] `uartTxCompleteSemaphoreHandle` 세마포어 선언
- [x] MX_FREERTOS_Init()에서 세마포어 생성 (초기값 1)
- [x] 세마포어 extern 선언 (comm_protocol.c에서 사용)

### 3단계: comm_protocol.c 수정
- [x] DMA 전송 버퍼 (더블 버퍼링) 추가
- [x] `Comm_TransmitFrame_DMA()` 함수 추가
- [x] `Comm_SendTelemetry()` 수정 - DMA 사용
- [x] 전송 중 플래그 추가 (`tx_dma_in_progress`)
- [x] `Comm_UART_TxCpltCallback()` 함수 추가

### 4단계: main.c 콜백 추가
- [x] `HAL_UART_TxCpltCallback()` 구현
- [x] `Comm_UART_TxCpltCallback()` 호출

### 5단계: 테스트
- [x] 빌드 확인 (STM32CubeIDE)
- [x] 텔레메트리 데이터 정상 수신 확인
- [x] CPU 사용률 감소 확인: **11.3ms → 1.2ms (89% 감소)**
- [ ] 24시간 연속 동작 테스트

### 6단계: 완료
- [x] 코드 리뷰
- [x] 문서 업데이트
- [ ] 커밋

---

## 상세 구현 내용

### CubeMX 설정

```
USART3 설정:
- Mode: Asynchronous
- DMA Settings:
  - DMA Request: USART3_TX
  - Channel: DMA1 (자동 할당)
  - Direction: Memory to Peripheral
  - Priority: Medium
  - Mode: Normal (not Circular)
  - Data Width: Byte
- NVIC Settings:
  - DMA interrupt: Enabled
  - USART3 global interrupt: Enabled
```

### freertos.c 추가

```c
/* 세마포어 선언 */
osSemaphoreId_t uartTxCompleteSemaphoreHandle;
const osSemaphoreAttr_t uartTxCompleteSemaphore_attributes = {
  .name = "uartTxCompleteSem"
};

/* MX_FREERTOS_Init()에서 생성 */
// 초기값 1 = 첫 전송 시 대기 없이 시작 가능
uartTxCompleteSemaphoreHandle = osSemaphoreNew(1, 1, &uartTxCompleteSemaphore_attributes);
```

### comm_protocol.c 수정

```c
// 더블 버퍼링 (전송 중 버퍼 보호)
static uint8_t tx_buffer[2][TELEMETRY_FRAME_SIZE];
static uint8_t tx_buffer_index = 0;
static volatile uint8_t tx_in_progress = 0;

extern osSemaphoreId_t uartTxCompleteSemaphoreHandle;

static int32_t Comm_TransmitFrame_DMA(const uint8_t *data, uint16_t length)
{
    // 이전 전송 완료 대기 (타임아웃 15ms)
    if (osSemaphoreAcquire(uartTxCompleteSemaphoreHandle, 15) != osOK) {
        return -1;  // 타임아웃
    }

    // 더블 버퍼에 복사
    uint8_t *buf = tx_buffer[tx_buffer_index];
    memcpy(buf, data, length);
    tx_buffer_index = (tx_buffer_index + 1) % 2;

    tx_in_progress = 1;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(comm_huart, buf, length);

    if (status != HAL_OK) {
        tx_in_progress = 0;
        osSemaphoreRelease(uartTxCompleteSemaphoreHandle);
        return -1;
    }

    return 0;
}
```

### main.c 콜백

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        extern osSemaphoreId_t uartTxCompleteSemaphoreHandle;
        osSemaphoreRelease(uartTxCompleteSemaphoreHandle);
    }
}
```

---

## 아키텍처 변경

### Before (Blocking)
```
TelemetryTask (12ms 주기)
    │
    ├── Comm_BuildTelemetryFrame()  [0.5ms]
    ├── Comm_CalculateCRC16()       [0.2ms]
    └── HAL_UART_Transmit()         [11.3ms, BLOCKING!]
                                     ▲
                                     └── CPU 94% 점유
```

### After (DMA)
```
TelemetryTask (12ms 주기)
    │
    ├── Comm_BuildTelemetryFrame()  [0.5ms]
    ├── Comm_CalculateCRC16()       [0.2ms]
    └── HAL_UART_Transmit_DMA()     [0.1ms, 즉시 반환]
                                     │
                                     ▼
                           DMA 하드웨어가 전송 (11.3ms)
                                     │
                                     ▼
                           HAL_UART_TxCpltCallback() [ISR]
                                     │
                                     └── 세마포어 릴리즈
```

---

## 예상 결과

| 항목 | Before | After |
|------|--------|-------|
| TelemetryTask CPU | 94% | 12% |
| Idle CPU | 0% | 53% |
| 다른 Task 실행 기회 | 0.7ms/12ms | 10.5ms/12ms |
| 우선순위 역전 위험 | High | Low |

---

## 위험 요소 및 대응

| 위험 | 대응 |
|------|------|
| DMA 채널 충돌 | CubeMX에서 사용 중인 DMA 채널 확인 |
| 버퍼 재사용 (데이터 손상) | 더블 버퍼링으로 해결 |
| 전송 완료 전 다음 전송 시도 | 세마포어로 동기화 |
| DMA 에러 | HAL_UART_ErrorCallback()에서 복구 |

---

## 진행 기록

### 2026-01-22
- 작업 문서 생성
- CubeMX에서 USART3 TX DMA 설정 완료
- **freertos.c**: `uartTxCompleteSemaphoreHandle` 세마포어 추가
- **comm_protocol.c**:
  - 더블 버퍼링 (`tx_dma_buffer[2]`) 추가
  - `Comm_TransmitFrame_DMA()` 함수 추가
  - `Comm_UART_TxCpltCallback()` 함수 추가
  - `Comm_SendTelemetry()` DMA 사용으로 변경
- **main.c**: `HAL_UART_TxCpltCallback()` 추가
- 코드 구현 완료, STM32CubeIDE에서 빌드 필요
- CubeMX에서 세마포어 설정 (adcDataReadySem, uartTxCompleteSem)
- 변수명 통일 (CubeMX 생성 이름 사용)
- **테스트 완료**: TelemetryTask 실행 시간 11.3ms → 1.2ms (89% 감소)

---

## 참고 사항
- Phase 1 완료 후 Phase 2 (타이밍 충돌 해결) 진행
- `docs/RTOS_Issues_and_Improvements.md` 참조
