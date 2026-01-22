# ISR 경량화 구현 PRD

## 개요
- **작업 ID**: TASK_ISR_LIGHTWEIGHT
- **시작일**: 2026-01-22
- **상태**: [ ] 진행중 / [x] 완료 / [ ] 보류

## 목표
ISR(ADC 완료 콜백)에서 float 연산과 snprintf() 호출을 제거하여 인터럽트 응답성을 개선하고 시스템 안정성을 확보한다.

**예상 효과**: ISR 실행 시간 100μs+ → **5μs** (20배 개선)

## 관련 파일
- `Core/Src/sensors.c` - ISR용 함수 분리, float 연산 Task로 이전
- `Core/Inc/sensors.h` - 새 함수 프로토타입 추가
- `Core/Src/freertos.c` - SensorProcessTask 추가, 세마포어 생성

---

## 체크리스트

### 1단계: 분석 및 설계
- [x] 현재 ISR 코드 분석 (Sensor_UpdateADC)
- [x] float 연산 및 snprintf 호출 위치 파악
- [x] 아키텍처 변경 설계 완료

### 2단계: 구현 - sensors.h
- [x] `Sensor_UpdateADC_ISR()` 프로토타입 추가 (이미 완료됨)
- [x] `Sensor_ProcessADC()` 프로토타입 추가 (이미 완료됨)

### 3단계: 구현 - sensors.c
- [x] `Sensor_UpdateADC_ISR()` 함수 추가 (Raw 값만 복사)
- [x] `Sensor_ProcessADC()` 함수 추가 (float 연산)
- [x] `check_temp_sensor_fault()` 수정 (snprintf 제거, 카운터만 증가)
- [x] `apply_outlier_filter()` 수정 (snprintf 제거, 카운터만 증가)

### 4단계: 구현 - freertos.c
- [x] `adcDataReadySemaphoreHandle` 세마포어 추가
- [x] `SensorProcessTask` 태스크 추가 (High 우선순위)
- [x] `RTOS_ADC_ConvCpltCallback()` 수정 (ISR 함수 호출 + 세마포어 릴리즈)
- [x] `StartSensorProcessTask()` 함수 구현

### 5단계: 테스트
- [x] 빌드 확인 (STM32CubeIDE에서 수동 빌드 필요)
- [x] 온도 센서 데이터 정상 출력 확인
- [x] 12ms 주기 텔레메트리 출력 확인

### 6단계: 완료
- [x] 코드 리뷰
- [x] 문서 업데이트
- [ ] 커밋

---

## 아키텍처 변경

### Before (현재)
```
ADC DMA Complete ISR
    └── Sensor_UpdateADC()  [100μs+, float 연산, snprintf()]
            └── ControlTask가 결과 사용
```

### After (수정 후)
```
ADC DMA Complete ISR
    ├── Sensor_UpdateADC_ISR()  [5μs, raw 복사만]
    └── osSemaphoreRelease()
            │
            ▼
SensorProcessTask (High 우선순위)
    └── Sensor_ProcessADC()  [100μs, float 연산]
            └── ControlTask가 결과 사용
```

---

## 상세 구현 내용

### sensors.h 추가 내용
```c
/**
 * @brief ADC 데이터 업데이트 (ISR용 - Raw 값만)
 * @note ISR에서 호출, float 연산 없음
 */
void Sensor_UpdateADC_ISR(volatile uint16_t *adc_buffer);

/**
 * @brief ADC 데이터 처리 (Task용 - float 연산)
 * @note Task context에서 호출
 */
void Sensor_ProcessADC(void);
```

### freertos.c 추가 내용
```c
// 세마포어
osSemaphoreId_t adcDataReadySemaphoreHandle;

// Task 속성
const osThreadAttr_t SensorProcessTask_attributes = {
  .name = "SensorProcess",
  .stack_size = 512 * 4,  // 2048 bytes
  .priority = (osPriority_t) osPriorityHigh,
};

// ISR 콜백 수정
void RTOS_ADC_ConvCpltCallback(void)
{
    extern volatile uint16_t buf_adc1[];
    Sensor_UpdateADC_ISR(buf_adc1);
    osSemaphoreRelease(adcDataReadySemaphoreHandle);
}
```

---

## 위험 요소 및 대응

| 위험 | 대응 |
|------|------|
| 센서 데이터 지연 증가 | SensorProcessTask 우선순위를 High로 설정하여 최소화 |
| 세마포어 오버플로우 | 바이너리 세마포어 사용 (카운트 1) |
| ControlTask가 처리 전 데이터 사용 | 기존 동작과 동일 (ADC 주기 > Control 주기) |

---

## 예상 결과

| 항목 | Before | After |
|------|--------|-------|
| ISR 실행 시간 | 100μs+ | 5μs |
| ISR 내 float 연산 | 있음 | 없음 |
| ISR 내 snprintf() | 있음 | 없음 |
| 시스템 안정성 | 크래시 위험 | 안전 |

---

## 진행 기록

### 2026-01-22
- 작업 문서 생성
- 분석 및 설계 완료
- 구현 시작
- **sensors.c**: `Sensor_UpdateADC_ISR()`, `Sensor_ProcessADC()` 함수 추가
- **sensors.c**: `check_temp_sensor_fault()`, `apply_outlier_filter()`에서 snprintf 제거
- **freertos.c**: `adcDataReadySemaphoreHandle` 세마포어 추가
- **freertos.c**: `SensorProcessTask` (High 우선순위) 추가
- **freertos.c**: `RTOS_ADC_ConvCpltCallback()` 수정 - ISR 경량화
- 코드 구현 완료, STM32CubeIDE에서 빌드 필요
- **테스트 완료**: 온도 센서 데이터 정상 출력, 12ms 주기 확인

---

## 참고 사항
- `docs/RTOS_Issues_and_Improvements.md` - RTOS 이슈 분석 문서
- Phase 1은 ISR 경량화에 집중, Phase 2에서 DMA 전송 최적화 예정
