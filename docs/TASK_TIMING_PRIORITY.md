# Phase 2: 타이밍 및 우선순위 최적화 PRD

## 개요
- **작업 ID**: TASK_TIMING_PRIORITY
- **시작일**: 2026-01-22
- **상태**: [ ] 진행중 / [x] 완료 / [ ] 보류

## 목표
1. ControlTask와 TelemetryTask 간 타이밍 충돌 해결
2. 태스크 우선순위 재조정으로 우선순위 역전 위험 감소

## 관련 파일
- `Core/Src/freertos.c` - 주기 및 우선순위 수정
- `260110_softrobot_main_v3.3.ioc` - CubeMX 우선순위 설정

---

## 체크리스트

### 1단계: 타이밍 충돌 분석
- [x] 현재 주기 확인 (Control: 10ms, Telemetry: 12ms)
- [x] 충돌 시점 분석 (LCM = 60ms)
- [x] DMA 전환 후 새 주기 결정

### 2단계: TelemetryTask 주기 변경
- [x] 12ms → **20ms**로 변경 (50Hz 텔레메트리)
- [x] 대역폭 확인: 50Hz × 130B = 6.5KB/s (56% 사용, 여유 충분)

### 3단계: 우선순위 재조정
- [x] TelemetryTask: High(40) → **Normal3(27)**
- [x] CubeMX에서 우선순위 변경
- [x] 코드 재생성

### 4단계: 테스트
- [x] 빌드 확인
- [x] ControlTask 10ms 주기 안정성 확인 (LED1 토글)
- [x] 텔레메트리 20ms 주기 정상 수신 확인
- [x] TelemetryTask 실행 시간: 1.2ms (LED2)

### 5단계: 완료
- [x] 문서 업데이트
- [ ] 커밋

---

## 현재 상태 분석

### 타이밍 충돌 패턴
```
ControlTask 주기: 10ms
TelemetryTask 주기: 12ms
최소공배수(LCM): 60ms

충돌 시점: 0ms, 60ms, 120ms, ...
```

### DMA 전환 후 상황
```
Before DMA:
- TelemetryTask 실행: 11.3ms (blocking)
- 충돌 시 Control 지연: 최대 4.3ms

After DMA:
- TelemetryTask 실행: 1.2ms
- 충돌 시 Control 지연: 최대 1.2ms (허용 가능)
```

### 주기 변경 옵션

| 옵션 | 주기 | 대역폭 | 장점 | 단점 |
|------|------|--------|------|------|
| A | 10ms | 13KB/s | 충돌 없음, 100Hz | 대역폭 한계 근접 |
| B | 20ms | 6.5KB/s | 충돌 없음, 안전 | 50Hz로 감소 |
| C | 12ms 유지 | 10.8KB/s | 변경 없음 | 60ms마다 충돌 |

**권장**: 옵션 A (10ms) - DMA로 CPU 여유 충분, 100Hz 유지

---

## 우선순위 재조정

### 현재 우선순위
```
ControlTask:   Realtime (56)  - 최고
TelemetryTask: High (40)      - 높음
CommandTask:   Normal (24)    - 보통
NRF70_Task:    Normal1 (25)   - 보통+1
```

### 제안 우선순위
```
ControlTask:   Realtime (56)     - 유지 (제어 최우선)
TelemetryTask: AboveNormal (32)  - 낮춤 (DMA로 빠름)
SensorProcess: High (40)         - 유지 (센서 처리)
CommandTask:   Normal (24)       - 유지
NRF70_Task:    Normal1 (25)      - 유지
```

**효과**: Control-Telemetry 우선순위 격차 확대 (16 → 24단계)

---

## 진행 기록

### 2026-01-22
- 작업 문서 생성
- 대역폭 분석: 10ms 주기는 초과, 12ms/20ms 가능
- **TelemetryTask 주기**: 12ms → 20ms (50Hz)
- **TelemetryTask 우선순위**: High(40) → Normal3(27)
- 코드 수정 완료, 빌드 필요
- **테스트 완료**: 모든 항목 정상 동작 확인

---

## 참고 사항
- Phase 1 (DMA 전환) 완료로 TelemetryTask 실행 시간 1.2ms
- `docs/RTOS_Issues_and_Improvements.md` 참조
