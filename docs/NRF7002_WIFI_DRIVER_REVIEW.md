# nRF7002 Wi-Fi 드라이버 검토 보고서

**작성일**: 2026-01-16
**대상 모듈**: Nordic nRF7002 Wi-Fi 6 Companion IC (WM02C)
**플랫폼**: STM32H725RG + FreeRTOS
**목적**: Wi-Fi 모듈 Feasibility 확인 및 드라이버 통합 준비

---

## 📋 Executive Summary

### 결론
✅ **nRF7002 Wi-Fi 모듈 Feasibility 확인 완료**

STM32H725RG와 nRF7002(WM02C) 간 OCTOSPI 통신이 정상 동작하며, 펌웨어 임베딩 및 파싱이 성공적으로 완료되었습니다. 향후 nrf70-bm 드라이버 전체 통합을 통해 Wi-Fi 기능 구현이 가능합니다.

### 주요 성과
- ✅ OCTOSPI(QSPI 모드) 통신 검증 완료
- ✅ 전원 제어 (BUCKEN) 정상 동작 확인
- ✅ 펌웨어 바이너리 준비 (32.8KB, Scan Only 모드)
- ✅ 펌웨어 파싱 및 메타데이터 검증
- ✅ 개발 환경 구축 (nrf70-bm 드라이버 다운로드)

---

## 🔧 하드웨어 구성

### MCU 플랫폼
- **MCU**: STM32H725RG (ARM Cortex-M7, 550MHz)
- **RTOS**: FreeRTOS
- **SYSCLK**: 500MHz
- **Flash**: 1MB
- **RAM**: 564KB

### Wi-Fi 모듈
- **모델**: Nordic nRF7002 (WM02C)
- **프로토콜**: Wi-Fi 6 (802.11ax)
- **인터페이스**: QSPI
- **최대 클럭**: 32MHz (QSPI), 8MHz (SPI)

### 핀맵

| 신호 | STM32 핀 | nRF7002 | 기능 |
|------|----------|---------|------|
| QSPI_CLK | PB2 | SCLK | Clock |
| QSPI_NCS | PB10 | CSN | Chip Select |
| QSPI_IO0 | PB1 | MOSI/IO0 | Data 0 |
| QSPI_IO1 | PB0 | MISO/IO1 | Data 1 |
| QSPI_IO2 | PA7 | IO2 | Data 2 |
| QSPI_IO3 | PA6 | IO3 | Data 3 |
| WM_BUCK | PC5 | BUCKEN | 전원 제어 |
| WM_IRQ | PC4 | HOST_IRQ | 인터럽트 (Rising Edge) |

### OCTOSPI 설정
```c
hospi1.Init.ClockPrescaler = 63;        // ~1.6MHz (안정성 우선)
hospi1.Init.DeviceSize = 32;            // 2^32 bytes
hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
```

**참고**: 향후 성능 최적화 시 Prescaler를 6~12로 낮춰 16-33MHz로 동작 가능

---

## ✅ Feasibility 테스트 결과

### 1. 하드웨어 통신 테스트

**테스트 코드**: `Core/Src/nrf7002_test.c`

```
========== nRF7002 (WM02C) Test Result ==========
QSPI Init:    OK
RDSR0:        OK (0x42)
RDSR1:        OK (0x00)
RDSR2:        OK (0x00)
RPU Awake:    NO

>>> nRF7002 ALIVE! Hardware OK! <<<
```

**결과 분석**:
- ✅ OCTOSPI 초기화 성공
- ✅ Status Register 읽기/쓰기 정상 (0x42는 Ready 상태)
- ✅ nRF7002 칩 응답 확인
- ⚠️ RPU(Radio Processing Unit) 미활성 → 펌웨어 미로드 상태 (정상)

### 2. 펌웨어 준비 및 검증

**펌웨어 파일**: `Core/Inc/nrf7002_fw.h`

```
========== nRF7002 Firmware Information ==========
Signature:    0xDEAD1EAF (Valid)
Num Images:   4
Version:      1.2.13.21
Features:     0x00000004 (Scan Only Mode)
Total Size:   32812 bytes
Data Length:  32760 bytes
SHA256 Hash:  048D83205B6F85C9E122F1072284B16D...

Firmware Images:
  [0] UMAC_PRI  : Type=0, Size=388 bytes
  [1] UMAC_SEC  : Type=1, Size=18424 bytes
  [2] LMAC_PRI  : Type=2, Size=448 bytes
  [3] LMAC_SEC  : Type=3, Size=13468 bytes
```

**검증 항목**:
- ✅ Nordic 펌웨어 Signature 검증 (0xDEAD1EAF)
- ✅ 4개 이미지 모두 정상 파싱 (UMAC/LMAC Primary/Secondary)
- ✅ SHA256 해시 확인
- ✅ 총 32,728 bytes 데이터 (헤더 제외)

### 3. IRQ 핀 설정 확인

**GPIO 설정**: `Core/Src/gpio.c:90-94, 118-119`

```c
// PC4 = WM_IRQ (Rising Edge Interrupt)
GPIO_InitStruct.Pin = WM_IRQ_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI4_IRQn);
```

**인터럽트 핸들러**: `Core/Src/stm32h7xx_it.c:170-179`
```c
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(WM_IRQ_Pin);
}
```

**상태**:
- ✅ 하드웨어 설정 완료
- ⚠️ 콜백 함수 미구현 (향후 드라이버 통합 시 구현 필요)

---

## 📦 구현 현황

### 완료된 기능

#### 1. 하드웨어 추상화 계층
- **파일**: `Core/Src/nrf7002_test.c`, `Core/Inc/nrf7002_test.h`
- **기능**:
  - `NRF70_Test_Init()`: OCTOSPI 핸들 초기화
  - `NRF70_PowerOn/Off()`: 전원 제어 (WM_BUCK)
  - `NRF70_ReadSR0/1/2()`: Status Register 읽기
  - `NRF70_WriteSR2()`: Status Register 쓰기
  - `NRF70_RunFullTest()`: 통합 테스트 실행

#### 2. 펌웨어 관리
- **파일**: `Core/Inc/nrf7002_fw.h`
- **내용**: 32.8KB Scan Only 펌웨어 (C 배열)
- **소스**: `nrf70-bm/sdk-nrfxlib/nrf_wifi/bin/ncs/scan_only/nrf70.bin`

#### 3. 펌웨어 파싱 및 검증
- **함수**:
  - `NRF70_PrintFirmwareInfo()`: 펌웨어 메타데이터 출력
  - `NRF70_LoadFirmware()`: 펌웨어 로더 Stub

### 미구현 기능 (장기 계획)

#### 1. 펌웨어 로딩
- RPU 메모리 맵핑
- QSPI를 통한 펌웨어 전송
- 부팅 시퀀스 실행

#### 2. Wi-Fi 드라이버 통합
- nrf70-bm 전체 드라이버 포팅
- FMAC(Firmware MAC) 레이어
- Platform Abstraction Layer (PAL)

#### 3. 네트워크 스택
- Wi-Fi 스캔
- 연결 관리
- TCP/IP 스택 연동

---

## 📂 프로젝트 구조

```
WR_embedded/
├── Core/
│   ├── Inc/
│   │   ├── nrf7002_test.h          # nRF7002 테스트 드라이버 헤더
│   │   └── nrf7002_fw.h            # 펌웨어 바이너리 (32.8KB)
│   └── Src/
│       ├── nrf7002_test.c          # nRF7002 테스트 드라이버 구현
│       ├── main.c                  # 메인 로직 (테스트 실행)
│       ├── octospi.c               # OCTOSPI 초기화
│       └── gpio.c                  # GPIO 설정 (전원, IRQ)
│
├── nrf70-bm/                       # Nordic 공식 드라이버 (향후 통합)
│   ├── nrf70_bm_lib/              # 핵심 드라이버 라이브러리
│   ├── nrf_wifi/                  # Wi-Fi HAL 및 펌웨어 인터페이스
│   ├── sdk-nrfxlib/               # Nordic 라이브러리
│   │   └── nrf_wifi/bin/ncs/     # 펌웨어 바이너리 (5가지 모드)
│   ├── samples/                   # 샘플 코드
│   └── nrf70_zephyr_shim/         # Zephyr OS 참조 구현
│
└── docs/
    └── NRF7002_WIFI_DRIVER_REVIEW.md  # 본 문서
```

---

## 🔮 향후 계획

### Phase 1: 펌웨어 로딩 구현 (단기)
**목표**: RPU 부팅 및 Alive 상태 확인

**작업 항목**:
1. ✅ 펌웨어 바이너리 준비 (완료)
2. ⏳ QSPI 메모리 맵핑 구현
3. ⏳ `hal_fw_patch_chunk_load()` 포팅
4. ⏳ `hal_fw_patch_boot()` 포팅
5. ⏳ RPU Awake 상태 확인

**예상 결과**:
```
RDSR1:        OK (0x01)  # RPU Awake Bit 활성화
RPU Awake:    YES
```

### Phase 2: nrf70-bm 드라이버 통합 (중기)
**목표**: Wi-Fi 스캔 기능 구현

**작업 항목**:
1. Platform Abstraction Layer (PAL) 구현
   - `nrf_wifi_osal_*` 함수 (메모리, 스레드, 타이머 등)
2. HAL 레이어 포팅
   - `hal_api_common.c`, `hal_mem.c`, `hal_interrupt.c`
3. FMAC 레이어 통합
   - `fmac_api_common.c` 포팅
4. IRQ 핸들러 구현
   - `HAL_GPIO_EXTI_Callback()` 연결
   - 이벤트 큐 처리

**참고 구현**:
- `nrf70-bm/nrf70_zephyr_shim/` (Zephyr OS 예제)
- `nrf70-bm/samples/scan_bm/` (Scan 샘플)

### Phase 3: 네트워크 스택 연동 (장기)
**목표**: 실제 Wi-Fi 통신 기능

**작업 항목**:
1. Wi-Fi 연결 관리
   - SSID 스캔
   - WPA2/WPA3 인증
   - IP 주소 획득 (DHCP)
2. TCP/IP 스택 연동
   - LwIP 또는 FreeRTOS+TCP 통합
3. 애플리케이션 레벨 프로토콜
   - HTTP, MQTT, WebSocket 등

---

## 📊 성능 고려사항

### 1. OCTOSPI 클럭 최적화
**현재**: 1.6MHz (Prescaler = 63)
**최적**: 16-33MHz (Prescaler = 6-12)

**변경 방법** (`Core/Src/octospi.c:51`):
```c
hospi1.Init.ClockPrescaler = 6;  // D1HCLK(200MHz) / 6 ≈ 33MHz
```

**테스트 필요**: 안정성 확인 후 단계적으로 클럭 증가

### 2. 메모리 사용량

| 항목 | 크기 | 위치 |
|------|------|------|
| 펌웨어 (Scan Only) | 32.8 KB | Flash |
| 펌웨어 (Default) | 78 KB | Flash |
| 드라이버 코드 | ~50 KB | Flash (예상) |
| 런타임 버퍼 | ~20 KB | RAM (예상) |

**STM32H725RG 여유 공간**:
- Flash: 1MB - 현재 사용량 ≈ 700KB 이상 여유
- RAM: 564KB - 충분한 여유

### 3. FreeRTOS 통합
- Wi-Fi 스택 전용 태스크 필요 (스택 크기: 4-8KB 권장)
- IRQ → 이벤트 큐 → 태스크 처리 구조
- 우선순위: Medium~High (네트워크 지연 최소화)

---

## 🛠 개발 환경

### 소프트웨어
- **IDE**: STM32CubeIDE
- **SDK**: STM32Cube HAL Driver
- **빌드 시스템**: GNU Make / CMake
- **디버거**: ST-Link

### 외부 라이브러리
- **nrf70-bm**: Nordic nRF70 시리즈 Bare-Metal 드라이버
  - 버전: Latest (2026-01-16 기준)
  - 리포지토리: https://github.com/nrfconnect/nrf70-bm
  - 라이선스: BSD-3-Clause

### 펌웨어 버전
- **nRF7002 Firmware**: v1.2.13.21
- **모드**: Scan Only (Feature Flag 0x04)
- **소스**: sdk-nrfxlib/nrf_wifi/bin/ncs/scan_only/

---

## 📚 참고 자료

### Nordic Semiconductor 문서
1. **nRF7002 Product Specification**
   - SPI/QSPI 인터페이스 상세 사양
   - 전원 시퀀스 및 타이밍
   - https://www.nordicsemi.com/Products/nRF7002

2. **nrf70-bm 드라이버 문서**
   - Porting Guide: `nrf70-bm/nrf70_bm_lib/docs/`
   - API Reference: Doxygen 생성 필요

3. **nRF Connect SDK 문서**
   - https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/index.html

### STMicroelectronics 문서
1. **STM32H725 Reference Manual**
   - OCTOSPI 레지스터 맵
   - GPIO 설정

2. **AN5050: OCTOSPI 인터페이스 사용 가이드**
   - QSPI 메모리 매핑
   - 성능 최적화

### 커뮤니티 리소스
1. **Nordic DevZone**
   - nRF7002 관련 Q&A
   - https://devzone.nordicsemi.com/

2. **STM32 Community**
   - OCTOSPI 활용 사례

---

## 🔍 알려진 이슈 및 제한사항

### 현재 제한사항
1. **펌웨어 로딩 미구현**
   - 현재는 파싱만 가능
   - 실제 RPU로 전송 필요

2. **IRQ 핸들러 미구현**
   - 인터럽트 설정은 완료
   - 콜백 처리 로직 필요

3. **클럭 속도 보수적**
   - 안정성을 위해 1.6MHz로 동작
   - 최적화 여지 있음

### 해결 방법
- Phase 1~3 계획에 따라 단계적 구현
- Nordic 샘플 코드 및 Zephyr 참조 구현 활용

---

## 📝 변경 이력

| 날짜 | 버전 | 변경 내용 |
|------|------|-----------|
| 2026-01-16 | 1.0 | 초기 Feasibility 검토 완료 |
|  |  | - 하드웨어 통신 검증 |
|  |  | - 펌웨어 준비 및 파싱 |
|  |  | - 개발 환경 구축 |

---

## 👥 Contact

**프로젝트**: WR Embedded (Soft Robot Main Controller)
**Wi-Fi 모듈**: nRF7002 (WM02C)
**담당**: Wi-Fi 드라이버 통합 팀

---

**End of Document**
