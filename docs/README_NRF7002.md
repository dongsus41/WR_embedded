# nRF7002 Wi-Fi 드라이버 프로젝트

STM32H725RG + nRF7002 Wi-Fi 모듈 통합 프로젝트

---

## 🚀 Quick Start

### 현재 상태
✅ **Feasibility 확인 완료** (2026-01-16)

- nRF7002 하드웨어 통신 검증 완료
- 펌웨어 임베딩 및 파싱 완료
- 드라이버 통합 준비 완료

### 테스트 실행
1. 프로젝트 빌드 및 다운로드
2. 시리얼 콘솔 연결 (115200 baud)
3. 보드 리셋 후 다음 출력 확인:

```
*** Starting nRF7002 Communication Test ***
QSPI Init:    OK
RDSR0:        OK (0x42)
...
>>> nRF7002 ALIVE! Hardware OK! <<<

Firmware signature valid: 0xDEAD1EAF
Number of images: 4
```

---

## 📚 문서

### [검토 보고서](./NRF7002_WIFI_DRIVER_REVIEW.md)
Feasibility 테스트 결과 및 현재 구현 상태

**주요 내용**:
- 하드웨어 구성 및 핀맵
- OCTOSPI 통신 검증 결과
- 펌웨어 파싱 및 메타데이터
- 성능 고려사항

### [통합 계획](./NRF7002_INTEGRATION_PLAN.md)
nrf70-bm 드라이버 전체 통합 로드맵

**Phase 1**: 펌웨어 로딩 구현 (2-3주)
**Phase 2**: 드라이버 통합 (4-6주)
**Phase 3**: 네트워크 스택 (4-6주)

---

## 🔧 하드웨어 요구사항

### STM32H725RG
- OCTOSPI1: QSPI 통신
- GPIO PC5: WM_BUCK (전원 제어)
- GPIO PC4: WM_IRQ (인터럽트, EXTI4)

### nRF7002 (WM02C)
- SPI/QSPI 인터페이스
- 최대 클럭: 32MHz
- 전원: 1.7V - 3.6V

### 핀 연결
| STM32 | nRF7002 | 기능 |
|-------|---------|------|
| PB2 | SCLK | Clock |
| PB10 | CSN | Chip Select |
| PB1 | MOSI/IO0 | Data 0 |
| PB0 | MISO/IO1 | Data 1 |
| PA7 | IO2 | Data 2 |
| PA6 | IO3 | Data 3 |
| PC5 | BUCKEN | Power |
| PC4 | HOST_IRQ | Interrupt |

---

## 📁 파일 구조

```
Core/
├── Inc/
│   ├── nrf7002_test.h          # 테스트 드라이버 헤더
│   └── nrf7002_fw.h            # 펌웨어 바이너리 (32.8KB)
└── Src/
    ├── nrf7002_test.c          # 테스트 드라이버 구현
    ├── main.c                  # 메인 로직
    ├── octospi.c               # OCTOSPI 초기화
    └── gpio.c                  # GPIO 설정

nrf70-bm/                       # Nordic 드라이버 (향후 통합)
├── nrf70_bm_lib/
├── nrf_wifi/
└── sdk-nrfxlib/

docs/
├── README_NRF7002.md           # 본 문서
├── NRF7002_WIFI_DRIVER_REVIEW.md
└── NRF7002_INTEGRATION_PLAN.md
```

---

## 🎯 현재 기능

### 구현 완료 ✅
- [x] OCTOSPI 통신
- [x] 전원 제어 (BUCKEN)
- [x] Status Register 읽기/쓰기
- [x] 펌웨어 임베딩 (Scan Only, 32KB)
- [x] 펌웨어 메타데이터 파싱
- [x] IRQ 핀 하드웨어 설정

### 향후 구현 ⏳
- [ ] 펌웨어 로딩 (RPU로 전송)
- [ ] RPU 부팅
- [ ] Wi-Fi 스캔
- [ ] Wi-Fi 연결
- [ ] TCP/IP 통신

---

## 🔨 빌드 방법

### STM32CubeIDE
1. 프로젝트 import
2. Build Project (Ctrl+B)
3. Debug/Run (F11/Ctrl+F11)

### 명령줄
```bash
# 빌드
make -j8

# 플래시
openocd -f interface/stlink.cfg \
        -f target/stm32h7x.cfg \
        -c "program build/firmware.elf verify reset exit"
```

---

## 📊 성능 정보

### 펌웨어 모드별 크기
| 모드 | 크기 | 기능 |
|------|------|------|
| scan_only | 33KB | 스캔만 (현재 사용) |
| radio_test | 43KB | RF 테스트 |
| default | 78KB | 풀 기능 |
| system_with_raw | 82KB | 풀 기능 + Raw |

### 메모리 사용량 (예상)
- Flash: ~150KB (펌웨어 + 드라이버)
- RAM: ~50KB (버퍼 + 스택)

---

## 🐛 알려진 이슈

### Phase 1 (Feasibility)
- ⚠️ RPU 미활성 상태 (펌웨어 미로드) - 정상
- ⚠️ OCTOSPI 클럭 느림 (1.6MHz) - 향후 최적화 필요

### 해결 방법
→ [통합 계획](./NRF7002_INTEGRATION_PLAN.md) 참조

---

## 📞 지원 및 참고자료

### Nordic Semiconductor
- **Product Specification**: https://www.nordicsemi.com/Products/nRF7002
- **DevZone**: https://devzone.nordicsemi.com/
- **nrf70-bm 드라이버**: https://github.com/nrfconnect/nrf70-bm

### STMicroelectronics
- **STM32H7 Reference Manual**: RM0468
- **OCTOSPI 가이드**: AN5050

### 커뮤니티
- STM32 Forum: https://community.st.com/
- Nordic DevZone Q&A

---

## 📝 변경 이력

| 날짜 | 버전 | 내용 |
|------|------|------|
| 2026-01-16 | 1.0 | Feasibility 완료, 문서 작성 |

---

## 📄 라이선스

- **nrf70-bm 드라이버**: BSD-3-Clause (Nordic Semiconductor)
- **STM32 HAL**: BSD-3-Clause (STMicroelectronics)
- **프로젝트 코드**: 프로젝트 라이선스 따름

---

**다음 단계**: [통합 계획](./NRF7002_INTEGRATION_PLAN.md) 참조
