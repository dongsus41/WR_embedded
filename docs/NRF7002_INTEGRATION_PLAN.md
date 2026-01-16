# nRF7002 Wi-Fi 드라이버 통합 계획

**작성일**: 2026-01-16
**목표**: nrf70-bm 드라이버를 STM32H7 + FreeRTOS 환경에 포팅
**최종 목표**: Wi-Fi 스캔, 연결, 데이터 통신 기능 구현

---

## 🎯 통합 로드맵

```
[Phase 1]           [Phase 2]              [Phase 3]
펌웨어 로딩    →   드라이버 통합      →   네트워크 스택
(2-3주)            (4-6주)               (4-6주)

RPU 부팅           Wi-Fi 스캔            TCP/IP 통신
Alive 확인         연결 관리             애플리케이션
```

---

## Phase 1: 펌웨어 로딩 구현

**목표**: nRF7002 RPU(Radio Processing Unit) 부팅 성공

### 1.1 QSPI 메모리 맵핑 구현

**참조 파일**: `nrf70-bm/nrf_wifi/hw_if/hal/src/common/hal_mem.c`

**구현 내용**:
```c
// RPU 메모리 주소 정의 (nRF7002 데이터시트 참조)
#define NRF_WIFI_SYSBUS_BASE_ADDR    0x000C0000
#define NRF_WIFI_GRAMRF_BASE_ADDR    0x0C000000

// QSPI를 통한 메모리 쓰기
HAL_StatusTypeDef NRF70_MemWrite(uint32_t addr, const uint8_t *data, uint32_t len);

// QSPI를 통한 메모리 읽기
HAL_StatusTypeDef NRF70_MemRead(uint32_t addr, uint8_t *data, uint32_t len);
```

**작업 순서**:
1. nRF7002 메모리 맵 분석 (Product Specification 참조)
2. QSPI Address 모드 활성화 (현재는 No Address 모드)
3. `HAL_OSPI_Command()` + `HAL_OSPI_Transmit()` 조합으로 주소 지정 쓰기 구현
4. 테스트: Dummy 데이터 쓰고 읽기 검증

**예상 난이도**: ⭐⭐⭐ (Medium)

### 1.2 펌웨어 로더 포팅

**참조 파일**: `nrf70-bm/nrf_wifi/hw_if/hal/src/common/hal_fw_patch_loader.c`

**핵심 함수**:
```c
enum nrf_wifi_status hal_fw_patch_chunk_load(
    struct nrf_wifi_hal_dev_ctx *hal_dev_ctx,
    enum RPU_PROC_TYPE rpu_proc,
    unsigned int dest_addr,
    const void *fw_chunk_data,
    unsigned int fw_chunk_size);
```

**구현 단계**:
1. `nrf7002_test.c`에 `hal_fw_patch_chunk_load()` 간소화 버전 구현
2. 펌웨어 이미지를 청크 단위로 분할 (예: 1KB씩)
3. 각 청크를 RPU 메모리에 QSPI로 전송
4. 전송 완료 후 체크섬 검증

**구현 예시**:
```c
HAL_StatusTypeDef NRF70_LoadFirmwareImage(
    const nrf70_fw_image_t *image,
    uint32_t dest_addr)
{
    const uint8_t *data = image->data;
    uint32_t remaining = image->len;
    uint32_t chunk_size = 1024;  // 1KB 청크

    while (remaining > 0) {
        uint32_t size = (remaining > chunk_size) ? chunk_size : remaining;

        // QSPI를 통해 RPU 메모리에 쓰기
        if (NRF70_MemWrite(dest_addr, data, size) != HAL_OK) {
            return HAL_ERROR;
        }

        data += size;
        dest_addr += size;
        remaining -= size;
    }

    return HAL_OK;
}
```

**예상 난이도**: ⭐⭐⭐⭐ (High)

### 1.3 RPU 부팅 시퀀스

**참조 함수**: `nrf_wifi_hal_fw_patch_boot()`

**부팅 순서**:
1. LMAC Primary 패치 로드
2. LMAC Secondary 패치 로드
3. LMAC 부팅 신호 전송
4. UMAC Primary 패치 로드
5. UMAC Secondary 패치 로드
6. UMAC 부팅 신호 전송
7. RPU Awake 확인 (RDSR1 & 0x01)

**구현**:
```c
HAL_StatusTypeDef NRF70_BootRPU(const nrf70_fw_image_info_t *fw_info)
{
    // 1. LMAC 패치 로딩
    const nrf70_fw_image_t *lmac_pri = /* 파싱 */;
    const nrf70_fw_image_t *lmac_sec = /* 파싱 */;

    NRF70_LoadFirmwareImage(lmac_pri, LMAC_PRI_ADDR);
    NRF70_LoadFirmwareImage(lmac_sec, LMAC_SEC_ADDR);

    // 2. LMAC 부팅
    NRF70_BootProcessor(RPU_PROC_TYPE_MCU_LMAC);

    // 3. UMAC 패치 로딩
    const nrf70_fw_image_t *umac_pri = /* 파싱 */;
    const nrf70_fw_image_t *umac_sec = /* 파싱 */;

    NRF70_LoadFirmwareImage(umac_pri, UMAC_PRI_ADDR);
    NRF70_LoadFirmwareImage(umac_sec, UMAC_SEC_ADDR);

    // 4. UMAC 부팅
    NRF70_BootProcessor(RPU_PROC_TYPE_MCU_UMAC);

    // 5. RPU Awake 대기 (최대 1초)
    for (int i = 0; i < 100; i++) {
        uint8_t status;
        NRF70_ReadSR1(&status);
        if (status & NRF70_RPU_AWAKE_BIT) {
            return HAL_OK;  // 성공!
        }
        HAL_Delay(10);
    }

    return HAL_TIMEOUT;
}
```

**예상 난이도**: ⭐⭐⭐⭐⭐ (Very High)

### 1.4 검증 방법

**성공 기준**:
```
RDSR1:        OK (0x01)
RPU Awake:    YES  ✅
```

**디버깅 팁**:
- UART로 각 단계별 로그 출력
- 메모리 쓰기 후 읽기로 검증
- 타임아웃 값 넉넉하게 설정 (디버깅 중)

---

## Phase 2: nrf70-bm 드라이버 통합

**목표**: Wi-Fi 스캔 기능 구현

### 2.1 Platform Abstraction Layer (PAL) 구현

**참조**: `nrf70-bm/nrf_wifi/hw_if/hal/inc/common/pal.h`

**필수 구현 함수**:

#### 메모리 관리
```c
void* nrf_wifi_osal_mem_alloc(size_t size);
void  nrf_wifi_osal_mem_free(void *buf);
void* nrf_wifi_osal_mem_cpy(void *dest, const void *src, size_t count);
void* nrf_wifi_osal_mem_set(void *start, int val, size_t size);
```

**FreeRTOS 구현 예시**:
```c
void* nrf_wifi_osal_mem_alloc(size_t size) {
    return pvPortMalloc(size);
}

void nrf_wifi_osal_mem_free(void *buf) {
    vPortFree(buf);
}
```

#### 스레드/태스크 관리
```c
void* nrf_wifi_osal_task_create(const char *task_name,
                                 void (*task_func)(void *),
                                 void *task_params,
                                 unsigned int stack_size,
                                 unsigned int priority);
void  nrf_wifi_osal_task_kill(void *task_handle);
void  nrf_wifi_osal_sleep_ms(unsigned int msecs);
```

**FreeRTOS 구현 예시**:
```c
void* nrf_wifi_osal_task_create(const char *name,
                                 void (*func)(void *),
                                 void *params,
                                 unsigned int stack_size,
                                 unsigned int priority)
{
    TaskHandle_t handle;
    BaseType_t ret = xTaskCreate(func, name, stack_size/4, params, priority, &handle);
    return (ret == pdPASS) ? handle : NULL;
}

void nrf_wifi_osal_sleep_ms(unsigned int msecs) {
    vTaskDelay(pdMS_TO_TICKS(msecs));
}
```

#### 동기화 (Mutex, Semaphore)
```c
void* nrf_wifi_osal_spinlock_alloc(void);
void  nrf_wifi_osal_spinlock_free(void *lock);
void  nrf_wifi_osal_spinlock_take(void *lock);
void  nrf_wifi_osal_spinlock_rel(void *lock);
```

**FreeRTOS 구현 (Mutex 사용)**:
```c
void* nrf_wifi_osal_spinlock_alloc(void) {
    return xSemaphoreCreateMutex();
}

void nrf_wifi_osal_spinlock_take(void *lock) {
    xSemaphoreTake((SemaphoreHandle_t)lock, portMAX_DELAY);
}
```

#### 타이머
```c
void* nrf_wifi_osal_timer_alloc(void);
void  nrf_wifi_osal_timer_init(void *timer,
                                void (*callback)(void *),
                                void *data);
void  nrf_wifi_osal_timer_schedule(void *timer, unsigned long duration);
```

**예상 난이도**: ⭐⭐⭐ (Medium) - FreeRTOS API 1:1 매핑

### 2.2 HAL 레이어 포팅

**통합 파일 목록**:
- `hal_api_common.c` - HAL 초기화 및 관리
- `hal_mem.c` - 메모리 접근
- `hal_interrupt.c` - 인터럽트 처리
- `hal_fw_patch_loader.c` - 펌웨어 로더 (Phase 1에서 구현)

**디렉토리 구조 제안**:
```
Core/
├── Src/
│   └── nrf70_port/
│       ├── nrf70_pal.c          # PAL 구현
│       ├── nrf70_hal_mem.c      # HAL 메모리 (QSPI 래퍼)
│       └── nrf70_hal_irq.c      # HAL 인터럽트
└── Inc/
    └── nrf70_port/
        ├── nrf70_pal.h
        └── nrf70_config.h       # 설정 헤더
```

**예상 난이도**: ⭐⭐⭐⭐ (High)

### 2.3 FMAC 레이어 통합

**참조**: `nrf70-bm/nrf_wifi/fw_if/umac_if/src/common/fmac_api_common.c`

**핵심 함수**:
```c
// Wi-Fi 드라이버 초기화
struct nrf_wifi_fmac_priv* nrf_wifi_fmac_init(
    struct nrf_wifi_fmac_callbk_fns *callbk_fns);

// 디바이스 추가
struct nrf_wifi_fmac_dev_ctx* nrf_wifi_fmac_dev_add(
    struct nrf_wifi_fmac_priv *fmac_priv);

// 펌웨어 로딩 및 초기화
enum nrf_wifi_status nrf_wifi_fmac_fw_load(
    struct nrf_wifi_fmac_dev_ctx *fmac_dev_ctx,
    struct nrf_wifi_fmac_fw_info *fmac_fw);
```

**통합 예시**:
```c
// main.c 또는 wifi_task.c
void wifi_init_task(void *params)
{
    // 1. FMAC 초기화
    struct nrf_wifi_fmac_callbk_fns callbacks = {
        .scan_done_callbk_fn = wifi_scan_done_callback,
        .scan_res_callbk_fn  = wifi_scan_result_callback,
        // ...
    };

    fmac_priv = nrf_wifi_fmac_init(&callbacks);

    // 2. 디바이스 추가
    fmac_dev = nrf_wifi_fmac_dev_add(fmac_priv);

    // 3. 펌웨어 로딩
    struct nrf_wifi_fmac_fw_info fw_info = {
        .lmac_patch_pri = { .data = /* ... */, .size = /* ... */ },
        .lmac_patch_sec = { .data = /* ... */, .size = /* ... */ },
        .umac_patch_pri = { .data = /* ... */, .size = /* ... */ },
        .umac_patch_sec = { .data = /* ... */, .size = /* ... */ },
    };

    nrf_wifi_fmac_fw_load(fmac_dev, &fw_info);

    // 4. Wi-Fi 스캔 시작
    nrf_wifi_fmac_scan_start(fmac_dev, /* ... */);
}
```

**예상 난이도**: ⭐⭐⭐⭐⭐ (Very High)

### 2.4 IRQ 핸들러 구현

**현재 상태**: 하드웨어 설정 완료, 콜백 미구현

**구현 위치**: `Core/Src/gpio.c` 또는 `Core/Src/nrf70_port/nrf70_hal_irq.c`

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WM_IRQ_Pin) {
        // nRF7002 IRQ 발생
        extern void nrf_wifi_hal_irq_handler(void);
        nrf_wifi_hal_irq_handler();
    }
}
```

**드라이버 내부 처리**:
1. IRQ 원인 파악 (RX 데이터, TX 완료, 이벤트 등)
2. 이벤트 큐에 추가
3. Wi-Fi 태스크 깨우기

**예상 난이도**: ⭐⭐⭐ (Medium)

### 2.5 검증 방법

**Wi-Fi 스캔 테스트**:
```c
// 스캔 시작
nrf_wifi_fmac_scan_start(fmac_dev, scan_params);

// 콜백에서 결과 수신
void wifi_scan_result_callback(void *ctx,
                                struct nrf_wifi_umac_event_new_scan_results *scan_res)
{
    printf("SSID: %s, RSSI: %d dBm\n",
           scan_res->ssid.nrf_wifi_ssid,
           scan_res->signal.signal);
}
```

**성공 기준**:
- ✅ 주변 AP 목록 출력
- ✅ SSID, BSSID, RSSI, 채널 정보 정확

---

## Phase 3: 네트워크 스택 연동

**목표**: TCP/IP 통신 기능

### 3.1 Wi-Fi 연결 관리

**구현 함수**:
```c
// AP 연결
nrf_wifi_fmac_add_vif(fmac_dev, ...);
nrf_wifi_fmac_start_xmit(fmac_dev, ...);

// 인증
nrf_wifi_fmac_auth(fmac_dev, ...);
nrf_wifi_fmac_assoc(fmac_dev, ...);
```

**연결 시퀀스**:
1. 스캔으로 대상 AP 찾기
2. 인증 (Open/WPA2/WPA3)
3. Association
4. 4-Way Handshake (WPA)
5. 연결 완료

### 3.2 TCP/IP 스택 통합

**옵션 1: LwIP**
- 가벼운 TCP/IP 스택
- FreeRTOS와 잘 통합됨
- STM32CubeMX에서 지원

**옵션 2: FreeRTOS+TCP**
- FreeRTOS 네이티브 스택
- 간단한 설정

**통합 지점**:
```c
// netif (Network Interface) 구현
struct netif wifi_netif;

// Low-level output (nRF7002로 전송)
static err_t wifi_netif_output(struct netif *netif, struct pbuf *p)
{
    // pbuf 데이터를 nrf_wifi_fmac_start_xmit()로 전달
    nrf_wifi_fmac_start_xmit(fmac_dev, p->payload, p->tot_len);
    return ERR_OK;
}

// RX 콜백 (nRF7002에서 수신)
void wifi_rx_callback(void *ctx, void *frm)
{
    // 수신 프레임을 pbuf로 변환
    struct pbuf *p = pbuf_alloc(PBUF_RAW, frm_len, PBUF_RAM);
    memcpy(p->payload, frm, frm_len);

    // LwIP로 전달
    wifi_netif.input(p, &wifi_netif);
}
```

### 3.3 DHCP 및 DNS

**LwIP 설정**:
```c
netif_add(&wifi_netif, NULL, NULL, NULL, NULL, wifi_netif_init, tcpip_input);
netif_set_default(&wifi_netif);
netif_set_up(&wifi_netif);

// DHCP 시작
dhcp_start(&wifi_netif);
```

### 3.4 애플리케이션 프로토콜

**예시: HTTP 클라이언트**
```c
#include "lwip/apps/http_client.h"

void http_get_example(void)
{
    httpc_connection_t settings;
    httpc_state_t *connection;

    httpc_get_file_dns("example.com", 80, "/api/data",
                       &settings, http_callback, NULL, &connection);
}
```

**예시: MQTT**
```c
#include "lwip/apps/mqtt.h"

mqtt_client_t *mqtt_client;
mqtt_client = mqtt_client_new();

mqtt_connect(mqtt_client, "broker.hivemq.com", 1883,
             mqtt_connection_cb, NULL, &mqtt_client_info);
```

---

## 📁 최종 프로젝트 구조

```
WR_embedded/
├── Core/
│   ├── Inc/
│   │   ├── nrf7002_test.h
│   │   ├── nrf7002_fw.h
│   │   └── nrf70_port/
│   │       ├── nrf70_pal.h
│   │       ├── nrf70_config.h
│   │       └── nrf70_hal.h
│   └── Src/
│       ├── nrf7002_test.c
│       ├── main.c
│       └── nrf70_port/
│           ├── nrf70_pal.c          # PAL 구현
│           ├── nrf70_hal_mem.c      # QSPI 메모리 접근
│           ├── nrf70_hal_irq.c      # 인터럽트 처리
│           └── nrf70_fmac_port.c    # FMAC 포팅
│
├── Middlewares/
│   ├── LwIP/                        # TCP/IP 스택
│   └── nrf70_driver/                # nrf70-bm 드라이버 (선택한 파일만)
│       ├── hal/
│       ├── fmac/
│       └── inc/
│
└── nrf70-bm/                        # 참조용 원본 (빌드 제외)
```

---

## ⚠️ 주의사항 및 Risk

### 기술적 Risk

1. **메모리 맵핑 복잡도**
   - nRF7002의 내부 메모리 구조 파악 필요
   - QSPI 주소 모드 설정 오류 가능성
   - **완화**: Nordic 샘플 코드 면밀히 분석, 단계별 검증

2. **타이밍 이슈**
   - RPU 부팅 시간 가변적
   - QSPI 클럭 속도에 따른 안정성
   - **완화**: 타임아웃 충분히 설정, 클럭 점진적 증가

3. **FreeRTOS 통합**
   - 스택 크기 부족 → Hard Fault
   - 우선순위 역전 → 데드락
   - **완화**: 스택 크기 넉넉히 (8KB), 우선순위 계획 수립

4. **메모리 부족**
   - 드라이버 코드 + 버퍼 + TCP/IP 스택
   - STM32H725: RAM 564KB (충분하지만 주의 필요)
   - **완화**: 메모리 프로파일링, 불필요한 기능 제거

### 일정 Risk

| Phase | 예상 기간 | Risk 요인 | 완화 방안 |
|-------|-----------|-----------|-----------|
| Phase 1 | 2-3주 | 메모리 맵핑 복잡도 | Nordic 지원 활용 |
| Phase 2 | 4-6주 | PAL/HAL 디버깅 | 샘플 코드 참조, 단위 테스트 |
| Phase 3 | 4-6주 | TCP/IP 통합 이슈 | LwIP 경험자 자문 |

---

## 🛠 개발 도구 및 리소스

### 필수 도구
- **디버거**: ST-Link V3 (실시간 디버깅)
- **Logic Analyzer**: QSPI 신호 분석용 (옵션)
- **Wi-Fi Sniffer**: Wireshark + Wi-Fi 어댑터 (패킷 캡처)

### 참조 리포지토리
1. **nrf70-bm**: https://github.com/nrfconnect/nrf70-bm
2. **sdk-nrfxlib**: https://github.com/nrfconnect/sdk-nrfxlib
3. **LwIP**: https://git.savannah.nongnu.org/git/lwip.git

### 커뮤니티
- **Nordic DevZone**: https://devzone.nordicsemi.com/
- **STM32 Forum**: https://community.st.com/

---

## 📊 성공 지표 (KPI)

### Phase 1
- ✅ RPU Awake 비트 활성화 (RDSR1 & 0x01)
- ✅ 펌웨어 로딩 시간 < 5초

### Phase 2
- ✅ Wi-Fi AP 스캔 성공 (>= 3개 AP 검출)
- ✅ RSSI 정확도 ± 3 dBm

### Phase 3
- ✅ AP 연결 성공률 >= 95%
- ✅ TCP 연결 성공 (HTTP GET 200 OK)
- ✅ 데이터 전송 속도 >= 1 Mbps
- ✅ 패킷 손실률 < 1%

---

## 📝 다음 단계 (Immediate Next Steps)

### 1주차
- [ ] nRF7002 Product Specification 정독 (메모리 맵 중점)
- [ ] QSPI Address 모드 활성화 테스트
- [ ] `NRF70_MemWrite()` 구현 및 검증

### 2주차
- [ ] 펌웨어 이미지 파싱 로직 개선
- [ ] 청크 전송 구현
- [ ] LMAC Primary 패치 로딩 테스트

### 3주차
- [ ] UMAC 패치 로딩
- [ ] RPU 부팅 시퀀스 구현
- [ ] RPU Awake 확인

---

**문서 버전**: 1.0
**최종 수정**: 2026-01-16
**다음 리뷰**: Phase 1 완료 후
