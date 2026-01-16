/*
 * nrf7002_test.h - nRF7002 QSPI Communication Test
 *
 * STM32H7 + FreeRTOS 환경에서 nRF7002 (WM02C) 통신 테스트
 * 목표: Chip ID 읽어서 하드웨어 동작 확인
 */

#ifndef NRF7002_TEST_H
#define NRF7002_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#include "octospi.h"

/* ========== nRF7002 SPI/QSPI Commands ========== */
// Status Register Commands (from nRF7002 Product Specification)
#define NRF70_CMD_RDSR0         0x05    // Read Status Register 0
#define NRF70_CMD_RDSR1         0x1F    // Read Status Register 1 (RPU awake)
#define NRF70_CMD_RDSR2         0x2F    // Read Status Register 2
#define NRF70_CMD_WRSR2         0x3F    // Write Status Register 2

// Expected values
#define NRF70_RDSR0_EXPECTED    0x02    // 예상 값 (데이터시트 확인 필요)
#define NRF70_RPU_AWAKE_BIT     0x01    // RPU가 깨어있을 때 비트

/* ========== nRF7002 Power/Reset Pins ========== */
// WM02C 핀맵에 따라 수정 필요 - 현재 PCB 회로도 확인 후 설정
// #define NRF70_BUCKEN_PORT       GPIOx
// #define NRF70_BUCKEN_PIN        GPIO_PIN_x
// #define NRF70_IOVDD_EN_PORT     GPIOx
// #define NRF70_IOVDD_EN_PIN      GPIO_PIN_x
// #define NRF70_HOST_IRQ_PORT     GPIOx
// #define NRF70_HOST_IRQ_PIN      GPIO_PIN_x

/* ========== Test Result Structure ========== */
typedef struct {
    bool        qspi_init_ok;       // QSPI 초기화 성공
    bool        rdsr0_ok;           // RDSR0 읽기 성공
    bool        rdsr1_ok;           // RDSR1 읽기 성공
    bool        rdsr2_ok;           // RDSR2 읽기 성공
    bool        rpu_awake;          // RPU 깨어있음
    uint8_t     rdsr0_value;        // RDSR0 읽은 값
    uint8_t     rdsr1_value;        // RDSR1 읽은 값
    uint8_t     rdsr2_value;        // RDSR2 읽은 값
    uint32_t    error_code;         // HAL 에러 코드
} NRF70_TestResult_t;

/* ========== Function Prototypes ========== */

/**
 * @brief nRF7002 테스트 초기화
 * @param hospi OCTOSPI 핸들 포인터
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NRF70_Test_Init(OSPI_HandleTypeDef *hospi);

/**
 * @brief nRF7002 전원 켜기 시퀀스
 * @note BUCKEN -> IOVDD_EN 순서로 활성화
 */
void NRF70_PowerOn(void);

/**
 * @brief nRF7002 전원 끄기
 */
void NRF70_PowerOff(void);

/**
 * @brief Status Register 0 읽기 (RDSR0)
 * @param value 읽은 값 저장할 포인터
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NRF70_ReadSR0(uint8_t *value);

/**
 * @brief Status Register 1 읽기 (RDSR1) - RPU Awake 상태
 * @param value 읽은 값 저장할 포인터
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NRF70_ReadSR1(uint8_t *value);

/**
 * @brief Status Register 2 읽기 (RDSR2)
 * @param value 읽은 값 저장할 포인터
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NRF70_ReadSR2(uint8_t *value);

/**
 * @brief Status Register 2 쓰기 (WRSR2)
 * @param value 쓸 값
 * @return HAL_OK on success
 */
HAL_StatusTypeDef NRF70_WriteSR2(uint8_t value);

/**
 * @brief nRF7002 풀 테스트 실행
 * @param result 테스트 결과 저장할 구조체 포인터
 * @return true if all tests passed
 */
bool NRF70_RunFullTest(NRF70_TestResult_t *result);

/**
 * @brief 테스트 결과 출력 (UART 디버그용)
 * @param result 테스트 결과 구조체 포인터
 */
void NRF70_PrintTestResult(const NRF70_TestResult_t *result);

/**
 * @brief nRF7002 펌웨어 로딩 (단순 버전 - Feasibility 테스트용)
 * @param fw_data 펌웨어 바이너리 데이터 포인터
 * @param fw_size 펌웨어 크기 (bytes)
 * @return HAL_OK on success
 * @note 실제 포팅 시 nrf70-bm 드라이버 전체를 사용해야 함
 */
HAL_StatusTypeDef NRF70_LoadFirmware(const uint8_t *fw_data, uint32_t fw_size);

/**
 * @brief 펌웨어 헤더 정보 출력 (디버그용)
 * @param fw_data 펌웨어 바이너리 데이터 포인터
 * @param fw_size 펌웨어 크기 (bytes)
 */
void NRF70_PrintFirmwareInfo(const uint8_t *fw_data, uint32_t fw_size);

#ifdef __cplusplus
}
#endif

#endif /* NRF7002_TEST_H */
