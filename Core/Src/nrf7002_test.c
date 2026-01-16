/*
 * nrf7002_test.c - nRF7002 QSPI Communication Test
 *
 * STM32H7 OCTOSPI를 사용하여 nRF7002와 통신 테스트
 *
 * 참고: nRF7002는 SPI Mode 0 (CPOL=0, CPHA=0) 사용
 *       최대 클럭: 32MHz (QSPI), 8MHz (SPI)
 */

#include "nrf7002_test.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ========== Firmware Image Structures ========== */
#define NRF_WIFI_PATCH_SIGNATURE 0xDEAD1EAF
#define NRF_WIFI_PATCH_HASH_LEN 32

// nRF70 펌웨어 이미지 타입
typedef enum {
    NRF70_IMAGE_UMAC_PRI = 0,
    NRF70_IMAGE_UMAC_SEC,
    NRF70_IMAGE_LMAC_PRI,
    NRF70_IMAGE_LMAC_SEC,
} nrf70_image_id_t;

// 펌웨어 이미지 헤더
typedef struct __attribute__((packed)) {
    uint32_t type;
    uint32_t len;
    uint8_t  data[];
} nrf70_fw_image_t;

// 펌웨어 전체 헤더
typedef struct __attribute__((packed)) {
    uint32_t signature;
    uint32_t num_images;
    uint32_t version;
    uint32_t feature_flags;
    uint32_t len;
    uint8_t  hash[NRF_WIFI_PATCH_HASH_LEN];
    uint8_t  data[];
} nrf70_fw_image_info_t;

/* ========== Private Variables ========== */
static OSPI_HandleTypeDef *nrf70_hospi = NULL;

/* ========== Private Function Prototypes ========== */
static HAL_StatusTypeDef NRF70_SendCommand(uint8_t cmd, uint8_t *rx_data, uint32_t rx_len);
static HAL_StatusTypeDef NRF70_WriteCommand(uint8_t cmd, uint8_t *tx_data, uint32_t tx_len);

/* ========== Public Functions ========== */

HAL_StatusTypeDef NRF70_Test_Init(OSPI_HandleTypeDef *hospi)
{
    if (hospi == NULL) {
        return HAL_ERROR;
    }

    nrf70_hospi = hospi;

    // OCTOSPI가 이미 MX_OCTOSPI1_Init()에서 초기화됨
    // 추가 설정이 필요하면 여기서 수행

    return HAL_OK;
}

void NRF70_PowerOn(void)
{
    // PC5 = WM_BUCK -> HIGH로 설정하여 전원 켜기
	HAL_GPIO_WritePin(WM_BUCK_GPIO_Port, WM_BUCK_Pin, GPIO_PIN_SET);

    // 전원 안정화 대기 (최소 10ms, 넉넉히 50ms)
    HAL_Delay(50);
}

void NRF70_PowerOff(void)
{
	HAL_GPIO_WritePin(WM_BUCK_GPIO_Port, WM_BUCK_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef NRF70_ReadSR0(uint8_t *value)
{
    if (value == NULL) {
        return HAL_ERROR;
    }
    return NRF70_SendCommand(NRF70_CMD_RDSR0, value, 1);
}

HAL_StatusTypeDef NRF70_ReadSR1(uint8_t *value)
{
    if (value == NULL) {
        return HAL_ERROR;
    }
    return NRF70_SendCommand(NRF70_CMD_RDSR1, value, 1);
}

HAL_StatusTypeDef NRF70_ReadSR2(uint8_t *value)
{
    if (value == NULL) {
        return HAL_ERROR;
    }
    return NRF70_SendCommand(NRF70_CMD_RDSR2, value, 1);
}

HAL_StatusTypeDef NRF70_WriteSR2(uint8_t value)
{
    return NRF70_WriteCommand(NRF70_CMD_WRSR2, &value, 1);
}

bool NRF70_RunFullTest(NRF70_TestResult_t *result)
{
    if (result == NULL) {
        return false;
    }

    // 결과 초기화
    memset(result, 0, sizeof(NRF70_TestResult_t));

    // QSPI 초기화 확인
    if (nrf70_hospi == NULL) {
        result->qspi_init_ok = false;
        return false;
    }
    result->qspi_init_ok = true;

    // 전원 켜기
    NRF70_PowerOn();

    // Test 1: RDSR0 읽기
    HAL_StatusTypeDef status = NRF70_ReadSR0(&result->rdsr0_value);
    if (status == HAL_OK) {
        result->rdsr0_ok = true;
        // 0xFF나 0x00이 아니면 뭔가 읽힌 것
        if (result->rdsr0_value != 0xFF && result->rdsr0_value != 0x00) {
            // 칩이 응답함!
        }
    } else {
        result->error_code = status;
    }

    // Test 2: RDSR1 읽기 (RPU Awake 상태)
    status = NRF70_ReadSR1(&result->rdsr1_value);
    if (status == HAL_OK) {
        result->rdsr1_ok = true;
        result->rpu_awake = (result->rdsr1_value & NRF70_RPU_AWAKE_BIT) != 0;
    } else {
        result->error_code = status;
    }

    // Test 3: RDSR2 읽기
    status = NRF70_ReadSR2(&result->rdsr2_value);
    if (status == HAL_OK) {
        result->rdsr2_ok = true;
    } else {
        result->error_code = status;
    }

    // Test 4: WRSR2 쓰기/읽기 테스트 (Echo Test)
    // 0xAA 패턴 쓰고 다시 읽어서 확인
    uint8_t test_pattern = 0xAA;
    uint8_t read_back = 0;

    status = NRF70_WriteSR2(test_pattern);
    if (status == HAL_OK) {
        HAL_Delay(1);  // 쓰기 완료 대기
        status = NRF70_ReadSR2(&read_back);
        if (status == HAL_OK && read_back == test_pattern) {
            // Echo 테스트 성공!
        }
    }

    // 모든 테스트 통과 여부
    return (result->rdsr0_ok && result->rdsr1_ok && result->rdsr2_ok);
}

void NRF70_PrintTestResult(const NRF70_TestResult_t *result)
{
    if (result == NULL) {
        return;
    }

    printf("\r\n========== nRF7002 (WM02C) Test Result ==========\r\n");
    printf("QSPI Init:    %s\r\n", result->qspi_init_ok ? "OK" : "FAIL");
    printf("RDSR0:        %s (0x%02X)\r\n", result->rdsr0_ok ? "OK" : "FAIL", result->rdsr0_value);
    printf("RDSR1:        %s (0x%02X)\r\n", result->rdsr1_ok ? "OK" : "FAIL", result->rdsr1_value);
    printf("RDSR2:        %s (0x%02X)\r\n", result->rdsr2_ok ? "OK" : "FAIL", result->rdsr2_value);
    printf("RPU Awake:    %s\r\n", result->rpu_awake ? "YES" : "NO");

    if (result->error_code != 0) {
        printf("Error Code:   0x%08lX\r\n", result->error_code);
    }

    // 결과 판정
    if (result->rdsr0_ok && result->rdsr1_ok && result->rdsr2_ok) {
        if (result->rdsr0_value != 0xFF && result->rdsr0_value != 0x00) {
            printf("\r\n>>> nRF7002 ALIVE! Hardware OK! <<<\r\n");
        } else {
            printf("\r\n>>> WARNING: Read all 0x00 or 0xFF - Check wiring <<<\r\n");
        }
    } else {
        printf("\r\n>>> FAIL: QSPI communication error <<<\r\n");
    }
    printf("=================================================\r\n");
}

/* ========== Private Functions ========== */

/**
 * @brief OCTOSPI를 사용하여 nRF7002에 명령 전송 후 데이터 수신
 * @note nRF7002는 단순 SPI 모드에서 1-line 모드로 시작
 */
static HAL_StatusTypeDef NRF70_SendCommand(uint8_t cmd, uint8_t *rx_data, uint32_t rx_len)
{
    if (nrf70_hospi == NULL || rx_data == NULL) {
        return HAL_ERROR;
    }

    OSPI_RegularCmdTypeDef sCommand = {0};

    // nRF7002 SPI 명령 설정
    // 초기에는 1-line SPI 모드로 통신
    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = cmd;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

    // 주소 없음 (Status Register 읽기는 주소 불필요)
    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;

    // Alternate bytes 없음
    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

    // 더미 사이클 없음 (Status Register 읽기)
    sCommand.DummyCycles        = 0;

    // 데이터 모드
    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.NbData             = rx_len;

    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    // 명령 전송
    HAL_StatusTypeDef status = HAL_OSPI_Command(nrf70_hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        return status;
    }

    // 데이터 수신
    status = HAL_OSPI_Receive(nrf70_hospi, rx_data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    return status;
}

/**
 * @brief OCTOSPI를 사용하여 nRF7002에 명령과 데이터 전송
 */
static HAL_StatusTypeDef NRF70_WriteCommand(uint8_t cmd, uint8_t *tx_data, uint32_t tx_len)
{
    if (nrf70_hospi == NULL || tx_data == NULL) {
        return HAL_ERROR;
    }

    OSPI_RegularCmdTypeDef sCommand = {0};

    sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
    sCommand.Instruction        = cmd;
    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

    sCommand.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    sCommand.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;

    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;

    sCommand.DummyCycles        = 0;

    sCommand.DataMode           = HAL_OSPI_DATA_1_LINE;
    sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    sCommand.NbData             = tx_len;

    sCommand.DQSMode            = HAL_OSPI_DQS_DISABLE;
    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    // 명령 전송
    HAL_StatusTypeDef status = HAL_OSPI_Command(nrf70_hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    if (status != HAL_OK) {
        return status;
    }

    // 데이터 전송
    status = HAL_OSPI_Transmit(nrf70_hospi, tx_data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    return status;
}

/* ========== Firmware Loader Functions ========== */

void NRF70_PrintFirmwareInfo(const uint8_t *fw_data, uint32_t fw_size)
{
    if (fw_data == NULL || fw_size < sizeof(nrf70_fw_image_info_t)) {
        printf("ERROR: Invalid firmware data\r\n");
        return;
    }

    const nrf70_fw_image_info_t *fw_info = (const nrf70_fw_image_info_t *)fw_data;

    printf("\r\n========== nRF7002 Firmware Information ==========\r\n");
    printf("Signature:    0x%08lX %s\r\n", fw_info->signature,
           (fw_info->signature == NRF_WIFI_PATCH_SIGNATURE) ? "(Valid)" : "(INVALID!)");
    printf("Num Images:   %lu\r\n", fw_info->num_images);
    printf("Version:      %lu.%lu.%lu.%lu\r\n",
           (fw_info->version >> 24) & 0xFF,
           (fw_info->version >> 16) & 0xFF,
           (fw_info->version >> 8) & 0xFF,
           (fw_info->version >> 0) & 0xFF);
    printf("Features:     0x%08lX\r\n", fw_info->feature_flags);
    printf("Total Size:   %lu bytes\r\n", fw_size);
    printf("Data Length:  %lu bytes\r\n", fw_info->len);

    // SHA256 해시 출력
    printf("SHA256 Hash:  ");
    for (int i = 0; i < 16; i++) {
        printf("%02X", fw_info->hash[i]);
    }
    printf("...\r\n");

    // 각 이미지 파싱
    if (fw_info->signature == NRF_WIFI_PATCH_SIGNATURE) {
        const uint8_t *data_ptr = fw_info->data;
        uint32_t offset = 0;

        printf("\r\nFirmware Images:\r\n");
        for (uint32_t i = 0; i < fw_info->num_images && i < 4; i++) {
            if (offset + sizeof(nrf70_fw_image_t) > fw_info->len) {
                break;
            }

            const nrf70_fw_image_t *img = (const nrf70_fw_image_t *)(data_ptr + offset);
            const char *img_name[] = {"UMAC_PRI", "UMAC_SEC", "LMAC_PRI", "LMAC_SEC"};

            printf("  [%lu] %-10s: Type=%lu, Size=%lu bytes\r\n",
                   i,
                   (img->type < 4) ? img_name[img->type] : "UNKNOWN",
                   img->type,
                   img->len);

            offset += sizeof(nrf70_fw_image_t) + img->len;
        }
    }

    printf("==================================================\r\n");
}

HAL_StatusTypeDef NRF70_LoadFirmware(const uint8_t *fw_data, uint32_t fw_size)
{
    if (fw_data == NULL || fw_size == 0) {
        printf("ERROR: Invalid firmware parameters\r\n");
        return HAL_ERROR;
    }

    if (nrf70_hospi == NULL) {
        printf("ERROR: QSPI not initialized\r\n");
        return HAL_ERROR;
    }

    printf("\r\n>>> Starting Firmware Load...\r\n");

    // 펌웨어 헤더 검증
    const nrf70_fw_image_info_t *fw_info = (const nrf70_fw_image_info_t *)fw_data;

    if (fw_info->signature != NRF_WIFI_PATCH_SIGNATURE) {
        printf("ERROR: Invalid firmware signature: 0x%08lX\r\n", fw_info->signature);
        return HAL_ERROR;
    }

    printf("Firmware signature valid: 0x%08lX\r\n", fw_info->signature);
    printf("Number of images: %lu\r\n", fw_info->num_images);

    // TODO: 실제 펌웨어 로딩 구현
    // 현재는 feasibility 테스트용으로 헤더 파싱만 수행
    // 전체 드라이버 포팅 시 nrf70-bm의 hal_fw_patch_loader.c 사용 필요

    printf("\r\n>>> Firmware Load: STUB IMPLEMENTATION <<<\r\n");
    printf("    실제 로딩을 위해서는 nrf70-bm 드라이버 통합 필요\r\n");
    printf("    - hal_fw_patch_chunk_load() 구현\r\n");
    printf("    - hal_fw_patch_boot() 구현\r\n");
    printf("    - RPU 메모리 맵핑 및 전송 프로토콜 구현\r\n");

    return HAL_OK;
}
