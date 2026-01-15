#ifndef EMC2303_H
#define EMC2303_H
	
#ifdef __cplusplus
	extern "C" {
#endif
	
#include <stdint.h>
	
/* I2C 타임아웃(필요 시 프로젝트 전역에서 재정의) */
#ifndef HAL_I2C_TIMEOUT_MS
#define HAL_I2C_TIMEOUT_MS   10
#endif

#ifndef CLAMP
#define CLAMP(x,lo,hi)   ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

/* ===== I2C addresses (7‑bit) decoded by ADDR_SEL resistor ===== */
#define EMC2303_ADDR_IC1   0x2E  // ADDR_SEL = 4.7kΩ
#define EMC2303_ADDR_IC2   0x2C  // ADDR_SEL = 10kΩ

/* ===== Common registers ===== */
#define REG_CONFIGURATION          0x20
#define REG_ENABLE                 0x29
#define REG_PWM_POLARITY           0x2A  // PLRITYx bits: 0=normal, 1=invert
#define REG_PWM_OUTPUT_CFG         0x2B  // PMOTx bits: 0=open‑drain, 1=push‑pull
#define REG_PWM_BASEF_45           0x2C  // PMB4[1:0], PMB5[1:0]
#define REG_PWM_BASEF_123          0x2D  // PMB1/2/3[1:0]
	
/* ===== PWM base frequency codes (Table 4‑2) ===== */
typedef enum
{
    EMC_PWM_BASE_26k   = 0,
    EMC_PWM_BASE_19k5  = 1,
    EMC_PWM_BASE_4k882 = 2,
    EMC_PWM_BASE_2k441 = 3,
} emc_pwm_base_t;

/* ===== Public configuration profile ===== */
typedef enum
{
    EMC_PROFILE_4WIRE_FAN,     // PWM open‑drain, ~25 kHz base, /1 divider → ~25 kHz out
    EMC_PROFILE_POWER_GATING,  // PWM push‑pull into TPS ON/OFF; use low freq (e.g., ≈500 Hz)
} emc_profile_t;

/* 6채널 논리 매핑용 구조체 */
typedef struct
{
	uint8_t addr;
	uint8_t ch;
} emc_map_t;

/* 애플리케이션 제공 (main.c에 정의) */
extern I2C_HandleTypeDef hi2c4;

/* 6채널 논리 매핑 (IC1: ch1..3, IC2: ch4..6) */
//extern const emc_map_t LCH_MAP[6];

/* === Public API === */
void EMC2303_Init(uint8_t addr7, emc_profile_t profile, uint32_t target_pwm_hz);
void EMC2303_SetDutyPct(uint8_t addr7, uint8_t ch, float pct);
void EMC2303_EnableClosedLoopRPM(uint8_t addr7, uint8_t ch, uint32_t rpm);
void EMC2303_ReadRPM(uint8_t addr7, uint8_t ch, uint32_t *out_rpm);

/* 6채널 편의 API */
void Fans6_Init(emc_profile_t profile, uint32_t target_pwm_hz);
void Fan6_SetDuty(uint8_t logical_ch0_5, float pct);      /* [MOD] was logical_ch1_6 */
void Fan6_EnableRPM(uint8_t logical_ch0_5, uint32_t rpm); /* [MOD] was logical_ch1_6 */
void Fan6_ReadRPM(uint8_t logical_ch0_5, uint32_t *rpm);  /* [MOD] was logical_ch1_6 */
	
/**
 * @brief 팬 Duty 설정 (mutex 자동 획득)
 * @param ch0_5 논리 채널 (0~5)
 * @param pct Duty (0.0~100.0%)
 * @return 0=성공, -1=실패 (timeout 포함)
 */
void Fan6_SetDuty_Safe(uint8_t ch0_5, float pct);

#ifdef __cplusplus
	}
#endif
	
#endif /* EMC2303_H */
