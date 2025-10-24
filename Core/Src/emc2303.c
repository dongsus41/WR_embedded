/*
 * EMC2303_6ch_Fan.c — Single‑file starter driver + example for two EMC2303 (6 channels)
 * MCU: STM32 (H5/H7/…); Bus: I2C/SMBus @100 kHz
 *
 * Wiring assumed
 *  - Two EMC2303: ADDR_SEL resistors -> IC1: 4.7k (addr 0x2E), IC2: 10k (addr 0x2C)
 *  - SMDATA/SMCLK: 3.3 V with pull‑ups (e.g., 2.2–4.7 kΩ); I2C open‑drain
 *  - OPTION A (권장, 4‑wire 팬): EMC2303 PWMx -> Fan PWM pin (open‑drain), pull‑up to 5 V; TACHx -> Fan tach
 *  - OPTION B (공급 전원 게이팅): EMC2303 PWMx -> TPS27081A ON/OFF (push‑pull). 주파수는 저주파(≈500 Hz) 사용 권장
 *
 * Notes
 *  - This file uses STM32 HAL I2C. The I2C handle is taken from extern: hi2c4.
 *  - EMC2303 has per‑channel registers spaced by 0x10. We use base+offset helpers.
 *  - Two control modes
 *      • Direct PWM duty (open‑loop): write FANx_SETTING (0..255) → duty 0..100%.
 *      • Closed‑loop RPM: write TACH TARGET (FxTT) + set ENAGx.
 *  - Watchdog: after power‑up, writing any FANx_SETTING or enabling ENAGx disables the initial watchdog.
 */

//#include <string.h>
#include "stm32h7xx_hal.h"
#include "debug.h"

#include "emc2303.h"

/* ===== Per‑channel base address and offsets ===== */
#define OFF_FAN_SETTING            0x00  // FxSP — direct PWM 0..255
#define OFF_PWM_DIVIDE             0x01  // PMxD — output freq divider (0->1)
#define OFF_FAN_CFG1               0x02  // ENAGx, RNGx, EDGEx, UPDATE
#define OFF_FAN_CFG2               0x03
#define OFF_GAIN                   0x05
#define OFF_SPINUP_CFG             0x06
#define OFF_MAX_STEP               0x07
#define OFF_MIN_DRIVE              0x08  // 0..255 (default 0x66 ~40%)
#define OFF_VALID_TACH             0x09
#define OFF_DRVFAIL_BAND_LO        0x0A
#define OFF_DRVFAIL_BAND_HI        0x0B
#define OFF_TACH_TARGET_LO         0x0C  // FxTT low, write high last to latch
#define OFF_TACH_TARGET_HI         0x0D
#define OFF_TACH_READING_HI        0x0E
#define OFF_TACH_READING_LO        0x0F

/* ===== Example: bring‑up two EMC2303 for 6 channels ===== */
/* Map logical channels 1..6 → (addr, ch) */

static const emc_map_t LCH_MAP[6] =
{
    {EMC2303_ADDR_IC1, 0u},
    {EMC2303_ADDR_IC1, 1u},
    {EMC2303_ADDR_IC1, 2u},
    {EMC2303_ADDR_IC2, 0u},
    {EMC2303_ADDR_IC2, 1u},
    {EMC2303_ADDR_IC2, 2u}
};

static inline uint32_t emc_pwm_base_hz(emc_pwm_base_t b)
{
    switch (b)
    {
        case EMC_PWM_BASE_26k:
            return 26000u;
        case EMC_PWM_BASE_19k5:
            return 19531u;
        case EMC_PWM_BASE_4k882:
            return 4882u;
        default:
            return 2441u;
    }
}

static inline uint8_t emc2303_ch_base(uint8_t ch0_2)
{
	return (uint8_t)(0x30u + 0x10u * ch0_2);
}

/* ===== Low‑level I2C helpers (use extern hi2c4) ===== */
static void emc_w8(uint8_t addr7, uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c4, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_I2C_TIMEOUT_MS) != HAL_OK)
        {REPORT_ERROR();}
}

static void emc_r8(uint8_t addr7, uint8_t reg, uint8_t *val)
{
    if (HAL_I2C_Mem_Read(&hi2c4, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT, val, 1, HAL_I2C_TIMEOUT_MS) != HAL_OK)
        {REPORT_ERROR();}
}

/* ===== Bitfield helpers ===== */
static uint8_t emc_mask_lsb_shift(uint8_t m)
{
	uint8_t s;
	uint8_t t;

	if (m == 0u)
	{
		REPORT_ERROR();
		return 0u;
	}

	s = 0u;
	t = m;

	while ((t & 1u) == 0u)
	{
		t = (uint8_t)(t >> 1);
		s = (uint8_t)(s + 1u);
	}
	return s;
}

static void emc_upd_bits(uint8_t a, uint8_t r, uint8_t m, uint8_t v)
{
	uint8_t x;
	uint8_t s;
	uint8_t vs;


	if (m == 0u)
	{
		REPORT_ERROR();
		return;
	}
	
	emc_r8(a, r, &x);
	s = emc_mask_lsb_shift(m);
	vs = (uint8_t)(((uint8_t)(v << s)) & m);
	x = (uint8_t)((x & (uint8_t)(~m)) | vs);
	emc_w8(a, r, x);
}

/* ===== Set PWM base frequency per channel (affects only base, divider set separately) ===== */
static void emc_set_base(uint8_t a, uint8_t ch0_2, emc_pwm_base_t base)
{
    if (ch0_2 > 2u)
    {
        REPORT_ERROR();
        return;
    }

    uint8_t reg;
    uint8_t mask;

	reg = REG_PWM_BASEF_123;
	mask = (uint8_t)(0x3u << (uint8_t)(ch0_2 * 2u));

	/* note: emc_upd_bits will shift 'base' into position using mask */
	emc_upd_bits(a, reg, mask, (uint8_t)base);
}

/* ===== Initialize one EMC2303 with a profile ===== */
void EMC2303_Init(uint8_t addr7, emc_profile_t profile, uint32_t target_pwm_hz)
{
    uint8_t ch;

    /* Disable RPM algorithm (Direct Setting), keep defaults for now. */
	for (ch = 0u; ch <= 2u; ch++) /* [MOD] 0..2 */
	{
		uint8_t base = emc2303_ch_base(ch);		
		/* Clear ENAGx bit in Fan Config1 (bit7) */
		emc_upd_bits(addr7, (uint8_t)(base + OFF_FAN_CFG1), 0x80u, 0u);		
		/* Write an initial setting to defeat the power‑up watchdog */
		emc_w8(addr7, (uint8_t)(base + OFF_FAN_SETTING), 0x00u);
	}

    if (profile == EMC_PROFILE_4WIRE_FAN)
    {
        /* Open‑drain PWM, normal polarity, base ≈26 kHz, divider = 1 */
//        emc_upd_bits(addr7, REG_PWM_OUTPUT_CFG, 0x07u, 0x00u); /* PMOT1..3 = 0 (open‑drain) */
        emc_upd_bits(addr7, REG_PWM_OUTPUT_CFG, 0x07u, 0x07u); /* PMOT1..3 = 1 (push‑pull) */
        emc_upd_bits(addr7, REG_PWM_POLARITY,   0x07u, 0x00u); /* PLRITY1..3 = 0 (00h→0%) */

        for (ch = 0u; ch <= 2u; ++ch)
        {
            emc_set_base(addr7, ch, EMC_PWM_BASE_26k);

            uint8_t base = emc2303_ch_base(ch);
            emc_w8(addr7, (uint8_t)(base + OFF_PWM_DIVIDE), 1u); /* ÷1 */
        }
    }
    else /* EMC_PROFILE_POWER_GATING */
    {
        uint32_t base_hz;
        uint32_t div32;
        uint8_t  div8;

        /* Push‑pull PWM into TPS ON/OFF, normal polarity */
        emc_upd_bits(addr7, REG_PWM_OUTPUT_CFG, 0x07u, 0x07u); /* PMOT1..3 = 1 (push‑pull) */
        emc_upd_bits(addr7, REG_PWM_POLARITY,   0x07u, 0x00u); /* PLRITY1..3 = 0 */

        /* Use base = 4.882 kHz and compute divider to approximate target_pwm_hz (default 500 Hz) */
        if (target_pwm_hz == 0u)
            target_pwm_hz = 500u;

        base_hz = emc_pwm_base_hz(EMC_PWM_BASE_4k882);

        for (ch = 0u; ch <= 2u; ++ch)
        {
            emc_set_base(addr7, ch, EMC_PWM_BASE_4k882);

            /* divider rounding: div = round(base / target) ; clamp to [1,255] ; 0 encodes as 1 */
            div32 = (uint32_t)((base_hz + (target_pwm_hz / 2u)) / target_pwm_hz);

            if (div32 == 0u)
                div32 = 1u;
            if (div32 > 255u)
                div32 = 255u;

            div8 = (uint8_t)div32;

            uint8_t base = emc2303_ch_base(ch);
            emc_w8(addr7, (uint8_t)(base + OFF_PWM_DIVIDE), div8);
        }
    }
}

/* ===== Duty set in percent (0.0..100.0) ===== */
void EMC2303_SetDutyPct(uint8_t addr7, uint8_t ch0_2, float pct)
{
    uint8_t code;
    uint8_t reg;

    if (ch0_2 > 2u)
    {
        REPORT_ERROR();
        return;
    }

    if (pct < 0.0f)
        pct = 0.0f;
    else if (pct > 100.0f)
        pct = 100.0f;

    code = (uint8_t)((pct * 255.0f + 50.0f) / 100.0f);
    reg = (uint8_t)(emc2303_ch_base(ch0_2) + OFF_FAN_SETTING);
    emc_w8(addr7, reg, code);
}

/* ===== Enable closed‑loop RPM control on channel ch ===== */
void EMC2303_EnableClosedLoopRPM(uint8_t addr7, uint8_t ch0_2, uint32_t rpm)
{
    uint8_t base;
    uint32_t cnt32;
    uint16_t cnt16;

    if (ch0_2 > 2u)
    {
        REPORT_ERROR();
        return;
    }

    base = emc2303_ch_base(ch0_2);

    if (rpm == 0u)
        cnt32 = 0xFFFFu;
    else
    {
        cnt32 = (uint32_t)(3932160u / rpm);
        if (cnt32 > 0x1FFFu)
            cnt32 = 0x1FFFu;
    }

    cnt16 = (uint16_t)cnt32;

    /* Write target low first, then high to latch */
    emc_w8(addr7, (uint8_t)(base + OFF_TACH_TARGET_LO), (uint8_t)(cnt16 & 0xFFu));
    emc_w8(addr7, (uint8_t)(base + OFF_TACH_TARGET_HI), (uint8_t)((cnt16 >> 8) & 0x1Fu));

    /* Enable algorithm ENAGx (bit7) */
    emc_upd_bits(addr7, (uint8_t)(base + OFF_FAN_CFG1), 0x80u, 1u);
}

/* ===== Read current RPM (requires valid tach wiring/clock) ===== */
void EMC2303_ReadRPM(uint8_t addr7, uint8_t ch0_2, uint32_t *out_rpm)
{
    uint8_t base;
    uint8_t hi;
    uint8_t lo;
    uint16_t cnt;

    if (out_rpm == NULL)
    {
        REPORT_ERROR();
        return;
    }

    if (ch0_2 > 2u)
    {
        REPORT_ERROR();
        *out_rpm = 0u;
        return;
    }

    base = emc2303_ch_base(ch0_2);

    emc_r8(addr7, (uint8_t)(base + OFF_TACH_READING_HI), &hi);
    emc_r8(addr7, (uint8_t)(base + OFF_TACH_READING_LO), &lo);

    cnt = (uint16_t)(((uint16_t)(hi & 0x1Fu) << 8) | (uint16_t)lo);

    if (cnt == 0u)
    {
        *out_rpm = 0u;
        return;
    }

    *out_rpm = (uint32_t)(3932160u / cnt);
}

void Fans6_Init(emc_profile_t profile, uint32_t target_pwm_hz)
{
    EMC2303_Init(EMC2303_ADDR_IC1, profile, target_pwm_hz);
    EMC2303_Init(EMC2303_ADDR_IC2, profile, target_pwm_hz);
}

void Fan6_SetDuty(uint8_t ch0_5, float pct)
{
    /* [MOD] 0-based logical index: valid range 0..5 */
    if (ch0_5 > 5u)
    {
        REPORT_ERROR();
        return;
    }
    EMC2303_SetDutyPct(LCH_MAP[ch0_5].addr, LCH_MAP[ch0_5].ch, pct);
}

void Fan6_EnableRPM(uint8_t ch0_5, uint32_t rpm)
{
    /* [MOD] 0-based logical index: valid range 0..5 */
    if (ch0_5 > 5u)
    {
        REPORT_ERROR();
        return;
    }
    EMC2303_EnableClosedLoopRPM(LCH_MAP[ch0_5].addr, LCH_MAP[ch0_5].ch, rpm);
}

void Fan6_ReadRPM(uint8_t ch0_5, uint32_t *rpm)
{
    /* [MOD] 0-based logical index: valid range 0..5 */
    if (ch0_5 > 5u)
    {
        REPORT_ERROR();
        return;
    }
    EMC2303_ReadRPM(LCH_MAP[ch0_5].addr, LCH_MAP[ch0_5].ch, rpm);
}


/* ===== Minimal usage example =====

// In your project:
// extern I2C_HandleTypeDef hi2c4; // configured to 100 kHz in CubeMX

void Fans_Bringup(void)
{
    emc_profile_t profile;
    uint32_t target_hz;

    // Choose profile depending on wiring
    // profile = EMC_PROFILE_4WIRE_FAN;                 // EMC PWM -> fan PWM pin
    profile = EMC_PROFILE_POWER_GATING;                 // EMC PWM -> TPS27081A ON/OFF
    target_hz = 500u;                                   // desired PWM frequency when power‑gating

    Fans6_Init(profile, target_hz);

    // Open‑loop example: set all 6 channels to 40% duty
    // for (uint8_t i = 1u; i <= 6u; ++i) { Fan6_SetDuty(i, 40.0f); }

    // Closed‑loop example (requires tach wiring/clock):
    // Fan6_EnableRPM(1u, 1800u); // 1800 RPM target on logical channel #1
}*/
