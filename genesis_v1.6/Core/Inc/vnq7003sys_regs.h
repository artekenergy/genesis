/**
 * @file    vnq7003_regs.h
 * @brief   Register map and bit‐field definitions for the VNQ7003SY quad‐channel high‐side driver.
 *
 * All register addresses and bit masks are taken from the VNQ7003SY datasheet (DS11059 Rev 8, Dec 2018).
 * This header defines every RAM and ROM register address the MCU might read or write over SPI, plus
 * the associated bit‐mask constants for each field.
 *
 * Usage:
 *   #include "vnq7003_regs.h"
 *
 *   // Example: read the Control Register (CTLR), then set the “Go to Standby” bit:
 *   uint8_t ctlr = VNQ7003_ReadReg(VNQ7003_REG_CTLR);
 *   ctlr |= VNQ7003_CTLR_GOSTBY;
 *   VNQ7003_WriteReg(VNQ7003_REG_CTLR, ctlr);
 *
 * Note: All addresses and bit positions are as documented in:
 *       Section 4.3 “Register map” (pp. 33–35) and Section 4.6 “Control registers” (pp. 40–44) of
 *       VNQ7003SY datasheet DS11059 Rev 8 (December 2018) .
 */

#ifndef VNQ7003_REGS_H
#define VNQ7003_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/***────────────────────────────────────────────────────────────────────────***/
/**                            RAM (volatile) Registers                     **/
/***────────────────────────────────────────────────────────────────────────***/

/* 0x00: Control Register (CTLR) – device enable, standby, protected init            */
#define VNQ7003_REG_CTLR           0x00U
/* CTLR Bits:
 *   Bit 7: – (Reserved)
 *   Bit 6: – (Reserved)
 *   Bit 5: GOSTBY – “Go to Standby”
 *   Bit 4: UNLOCK – unlock bit (must be set before setting GOSTBY or EN)
 *   Bits 3–0: – (Reserved)
 */
#define VNQ7003_CTLR_GOSTBY        (1U << 5)    /* 1 = enter Standby mode (requires UNLOCK=1) */
#define VNQ7003_CTLR_UNLOCK        (1U << 4)    /* 1 = unlock bit (must precede GOSTBY or EN) */
#define VNQ7003_CTLR_EN            (1U << 0)   /* Enable bit (Normal mode) */


/* 0x01: Direct Input Enable Control Register (DIENCR) – enable DIₓ pins for each channel */
#define VNQ7003_REG_DIENCR         0x01U
/* DIENCR Bits:
 *   Bit 7–4: – (Reserved)
 *   Bit 3: DIENCR3 – direct‐input enable for channel 3 (1 = IN₃ controls OUT₃)
 *   Bit 2: DIENCR2 – direct‐input enable for channel 2 (1 = IN₂ controls OUT₂)
 *   Bit 1: DIENCR1 – direct‐input enable for channel 1 (1 = IN₁ controls OUT₁)
 *   Bit 0: DIENCR0 – direct‐input enable for channel 0 (1 = IN₀ controls OUT₀)
 */
#define VNQ7003_DIENCR_CH0         (1U << 0)
#define VNQ7003_DIENCR_CH1         (1U << 1)
#define VNQ7003_DIENCR_CH2         (1U << 2)
#define VNQ7003_DIENCR_CH3         (1U << 3)

/* 0x02: Open‐load OFF‐State Control Register (OLOFFCR) – enable pull‐up for OFF‐state detection */
#define VNQ7003_REG_OLOFFCR        0x02U
/* OLOFFCR Bits:
 *   Bit 7–4: – (Reserved)
 *   Bit 3: OLOFFCR3 – enable pull‐up on channel 3 (1 = enable)
 *   Bit 2: OLOFFCR2 – enable pull‐up on channel 2 (1 = enable)
 *   Bit 1: OLOFFCR1 – enable pull‐up on channel 1 (1 = enable)
 *   Bit 0: OLOFFCR0 – enable pull‐up on channel 0 (1 = enable)
 */
#define VNQ7003_OLOFFCR_CH0        (1U << 0)
#define VNQ7003_OLOFFCR_CH1        (1U << 1)
#define VNQ7003_OLOFFCR_CH2        (1U << 2)
#define VNQ7003_OLOFFCR_CH3        (1U << 3)

/* 0x03: Channel Control Register (CCR) – selects BULB vs LED mode for channels 0–1 */
#define VNQ7003_REG_CCR            0x03U
/* CCR Bits:
 *   Bit 7–2: – (Reserved)
 *   Bit 1: CCR1 – mode for channel 1 (1 = LED mode, 0 = BULB mode)
 *   Bit 0: CCR0 – mode for channel 0 (1 = LED mode, 0 = BULB mode)
 */
#define VNQ7003_CCR_MODE_CH0       (1U << 0)
#define VNQ7003_CCR_MODE_CH1       (1U << 1)

/* 0x04: Fast Switching Configuration Register (FASTSWCR) – per‐channel fast/normal switch */
#define VNQ7003_REG_FASTSWCR       0x04U
/* FASTSWCR Bits:
 *   Bit 7–4: – (Reserved)
 *   Bit 3: FASTSWCR3 – channel 3 fast‐switch enable (1 = fast, 0 = normal)
 *   Bit 2: FASTSWCR2 – channel 2 fast‐switch enable
 *   Bit 1: FASTSWCR1 – channel 1 fast‐switch enable
 *   Bit 0: FASTSWCR0 – channel 0 fast‐switch enable
 */
#define VNQ7003_FASTSWCR_CH0       (1U << 0)
#define VNQ7003_FASTSWCR_CH1       (1U << 1)
#define VNQ7003_FASTSWCR_CH2       (1U << 2)
#define VNQ7003_FASTSWCR_CH3       (1U << 3)

/* 0x05: RESERVED (no user‐accessible register)                                                           */

/* 0x06: Current‐Sense Multiplexer Control Register (CSMUXCR) – select which channel’s sense is on I_SENSE */
#define VNQ7003_REG_CSMUXCR        0x06U
/* CSMUXCR Bits:
 *   Bit 7–4: – (Reserved)
 *   Bit 3: MUXEN – current‐sense‐mux enable (1 = output I_SENSE drives selected channel)
 *   Bits 2–0: MUXCH – selects which channel’s CSENSE is output:
 *             0 = CH0, 1 = CH1, 2 = CH2, 3 = CH3 (4..7 = reserved)
 */
#define VNQ7003_CSMUXCR_MUXEN      (1U << 3)
#define VNQ7003_CSMUXCR_MUXCH_MASK (0x07U)

/* 0x07: SPI Output Control Register (SOCR) – directly enable/disable each output while in Normal mode */
#define VNQ7003_REG_SOCR           0x07U
/* SOCR Bits:
 *   Bit 7–4: – (Reserved)
 *   Bit 3: SOCR3 – enables channel 3 output (1 = ON, 0 = OFF)
 *   Bit 2: SOCR2 – enables channel 2 output
 *   Bit 1: SOCR1 – enables channel 1 output
 *   Bit 0: SOCR0 – enables channel 0 output
 */
#define VNQ7003_SOCR_CH0           (1U << 0)
#define VNQ7003_SOCR_CH1           (1U << 1)
#define VNQ7003_SOCR_CH2           (1U << 2)
#define VNQ7003_SOCR_CH3           (1U << 3)

/* 0x08: Channel Latch‐OFF Timer Control Register (CHLOFFTCR0,1) – blanking window for channels 0–1 */
#define VNQ7003_REG_CHLOFFTCR01    0x08U
/* CHLOFFTCR0,1 Bits:
 *   Bits 3–0: CHLOFFTCR0x – 4-bit blanking code for channel 0 (0x0 = latch-off, 0x1..0xF = 17 ms..255 ms)
 *   Bits 7–4: CHLOFFTCR1x – 4-bit blanking code for channel 1 (same encoding)
 */
#define VNQ7003_CHLOFFTCR_CH0_MASK (0x0FU)
#define VNQ7003_CHLOFFTCR_CH1_MASK (0xF0U)

/* 0x09: Channel Latch‐OFF Timer Control Register (CHLOFFTCR2,3) – blanking window for channels 2–3 */
#define VNQ7003_REG_CHLOFFTCR23    0x09U
/* CHLOFFTCR2,3 Bits:
 *   Bits 3–0: CHLOFFTCR2x – 4-bit blanking code for channel 2
 *   Bits 7–4: CHLOFFTCR3x – 4-bit blanking code for channel 3
 */
#define VNQ7003_CHLOFFTCR_CH2_MASK (0x0FU)
#define VNQ7003_CHLOFFTCR_CH3_MASK (0xF0U)


/***────────────────────────────────────────────────────────────────────────***/
/**                            ROM (non‐volatile) Registers                  **/
/***────────────────────────────────────────────────────────────────────────***/

/* 0x00 – 0x04: ROM: Device Identification (read‐only)
 *   The first five bytes in ROM may be read to confirm that we have a genuine VNQ7003SY:
 *   0x00: Company Code   (should read 0x00h)
 *   0x01: Device Family  (should read 0x01h)
 *   0x02: Product Code 1 (should read 0x56h)
 *   0x03: Product Code 2 (should read 0x48h)
 *   0x04: Product Code 3 (should read 0x31h)
 */
#define VNQ7003_REG_COMPANY_CODE   0x00U  /* “STM” = 0x00h  (ROM)         */
#define VNQ7003_REG_DEVICE_FAMILY  0x01U  /* Product family = 0x01h (ROM) */
#define VNQ7003_REG_PRODUCT_CODE1  0x02U  /* First product code = 0x56h   */
#define VNQ7003_REG_PRODUCT_CODE2  0x03U  /* Second product code = 0x48h  */
#define VNQ7003_REG_PRODUCT_CODE3  0x04U  /* Third product code = 0x31h   */

/* 0x0A: Version register (ROM, read‐only) – silicon revision
 *   Ex: read 0x03h to confirm silicon rev.                                               */
#define VNQ7003_REG_VERSION        0x0AU

/* 0x10: SPI Mode register (ROM, read‐only) – bitfields define SPI config
 *   See “SPI Modes” (Table 22 on p. 37) for bit-layout.
 */
#define VNQ7003_REG_SPI_MODE       0x10U
/* 0x11: Watchdog Type register (ROM, read‐only) – indicates WD type used by device          */
#define VNQ7003_REG_WD_TYPE1       0x11U

/* 0x13: Watchdog Bit Position 1 (ROM, read‐only) – which register holds WD toggle bit        */
#define VNQ7003_REG_WD_POS1        0x13U
/* 0x14: Watchdog Bit Position 2 (ROM, read‐only) – exact bit offset for WD toggle             */
#define VNQ7003_REG_WD_POS2        0x14U

/* 0x20: SPI CPHA/CPOL configuration (ROM, read‐only) – see Table 22 (p. 37)                   */
#define VNQ7003_REG_SPI_CPHA       0x20U

/* 0x2F: Direct Input Status Register (DIENSR, ROM read‐only)
 *   Bits 3..0 reflect IN₃..IN₀ pin states.                                                   */
#define VNQ7003_REG_DIENSR         0x2FU
/* DIENSR Bits:
 *   Bit 3: DIENSR3 – current state of IN₃ pin (1 = HIGH)
 *   Bit 2: DIENSR2 – current state of IN₂ pin
 *   Bit 1: DIENSR1 – current state of IN₁ pin
 *   Bit 0: DIENSR0 – current state of IN₀ pin
 */

/* 0x30: Channel Feedback Status Register (CHFBSR, read‐clear)
 *   Bits 3..0 reflect overload/open‐load status for CH₃..CH₀.
 */
#define VNQ7003_REG_CHFBSR         0x30U
/* CHFBSR Bits:
 *   Bit 3: CHFBSR3 – feedback status CH₃ (overload, open‐load ON, etc.)
 *   Bit 2: CHFBSR2 – feedback status CH₂
 *   Bit 1: CHFBSR1 – feedback status CH₁
 *   Bit 0: CHFBSR0 – feedback status CH₀
 */

/* 0x31: Open‐load OFF‐State / Stuck‐to‐VCC Status Register (STKFLTR, read‐clear)
 *   Bits 3..0 reflect OLOFF or stuck‐to‐VCC condition for CH₃..CH₀.
 */
#define VNQ7003_REG_STKFLTR        0x31U
/* STKFLTR Bits:
 *   Bit 3: STKFLTR3 – channel 3 open‐load or VCC‐stuck fault
 *   Bit 2: STKFLTR2 – channel 2 open‐load or VCC‐stuck fault
 *   Bit 1: STKFLTR1 – channel 1 open‐load or VCC‐stuck fault
 *   Bit 0: STKFLTR0 – channel 0 open‐load or VCC‐stuck fault
 */

/* 0x32: Channels Latch‐OFF Status Register (CHLOFFSR, ROM read‐only)
 *   Bits 3..0 reflect “latched off” fault for CH₃..CH₀.                                      */
#define VNQ7003_REG_CHLOFFSR       0x32U
/* CHLOFFSR Bits:
 *   Bit 3: CHLOFFSR3 – latched‐off state on channel 3 (1 = latched)
 *   Bit 2: CHLOFFSR2 – latched‐off state on channel 2
 *   Bit 1: CHLOFFSR1 – latched‐off state on channel 1
 *   Bit 0: CHLOFFSR0 – latched‐off state on channel 0
 */

/* 0x33: VDS Feedback Status Register (VDSFSR, read‐clear)
 *   Bits 3..0 reflect “high VDS” fault at turn‐off for CH₃..CH₀.                              */
#define VNQ7003_REG_VDSFSR         0x33U
/* VDSFSR Bits:
 *   Bit 3: VDSFSR3 – high‐VDS‐at‐turn‐off on channel 3 (1 = fault)
 *   Bit 2: VDSFSR2 – high‐VDS‐at‐turn‐off on channel 2
 *   Bit 1: VDSFSR1 – high‐VDS‐at‐turn‐off on channel 1
 *   Bit 0: VDSFSR0 – high‐VDS‐at‐turn‐off on channel 0
 */

/* 0x34: Generic Status Register (GENSR, read‐clear)
 *   Bit 7: VCCUV – VCC‐undervoltage warning (real‐time)
 *   Bit 6: RST   – reset warning (set on HW/SW reset, cleared by read‐clear)
 *   Bit 5: SPIE  – SPI‐error warning (set on wrong clock count or SDI stuck, cleared by read‐clear)
 *   Bits 4..0: – (Reserved)
 */
#define VNQ7003_REG_GENSR          0x34U
#define VNQ7003_GENSR_VCCUV        (1U << 7)
#define VNQ7003_GENSR_RST          (1U << 6)
#define VNQ7003_GENSR_SPIE         (1U << 5)

/* 0x3E: GSB Options (ROM read‐only) – options bits for Global Status Byte definition                */
#define VNQ7003_REG_GSB_OPTIONS    0x3EU

/* 0x3F: Configuration Register (CONFIG, R/W) – masks, watchdog toggle bit, fail‐safe selections       */
#define VNQ7003_REG_CONFIG         0x3FU
/* CONFIG Bits:
 *   Bit 7: VDSMASK3 – mask VDS‐fault contribution for CH₃ (1 = masked)
 *   Bit 6: VDSMASK2 – mask VDS‐fault for CH₂
 *   Bit 5: VDSMASK1 – mask VDS‐fault for CH₁
 *   Bit 4: VDSMASK0 – mask VDS‐fault for CH₀
 *   Bit 3: WDTB     – watchdog‐toggle bit (must be toggled in Normal mode to avoid fail‐safe)
 *   Bits 2..0: – (Reserved)
 */
#define VNQ7003_CONFIG_VDSMASK_CH0 (1U << 4)
#define VNQ7003_CONFIG_VDSMASK_CH1 (1U << 5)
#define VNQ7003_CONFIG_VDSMASK_CH2 (1U << 6)
#define VNQ7003_CONFIG_VDSMASK_CH3 (1U << 7)
#define VNQ7003_CONFIG_WDTB        (1U << 3)


/***────────────────────────────────────────────────────────────────────────***/
/**                              SPI “Special” Commands                        **/
/***────────────────────────────────────────────────────────────────────────***/

/* 0xFF (OC1=1, OC0=1, Addr=0x3F = 0b111111)  → SW‐Reset (reset all control registers to defaults) */
#define VNQ7003_CMD_SWRESET        0xFFU

/* 0xBF (OC1=1, OC0=0, Addr=0x3F = 0b111111)  → “Read‐and‐Clear All Status Registers” */
#define VNQ7003_CMD_CLEAR_STATUS   0xBFU


/***────────────────────────────────────────────────────────────────────────***/
/**                              SPI I/O Helpers                              **/
/***────────────────────────────────────────────────────────────────────────***/

/**
 * @brief  Build a 16‐bit SPI “write” transaction for a given 6-bit register address.
 * @param  addr   6-bit register address (0x00..0x3F)
 * @param  data   8-bit data to be written
 * @return 16-bit payload (command byte << 8 | data byte)
 *
 * Format: [ OC1 OC0 | ADDR5..ADDR0 ] [ DATA7..DATA0 ]
 *   OC1=0, OC0=0 → write operation
 */
#define VNQ7003_SPI_MK_WRITE_FRAME(addr, data) \
    (uint16_t)(((uint8_t)(addr) & 0x3FU) << 8 | (uint8_t)(data))

/**
 * @brief  Build a 16-bit SPI “read” transaction for a given 6-bit register address.
 * @param  addr   6-bit register address (0x00..0x3F)
 * @return 16-bit payload (command byte << 8 | 0xFF) – second byte is dummy (0xFF)
 *
 * Format: [ OC1 OC0 | ADDR5..ADDR0 ] [ 0xFF ]
 *   OC1=0, OC0=1 → read operation
 */
#define VNQ7003_SPI_MK_READ_FRAME(addr) \
    (uint16_t)( ((uint8_t)(0x40 | ((addr) & 0x3F))) << 8 | 0xFFU )


#ifdef __cplusplus
}
#endif

#endif /* VNQ7003_REGS_H */
