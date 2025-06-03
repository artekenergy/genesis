/**
 * @file    rvc_defs.h
 * @brief   RV-C DGN (DGN) definitions, priority/SA/DA macros, and payload field masks.
 *
 * All DGN numbers, priority values, and helper macros for building/parsing
 * 29-bit RV-C identifiers are consolidated here. Include this file wherever
 * you send or receive RV-C messages.
 */

#ifndef RVC_DEFS_H
#define RVC_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 1) Node and Priority Definitions
 *───────────────────────────────────────────────────────────────────────────────
 */

// Your device’s “Source Address” (8 bits). Change as needed for each node.
#define RVC_SRC_ADDR           0x25U

// Standard RV-C priority (3 bits). Lower values = higher priority; 0..7.
#define RVC_PRIORITY           6U

// Broadcast Destination Address (DA = 0xFF implies “global”).
#define RVC_DA_BROADCAST       0xFFU

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 2) DGN (DGN) Definitions (18-bit DGN values in hex)
 *───────────────────────────────────────────────────────────────────────────────
 *   These come straight from the RV-C spec (e.g. “RV-C DGN Tables”).
 */

#define DGN_LOAD_COMMAND        0x1FFBCU   // “Load Command”
#define DGN_LOAD_STATUS         0x1FFBDU   // “Load Status” (not used right now)
#define DGN_DIMMER_COMMAND      0x1FFB9U   // “Dimmer Command”
#define DGN_DIMMER_STATUS       0x1FFBBU   // “Dimmer Status” (not used right now)

// Information Request: “send me DGN = ZZZZ now”
#define DGN_INFO_REQUEST        0x0EA00U   // DGN EA00h

// Acknowledgment: node responds “I got your command”
#define DGN_ACK                 0x0E800U   // DGN E800h

// Diagnostic “No-Fault” (DM_RV): keepalive if no other status sent in 5 s
#define DGN_DM_RV               0x1FECAU   // DGN 1FECAh

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 3) Acknowledgment Codes (byte 0 of ACK payload)
 *───────────────────────────────────────────────────────────────────────────────
 *   Per “Acknowledgment” DGN spec (Table 3.2.4.4.b), byte 0 = ACK code:
 */
#define ACK_CODE_POSITIVE       0x00U      // “Command succeeded”
#define ACK_CODE_STANDBY        0x01U      // “Node in Standby, try later”
#define ACK_CODE_NOT_SUPP       0x02U      // “Function not supported”
#define ACK_CODE_BAD_PARAM      0x03U      // “Invalid parameter”
#define ACK_CODE_FAULT          0x04U      // “Command failed due to fault”
#define ACK_CODE_NAK            0x80U      // “Generic NAK / failure”
// (See spec for other NAK values 0x81…0xFE)

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 4) Helper Macros for Building 29-bit Identifier (ID)
 *───────────────────────────────────────────────────────────────────────────────
 *
 * A 29-bit Extended ID is laid out as:
 *   [  3 bits priority | 1 reserved | 1 DP | 8 bits PF | 8 bits PS | 8 bits SA ]
 *
 * For RV-C, we set “reserved” and “DP” = 0. The 18-bit DGN = (PF << 8) | PS.
 * Destination Address = PS for command/status (or 0xFF for broadcast).
 * Source Address = SA (bits [7:0]).
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │      3b Pri  │ 1b rsv │ 1b DP │ 8b PF │ 8b PS (DA or LSB of DGN) │ 8b SA │
 *  │  (bits 28..26) (25)   (24)  (23..16)  (15..8)                (7..0)  │
 *  └─────────────────────────────────────────────────────────────────────────┘
 */

// Build a full 29-bit ID given 18-bit DGN, 8-bit DA (destination), and 8-bit SA (source).
#define RVC_ID(DGN, dest, src) \
    ( (uint32_t)((RVC_PRIORITY & 0x07U) << 26)           /* priority: bits 28..26 */ \
    |  (uint32_t)(( (DGN) & 0x3FFFFU ) << 8)             /* DGN: bits 25..8        */ \
    |  (uint32_t)((dest) & 0xFFU)                        /* DA:  bits 7..0 as PS   */ \
    |    (uint32_t)((src)  & 0xFFU) )                    /* SA also in bits 7..0   */

// Extract the 18-bit DGN from a 29-bit ID
#define RVC_GET_DGN(extended_id) \
    ( ((uint32_t)(extended_id) >> 8) & 0x3FFFFU )

// Extract the Source Address (SA) from a 29-bit ID (bits 7..0)
#define RVC_GET_SA(extended_id) \
    ( (uint8_t)((extended_id) & 0xFFU) )

// Extract the Destination Address (DA) from a 29-bit ID (bits 15..8)
#define RVC_GET_DA(extended_id) \
    ( (uint8_t)(((extended_id) >> 8) & 0xFFU) )

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 5) Payload Field Masks / Offsets
 *───────────────────────────────────────────────────────────────────────────────
 *   Defines for which byte in the 8-byte payload holds which parameter.
 */

// For Load Command (DGN_LOAD_COMMAND):
//   Byte 0 = channel index (0..3), Byte 1 = on/off (0=OFF, 1=ON), Bytes 2..7 = 0
#define LOADCMD_CH_INDEX_BYTE    0
#define LOADCMD_ONOFF_BYTE       1

// For Load Status (DGN_LOAD_STATUS):
//   Byte 0 = bitmask (bit0→CH0, bit1→CH1, bit2→CH2, bit3→CH3), Bytes 1..7 = 0
#define LOADSTAT_MASK_BYTE       0

// For Dimmer Command (DGN_DIMMER_COMMAND):
//   Byte 0 = LED channel (0 or 1), Byte 1 = brightness (0..100), Bytes 2..7 = 0
#define DIMCMD_CH_INDEX_BYTE     0
#define DIMCMD_PERCENT_BYTE      1

// For Dimmer Status (DGN_DIMMER_STATUS):
//   Byte 0 = LED channel (0 or 1), Byte 1 = current brightness (0..100)
#define DIMSTAT_CH_INDEX_BYTE    0
#define DIMSTAT_PERCENT_BYTE     1

// For Information Request (DGN_INFO_REQUEST):
//   Bytes 0..2 = “desired DGN” (LSB first), Byte 3 = instance (0..253, or 0xFF=all), Bytes 4..7 = 0
#define INFREQ_DGN_LSB_BYTE      0
#define INFREQ_DGN_MID_BYTE      1
#define INFREQ_DGN_MSB_BYTE      2
#define INFREQ_INSTANCE_BYTE     3

// For Acknowledgment (DGN_ACK):
//   Byte 0 = acknowledgment code (0=ACK, nonzero=NAK), Byte 1 = instance (0xFF if none),
//   Byte 2 = instance bank + reserved, Byte 3 = reserved, Byte 4 = “SA being acknowledged”,
//   Bytes 5..7 = acknowledged DGN (LSB first).
#define ACK_CODE_BYTE            0
#define ACK_INSTANCE_BYTE        1
#define ACK_BANK_BYTE            2
#define ACK_RESERVED1_BYTE       3
#define ACK_SOURCEACK_BYTE       4  // The SA of the command being ACKed
#define ACK_DGN_LSB_BYTE         5
#define ACK_DGN_MID_BYTE         6
#define ACK_DGN_MSB_BYTE         7

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 6) Utility Macros for Packing/Unpacking 24-bit DGNs in Payload
 *───────────────────────────────────────────────────────────────────────────────
 */

// Place a 24-bit DGN into payload bytes [base], [base+1], [base+2], LSB first:
#define RVC_WRITE_DGN_TO_PAYLOAD(buf, base, DGN) \
    do { \
        (buf)[(base) + 0] = (uint8_t)((DGN)        & 0xFFU); \
        (buf)[(base) + 1] = (uint8_t)(((DGN) >>  8) & 0xFFU); \
        (buf)[(base) + 2] = (uint8_t)(((DGN) >> 16) & 0xFFU); \
    } while (0)

// Read a 24-bit DGN from payload bytes [base], [base+1], [base+2]:
#define RVC_READ_DGN_FROM_PAYLOAD(buf, base) \
    ( (uint32_t)((buf)[(base) + 0])           \
    | ((uint32_t)((buf)[(base) + 1]) <<  8)    \
    | ((uint32_t)((buf)[(base) + 2]) << 16) )

/**
 *───────────────────────────────────────────────────────────────────────────────
 * 7) Convenience Macros for Common DA Values
 *───────────────────────────────────────────────────────────────────────────────
 */

// If you ever send a “command” to yourself (e.g. loopback), you can #define:
#define RVC_DA_SELF             (RVC_SRC_ADDR)

// If you often broadcast “all instances” in an Info-Request, use:
#define RVC_INSTANCE_ALL        0xFFU

#ifdef __cplusplus
}
#endif

#endif /* RVC_DEFS_H */
