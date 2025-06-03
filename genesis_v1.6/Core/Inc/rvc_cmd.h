#ifndef RVC_CMD_H
#define RVC_CMD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "rvc_defs.h"      // for RVC_DGN_DC_LOAD_COMMAND, etc.

/*============================================================================
 * RV-C Message Structure
 *============================================================================*/
typedef struct {
    uint8_t priority;        // 0..7
    uint8_t dgn_high;        // (DGN >> 8) & 0xFF
    uint8_t dgn_low;         // (DGN & 0xFF)  OR  DA
    uint8_t source_address;  // 0..255
    uint8_t data[8];         // always 8 bytes
    uint8_t data_length;     // always 8 for RV-C
} rvc_message_t;

/*============================================================================
 * RV-C ID / DGN Helpers
 *============================================================================*/
#define RVC_GET_PRIORITY(can_id)    (((can_id) >> 26) & 0x07)
#define RVC_GET_DGN_HIGH(can_id)    (((can_id) >> 16) & 0xFF)
#define RVC_GET_DGN_LOW(can_id)     (((can_id) >>  8) & 0xFF)
#define RVC_GET_SOURCE_ADDR(can_id) ((can_id)        & 0xFF)

static inline uint32_t rvc_build_can_id(uint8_t  priority,
                                        uint8_t  dgn_high,
                                        uint8_t  dgn_low,
                                        uint8_t  source_addr)
{
    uint32_t can_id = 0;
    can_id |= ((uint32_t)(priority & 0x07) << 26);
    can_id |= (0UL << 25);             // reserved=0
    can_id |= ((uint32_t)dgn_high << 16);
    can_id |= ((uint32_t)dgn_low  <<  8);
    can_id |=  (uint32_t)source_addr;
    return can_id;
}

/*============================================================================
 * Public Prototypes
 *============================================================================*/

/**
 * @brief Initialize RV-C command handler (filters, interrupts, PWM, etc.)
 */
void RVC_Init(void);

/**
 * @brief Get stats (# of messages processed, # of errors)
 */
void RVC_GetStats(uint32_t *messages, uint32_t *errors);

/**
 * @brief Get current state (ON/OFF, PWM%) of one channel
 */
bool RVC_GetChannelState(uint8_t channel, bool *on_off, uint8_t *pwm_percent);

/**
 * @brief RX FIFO0 callback — this is called by HAL when a new FDCAN frame arrives.
 * @param hfdcan pointer to the FDCAN handle (should be &hfdcan1 in main.c)
 */
void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Handle a Load-Command DGN (0x1FFBC) — called internally
 */
void RVC_HandleLoadCommand(uint16_t dgn, uint8_t *data, uint8_t length);

/**
 * @brief Handle a Dimmer-Command DGN (0x1FFB9) — called internally
 */
void RVC_HandleDimmerCommand(uint16_t dgn, uint8_t *data, uint8_t length);

/**
 * @brief Send a “Diagnostic (DM_RV)” message to the bus (DGN=0x1FECA)
 */
bool rvc_send_diagnostic(uint8_t operating_status, uint8_t lamp_status);

/**
 * @brief Send any RV-C message (builds 29-bit ID + TX)
 */
bool rvc_send_message(const rvc_message_t *msg);

#endif // RVC_CMD_H
