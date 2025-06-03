// vnq7003.h - Unified VNQ7003SY Driver Header
#ifndef VNQ7003_H
#define VNQ7003_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"
#include "vnq7003sys_regs.h"

/*============================================================================
 * VNQ7003SY Status Enumeration
 *============================================================================*/
typedef enum {
    VNQ_STATUS_HEALTHY = 0,     // Normal operation
    VNQ_STATUS_ERROR,           // Device error (overtemp, etc.)
    VNQ_STATUS_FAILSAFE,        // Device in fail-safe mode
    VNQ_STATUS_COMM_ERROR       // SPI communication error
} vnq_status_t;

// Alias for compatibility with both naming conventions
typedef vnq_status_t vnq7003_status_t;

// Status constants for compatibility
#define VNQ7003_STATUS_HEALTHY      VNQ_STATUS_HEALTHY
#define VNQ7003_STATUS_ERROR        VNQ_STATUS_ERROR
#define VNQ7003_STATUS_FAILSAFE     VNQ_STATUS_FAILSAFE
#define VNQ7003_STATUS_COMM_ERROR   VNQ_STATUS_COMM_ERROR

/*============================================================================
 * Core VNQ7003 Functions
 *============================================================================*/

/**
 * @brief Initialize VNQ7003 driver
 * - Enters Normal mode via 2-frame unlock sequence
 * - Turns off all outputs initially
 * - Starts internal watchdog servicing
 * @return true if successful, false on failure
 */
bool vnq_init(void);

/**
 * @brief Service VNQ7003 watchdog (call every 5ms max, 15ms absolute max)
 * If you exceed ~25ms between calls, VNQ goes into fail-safe mode
 */
void vnq_task(void);

/**
 * @brief Set individual output channel ON/OFF
 * @param channel Channel number (0-3)
 * @param enable true to turn ON, false to turn OFF
 * @return true if successful, false if invalid channel or not initialized
 */
bool vnq_set_channel(uint8_t channel, bool enable);

/**
 * @brief Set all 4 outputs at once using bitmask
 * @param channel_mask Bitmask (bit0=CH0, bit1=CH1, bit2=CH2, bit3=CH3)
 * @return true if successful, false if not initialized
 */
bool vnq_set_outputs(uint8_t channel_mask);

/**
 * @brief Get VNQ7003 device status
 * @return Current device status (healthy, error, fail-safe, comm error)
 */
vnq_status_t vnq_get_status(void);

/**
 * @brief Emergency stop - immediately turn off all outputs
 * Forces re-initialization on next vnq_init() call
 */
void vnq_emergency_stop(void);

/**
 * @brief Check if VNQ7003 driver is initialized
 * @return true if vnq_init() was called successfully
 */
bool vnq_is_initialized(void);

/**
 * @brief Get watchdog statistics
 * @param services Pointer to store total successful services (can be NULL)
 * @param failures Pointer to store total failures (can be NULL)
 * @param consecutive Pointer to store current consecutive failures (can be NULL)
 */
void vnq_get_watchdog_stats(uint32_t *services, uint32_t *failures, uint32_t *consecutive);

/*============================================================================
 * Enhanced Direct Input Functions (for RV-C integration)
 *============================================================================*/

/**
 * @brief Initialize VNQ7003 for Direct Input control
 * This is an enhanced initialization that:
 * - Calls vnq_init() for basic setup
 * - Enables direct input control on all channels
 * - Configures GPIO pins (PB3-PB6) for direct input control
 * - Starts the watchdog timer (TIM2)
 * - Sets up pull-ups and LED mode
 * @return true if successful, false on failure
 */
bool VNQ7003_Init_For_DI_Control(void);

/**
 * @brief Control a specific channel via direct input pin
 * @param channel Channel number (0-3)
 * @param enable true to turn load ON, false to turn OFF
 * @return true if successful, false if invalid channel
 *
 * Pin mapping:
 * - Channel 0 -> PB3 (IN0)
 * - Channel 1 -> PB4 (IN1)
 * - Channel 2 -> PB5 (IN2)
 * - Channel 3 -> PB6 (IN3)
 */
bool VNQ7003_SetChannel_DI(uint8_t channel, bool enable);

/**
 * @brief Toggle watchdog bit (called automatically by timer callback)
 */
void VNQ7003_ToggleWatchdog(void);

/**
 * @brief Get device status (alias for vnq_get_status)
 * @return Current device status
 */
vnq7003_status_t VNQ7003_GetStatus(void);

/*============================================================================
 * Low-Level SPI Functions (for advanced users)
 *============================================================================*/

/**
 * @brief Write to VNQ7003 register via SPI
 * @param regAddr Register address (0x00-0x3F)
 * @param data Data byte to write
 * @return true if successful, false if SPI error
 */
bool VNQ7003_SPI_WriteReg(uint8_t regAddr, uint8_t data);

/**
 * @brief Read from VNQ7003 register via SPI
 * @param regAddr Register address (0x00-0x3F)
 * @param data Pointer to store read data (can be NULL)
 * @param gsb Pointer to store Global Status Byte (can be NULL)
 * @return true if successful, false if SPI error
 */
bool VNQ7003_SPI_ReadReg(uint8_t regAddr, uint8_t *data, uint8_t *gsb);

/*============================================================================
 * Debug and Diagnostic Functions
 *============================================================================*/

/**
 * @brief Print comprehensive VNQ7003 status to console
 * Reads and displays CTRL, CONFIG, SOCR registers plus GSB analysis
 */
void vnq_print_simple_status(void);

/*============================================================================
 * HAL Callback Function
 *============================================================================*/

/**
 * @brief HAL Timer callback for watchdog servicing
 * @param htim Timer handle (checks for TIM2)
 * This function is called automatically by the HAL when TIM2 expires
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif // VNQ7003_H
