// vnq7003.c - Unified VNQ7003SY Driver Implementation
#include "vnq7003sys.h"
#include "vnq7003sys_regs.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// External handles
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

/*============================================================================
 * Private Constants
 *============================================================================*/
#define VNQ_SPI_TIMEOUT_MS      10
#define VNQ_WATCHDOG_PERIOD_MS  250

/*============================================================================
 * Private Variables
 *============================================================================*/
static bool vnq_initialized = false;
static bool watchdog_toggle = false;
static uint32_t total_services = 0;
static uint32_t total_failures = 0;
static uint32_t consecutive_failures = 0;

/*============================================================================
 * Private Function Prototypes
 *============================================================================*/
static void vnq_cs_low(void);
static void vnq_cs_high(void);
static bool vnq_write_reg(uint8_t addr, uint8_t data);
static bool vnq_read_reg(uint8_t addr, uint8_t *data, uint8_t *gsb);
static bool vnq_enter_normal_mode(void);
static bool vnq_service_watchdog(void);

/*============================================================================
 * Private Functions
 *============================================================================*/

static void vnq_cs_low(void)
{
    HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP();
}

static void vnq_cs_high(void)
{
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET);
}

static bool vnq_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t frame = VNQ7003_SPI_MK_WRITE_FRAME(addr, data);
    uint8_t txBuf[2] = {
        (uint8_t)(frame >> 8),
        (uint8_t)(frame & 0xFF)
    };

    vnq_cs_low();
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, txBuf, 2, VNQ_SPI_TIMEOUT_MS);
    vnq_cs_high();

    return (result == HAL_OK);
}

static bool vnq_read_reg(uint8_t addr, uint8_t *data, uint8_t *gsb)
{
    uint16_t txFrame = VNQ7003_SPI_MK_READ_FRAME(addr);
    uint16_t rxFrame = 0;

    uint8_t txBuf[2] = {
        (uint8_t)(txFrame >> 8),
        (uint8_t)(txFrame & 0xFF)
    };
    uint8_t rxBuf[2] = {0, 0};

    vnq_cs_low();
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, VNQ_SPI_TIMEOUT_MS);
    vnq_cs_high();

    if (result == HAL_OK) {
        rxFrame = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];

        if (gsb) {
            *gsb = (uint8_t)((rxFrame >> 8) & 0xFF);
        }
        if (data) {
            *data = (uint8_t)(rxFrame & 0xFF);
        }
        return true;
    }

    return false;
}

static bool vnq_enter_normal_mode(void)
{
    uint8_t ctrl_reg, gsb;

    printf("VNQ7003: Entering Normal mode sequence...\r\n");

    // Step 1: Read current CTRL register
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL register\r\n");
        return false;
    }
    printf("VNQ7003: Initial CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    // Step 2: Clear any previous state and set UNLOCK
    printf("VNQ7003: Setting UNLOCK bit...\r\n");
    if (!vnq_write_reg(VNQ7003_REG_CTLR, VNQ7003_CTLR_UNLOCK)) {
        printf("VNQ7003: Failed to set UNLOCK bit\r\n");
        return false;
    }
    HAL_Delay(10);

    // Step 3: Read back to verify UNLOCK was set
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL after UNLOCK\r\n");
        return false;
    }
    printf("VNQ7003: After UNLOCK: CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    if (!(ctrl_reg & VNQ7003_CTLR_UNLOCK)) {
        printf("VNQ7003: UNLOCK bit not set! CTRL=0x%02X\r\n", ctrl_reg);
        return false;
    }

    // Step 4: Set EN bit (enter Normal mode) and clear GOSTBY
    printf("VNQ7003: Setting EN bit (enter Normal mode)...\r\n");
    if (!vnq_write_reg(VNQ7003_REG_CTLR, VNQ7003_CTLR_EN)) {
        printf("VNQ7003: Failed to set EN bit\r\n");
        return false;
    }
    HAL_Delay(50);  // Longer delay for mode transition

    // Step 5: Verify Normal mode was entered
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL after EN\r\n");
        return false;
    }
    printf("VNQ7003: After EN: CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    // Check if EN bit is set and GOSTBY is clear
    if (!(ctrl_reg & VNQ7003_CTLR_EN)) {
        printf("VNQ7003: EN bit not set! CTRL=0x%02X\r\n", ctrl_reg);
        return false;
    }

    if (ctrl_reg & VNQ7003_CTLR_GOSTBY) {
        printf("VNQ7003: Still in GOSTBY mode! CTRL=0x%02X\r\n", ctrl_reg);
        return false;
    }

    // Step 6: Check GSB for fail-safe or undervoltage
    if (gsb & 0x01) {
        printf("VNQ7003: Device in fail-safe mode! GSB=0x%02X\r\n", gsb);
        return false;
    }

    if (gsb & 0x80) {
        printf("VNQ7003: WARNING - Undervoltage detected! GSB=0x%02X\r\n", gsb);
        printf("VNQ7003: Check VCC supply voltage (should be 8-28V)\r\n");
        // Continue anyway - this might be a supply issue, not a communication issue
    }

    printf("VNQ7003: Successfully entered Normal mode\r\n");
    return true;
}

static bool vnq_service_watchdog(void)
{
    uint8_t config, gsb;

    // Read CONFIG register to get current state
    if (!vnq_read_reg(VNQ7003_REG_CONFIG, &config, &gsb)) {
        consecutive_failures++;
        total_failures++;
        return false;
    }

    // Check if device is in fail-safe mode
    if (gsb & 0x01) {
        consecutive_failures++;
        total_failures++;
        return false;
    }

    // Toggle watchdog bit
    if (watchdog_toggle) {
        config |= VNQ7003_CONFIG_WDTB;
    } else {
        config &= ~VNQ7003_CONFIG_WDTB;
    }
    watchdog_toggle = !watchdog_toggle;

    // Write CONFIG back
    bool ok = vnq_write_reg(VNQ7003_REG_CONFIG, config);
    if (ok) {
        consecutive_failures = 0;
        total_services++;
    } else {
        consecutive_failures++;
        total_failures++;
    }

    return ok;
}

/*============================================================================
 * Public Functions
 *============================================================================*/

bool vnq_init(void)
{
    if (vnq_initialized) {
        return true;
    }

    printf("VNQ7003: Initializing...\r\n");

    // Enter Normal mode
    if (!vnq_enter_normal_mode()) {
        printf("VNQ7003: Failed to enter Normal mode\r\n");
        return false;
    }

    // Turn all outputs OFF initially
    if (!vnq_write_reg(VNQ7003_REG_SOCR, 0x00)) {
        printf("VNQ7003: Failed to turn off outputs\r\n");
        return false;
    }

    // Initialize watchdog toggle state
    watchdog_toggle = false;

    vnq_initialized = true;
    printf("VNQ7003: Initialization successful\r\n");

    return true;
}

void vnq_task(void)
{
    if (!vnq_initialized) return;
    vnq_service_watchdog();
}

bool vnq_set_channel(uint8_t channel, bool enable)
{
    if (!vnq_initialized || channel > 3) {
        return false;
    }

    uint8_t mask = enable ? (1 << channel) : 0x00;
    return vnq_write_reg(VNQ7003_REG_SOCR, mask);
}

bool vnq_set_outputs(uint8_t channel_mask)
{
    if (!vnq_initialized) {
        return false;
    }

    return vnq_write_reg(VNQ7003_REG_SOCR, channel_mask & 0x0F);
}

vnq_status_t vnq_get_status(void)
{
    uint8_t gensr, gsb;

    if (!vnq_initialized) {
        return VNQ_STATUS_COMM_ERROR;
    }

    if (!vnq_read_reg(VNQ7003_REG_GENSR, &gensr, &gsb)) {
        return VNQ_STATUS_COMM_ERROR;
    }

    // Check GSB for fail-safe mode
    if (gsb & 0x01) {
        return VNQ_STATUS_FAILSAFE;
    }

    // Check for other errors in GSB
    if (gsb & 0x3E) {
        return VNQ_STATUS_ERROR;
    }

    // Check GENSR for additional error conditions
    if (gensr & (VNQ7003_GENSR_VCCUV | VNQ7003_GENSR_SPIE)) {
        return VNQ_STATUS_ERROR;
    }

    return VNQ_STATUS_HEALTHY;
}

void vnq_emergency_stop(void)
{
    // Quick burst to ensure communication
    vnq_cs_low();
    __NOP(); __NOP();
    vnq_cs_high();
    __NOP();

    // Turn off all outputs
    vnq_write_reg(VNQ7003_REG_SOCR, 0x00);

    // Force re-initialization next time
    vnq_initialized = false;
}

bool vnq_is_initialized(void)
{
    return vnq_initialized;
}

void vnq_get_watchdog_stats(uint32_t *services, uint32_t *failures, uint32_t *consecutive)
{
    if (services) *services = total_services;
    if (failures) *failures = total_failures;
    if (consecutive) *consecutive = consecutive_failures;
}

/*============================================================================
 * Enhanced Direct Input Functions (for RV-C integration)
 *============================================================================*/

bool VNQ7003_Init_For_DI_Control(void)
{
    printf("VNQ7003: Initializing Direct Input control...\r\n");

    // First initialize the basic VNQ7003 driver
    if (!vnq_init()) {
        printf("VNQ7003: Basic initialization failed\r\n");
        return false;
    }

    // Enable direct input control for all channels
    uint8_t diencr_value = VNQ7003_DIENCR_CH0 | VNQ7003_DIENCR_CH1 |
                          VNQ7003_DIENCR_CH2 | VNQ7003_DIENCR_CH3;
    if (!vnq_write_reg(VNQ7003_REG_DIENCR, diencr_value)) {
        printf("VNQ7003: Failed to enable direct inputs\r\n");
        return false;
    }

    // Enable open-load pull-ups
    uint8_t oloffcr_value = VNQ7003_OLOFFCR_CH0 | VNQ7003_OLOFFCR_CH1 |
                           VNQ7003_OLOFFCR_CH2 | VNQ7003_OLOFFCR_CH3;
    if (!vnq_write_reg(VNQ7003_REG_OLOFFCR, oloffcr_value)) {
        printf("VNQ7003: Failed to enable pull-ups\r\n");
        return false;
    }

    // Set LED mode for channels 0-1
    uint8_t ccr_value = VNQ7003_CCR_MODE_CH0 | VNQ7003_CCR_MODE_CH1;
    if (!vnq_write_reg(VNQ7003_REG_CCR, ccr_value)) {
        printf("VNQ7003: Failed to set LED mode\r\n");
        return false;
    }

    // Configure GPIO pins for direct inputs
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set all DI pins LOW initially (loads OFF)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    // Start the watchdog timer
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        printf("VNQ7003: Failed to start watchdog timer\r\n");
        return false;
    }

    printf("VNQ7003: Direct Input control initialized successfully\r\n");
    return true;
}

bool VNQ7003_SetChannel_DI(uint8_t channel, bool enable)
{
    if (!vnq_initialized || channel > 3) {
        return false;
    }

    uint16_t pin;
    switch (channel) {
        case 0: pin = GPIO_PIN_3; break;  // IN0 -> PB3
        case 1: pin = GPIO_PIN_4; break;  // IN1 -> PB4
        case 2: pin = GPIO_PIN_5; break;  // IN2 -> PB5
        case 3: pin = GPIO_PIN_6; break;  // IN3 -> PB6
        default: return false;
    }

    // VNQ7003: HIGH on INx = load ON, LOW on INx = load OFF
    HAL_GPIO_WritePin(GPIOB, pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return true;
}

bool VNQ7003_SPI_WriteReg(uint8_t regAddr, uint8_t data)
{
    return vnq_write_reg(regAddr, data);
}

bool VNQ7003_SPI_ReadReg(uint8_t regAddr, uint8_t *data, uint8_t *gsb)
{
    return vnq_read_reg(regAddr, data, gsb);
}

void VNQ7003_ToggleWatchdog(void)
{
    if (!vnq_initialized) return;
    vnq_service_watchdog();
}

vnq7003_status_t VNQ7003_GetStatus(void)
{
    return vnq_get_status();
}

/*============================================================================
 * HAL Timer Callback (for watchdog)
 *============================================================================*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        VNQ7003_ToggleWatchdog();
    }
}

/*============================================================================
 * Debug Functions
 *============================================================================*/

void vnq_print_simple_status(void)
{
    uint8_t ctrl, gsb1, config, gsb2, socr, gsb3;

    printf("\r\n=== VNQ7003 Status ===\r\n");

    // Read CTRL (0x00)
    if (vnq_read_reg(VNQ7003_REG_CTLR, &ctrl, &gsb1)) {
        printf("CTRL   (0x00): 0x%02X, GSB: 0x%02X\r\n", ctrl, gsb1);
        printf("  EN: %s, UNLOCK: %s, GOSTBY: %s\r\n",
               (ctrl & VNQ7003_CTLR_EN) ? "SET" : "CLEAR",
               (ctrl & VNQ7003_CTLR_UNLOCK) ? "SET" : "CLEAR",
               (ctrl & VNQ7003_CTLR_GOSTBY) ? "ACTIVE" : "CLEAR");
    } else {
        printf("CTRL   (0x00): read failed\r\n");
    }

    // Read CONFIG (0x3F)
    if (vnq_read_reg(VNQ7003_REG_CONFIG, &config, &gsb2)) {
        printf("CONFIG (0x3F): 0x%02X, GSB: 0x%02X\r\n", config, gsb2);
        printf("  WDTB: %s\r\n", (config & VNQ7003_CONFIG_WDTB) ? "SET" : "CLEAR");
    } else {
        printf("CONFIG (0x3F): read failed\r\n");
    }

    // Read SOCR (0x07)
    if (vnq_read_reg(VNQ7003_REG_SOCR, &socr, &gsb3)) {
        printf("SOCR   (0x07): 0x%02X (Ch0:%s Ch1:%s Ch2:%s Ch3:%s), GSB: 0x%02X\r\n",
               socr,
               (socr & 0x01) ? "ON " : "OFF",
               (socr & 0x02) ? "ON " : "OFF",
               (socr & 0x04) ? "ON " : "OFF",
               (socr & 0x08) ? "ON " : "OFF",
               gsb3);
    } else {
        printf("SOCR   (0x07): read failed\r\n");
    }

    // Analyze GSB bits
    printf("\r\nGSB Analysis:\r\n");
    if (gsb1 & 0x01) printf("  - FAILSAFE mode active!\r\n");
    if (gsb1 & 0x02) printf("  - Open-load detected\r\n");
    if (gsb1 & 0x04) printf("  - Overtemperature detected\r\n");
    if (gsb1 & 0x08) printf("  - Power limitation active\r\n");
    if (gsb1 & 0x10) printf("  - VDS overvoltage detected\r\n");
    if (gsb1 & 0x20) printf("  - Case thermal protection\r\n");
    if (gsb1 & 0x40) printf("  - SPI error detected\r\n");
    if (gsb1 & 0x80) printf("  - Undervoltage warning\r\n");

    // Print watchdog stats
    printf("Watchdog: %lu services, %lu failures, %lu consecutive\r\n",
           total_services, total_failures, consecutive_failures);

    printf("======================\r\n\r\n");
}
