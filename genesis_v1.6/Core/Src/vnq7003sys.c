// vnq7003.c - Unified VNQ7003SY Driver Implementation - FIXED VERSION
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
static void vnq_comprehensive_diagnosis(void);
static bool vnq_check_power_supply(void);
static bool vnq_test_spi_communication(void);

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

// Enhanced VNQ7003 initialization with comprehensive debugging
static bool vnq_enter_normal_mode(void)
{
    uint8_t ctrl_reg, gsb;
    uint8_t device_id, family, version;

    printf("VNQ7003: Enhanced Normal mode entry with diagnostics...\r\n");

    // Step 0: Check device identification first
    printf("VNQ7003: Reading device identification...\r\n");
    if (vnq_read_reg(VNQ7003_REG_COMPANY_CODE, &device_id, &gsb)) {
        printf("VNQ7003: Company Code: 0x%02X (expected: 0x00)\r\n", device_id);
    }
    if (vnq_read_reg(VNQ7003_REG_DEVICE_FAMILY, &family, &gsb)) {
        printf("VNQ7003: Device Family: 0x%02X (expected: 0x01)\r\n", family);
    }
    if (vnq_read_reg(VNQ7003_REG_VERSION, &version, &gsb)) {
        printf("VNQ7003: Version: 0x%02X\r\n", version);
    }

    // Step 1: Try software reset first
    printf("VNQ7003: Attempting software reset...\r\n");
    vnq_write_reg(VNQ7003_REG_CTLR, VNQ7003_CMD_SWRESET);
    HAL_Delay(100);  // Wait for reset to complete

    // Step 2: Read initial state after reset
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL register\r\n");
        return false;
    }
    printf("VNQ7003: After reset - CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    // Step 3: Check VCC voltage issue
    if (gsb & 0x80) {
        printf("VNQ7003: CRITICAL - Undervoltage detected! GSB=0x%02X\r\n", gsb);
        printf("VNQ7003: VCC must be 8-28V. Current voltage appears too low.\r\n");
        printf("VNQ7003: Check power supply to VNQ7003 VCC pin!\r\n");
        // Continue anyway to see if we can get more info
    }

    // Step 4: Check if device is in fail-safe mode
    if (gsb & 0x01) {
        printf("VNQ7003: Device is in FAIL-SAFE mode! GSB=0x%02X\r\n", gsb);
        printf("VNQ7003: This may prevent Normal mode entry.\r\n");
    }

    // Step 5: Try different approach - set thermal detection threshold first
    printf("VNQ7003: Setting thermal threshold to 120°C and UNLOCK...\r\n");
    uint8_t ctrl_with_thermal = VNQ7003_CTLR_UNLOCK;  // 120°C = 00b for CTDTH bits
    if (!vnq_write_reg(VNQ7003_REG_CTLR, ctrl_with_thermal)) {
        printf("VNQ7003: Failed to set UNLOCK with thermal config\r\n");
        return false;
    }
    HAL_Delay(50);

    // Step 6: Verify UNLOCK was set
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL after UNLOCK\r\n");
        return false;
    }
    printf("VNQ7003: After UNLOCK: CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    if (!(ctrl_reg & VNQ7003_CTLR_UNLOCK)) {
        printf("VNQ7003: UNLOCK bit not set! CTRL=0x%02X\r\n", ctrl_reg);
        return false;
    }

    // Step 7: Now try to set EN bit with longer delay
    printf("VNQ7003: Setting EN bit (Normal mode) with extended delay...\r\n");
    uint8_t normal_mode_value = VNQ7003_CTLR_UNLOCK | VNQ7003_CTLR_EN;  // 0x11
    if (!vnq_write_reg(VNQ7003_REG_CTLR, normal_mode_value)) {
        printf("VNQ7003: Failed to write EN bit\r\n");
        return false;
    }
    HAL_Delay(200);  // Much longer delay for mode transition

    // Step 8: Check result
    if (!vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb)) {
        printf("VNQ7003: Failed to read CTRL after EN\r\n");
        return false;
    }
    printf("VNQ7003: After EN attempt: CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);

    // Step 9: Detailed bit analysis
    printf("VNQ7003: Bit analysis - EN=%d, UNLOCK=%d, GOSTBY=%d\r\n",
           (ctrl_reg & VNQ7003_CTLR_EN) ? 1 : 0,
           (ctrl_reg & VNQ7003_CTLR_UNLOCK) ? 1 : 0,
           (ctrl_reg & VNQ7003_CTLR_GOSTBY) ? 1 : 0);

    // Step 10: If EN still not set, try alternative approach
    if (!(ctrl_reg & VNQ7003_CTLR_EN)) {
        printf("VNQ7003: EN bit still not set. Trying alternative sequence...\r\n");

        // Clear everything first
        vnq_write_reg(VNQ7003_REG_CTLR, 0x00);
        HAL_Delay(50);

        // Set UNLOCK only
        vnq_write_reg(VNQ7003_REG_CTLR, VNQ7003_CTLR_UNLOCK);
        HAL_Delay(100);

        // Now set EN without clearing UNLOCK (different order)
        uint8_t current_ctrl;
        vnq_read_reg(VNQ7003_REG_CTLR, &current_ctrl, &gsb);
        current_ctrl |= VNQ7003_CTLR_EN;
        vnq_write_reg(VNQ7003_REG_CTLR, current_ctrl);
        HAL_Delay(200);

        // Final check
        vnq_read_reg(VNQ7003_REG_CTLR, &ctrl_reg, &gsb);
        printf("VNQ7003: Alternative approach result: CTRL=0x%02X, GSB=0x%02X\r\n", ctrl_reg, gsb);
    }

    // Step 11: Final validation
    if (!(ctrl_reg & VNQ7003_CTLR_EN)) {
        printf("VNQ7003: FAILED - EN bit never set. Possible causes:\r\n");
        printf("  1. VCC voltage too low (check power supply)\r\n");
        printf("  2. Device hardware fault\r\n");
        printf("  3. Watchdog expired (device in fail-safe)\r\n");
        printf("  4. Overtemperature protection active\r\n");

        // Try to read diagnostic info
        uint8_t gensr;
        if (vnq_read_reg(VNQ7003_REG_GENSR, &gensr, &gsb)) {
            printf("VNQ7003: GENSR=0x%02X (VCCUV=%d, RST=%d, SPIE=%d)\r\n",
                   gensr,
                   (gensr & VNQ7003_GENSR_VCCUV) ? 1 : 0,
                   (gensr & VNQ7003_GENSR_RST) ? 1 : 0,
                   (gensr & VNQ7003_GENSR_SPIE) ? 1 : 0);
        }

        return false;
    }

    // Step 12: Check for other issues
    if (ctrl_reg & VNQ7003_CTLR_GOSTBY) {
        printf("VNQ7003: WARNING - GOSTBY bit still set! CTRL=0x%02X\r\n", ctrl_reg);
    }

    if (gsb & 0x01) {
        printf("VNQ7003: WARNING - Still in fail-safe mode! GSB=0x%02X\r\n", gsb);
        return false;
    }

    printf("VNQ7003: SUCCESS - Normal mode entered! CTRL=0x%02X\r\n", ctrl_reg);
    return true;
}

// Comprehensive diagnosis function - SINGLE DEFINITION
static void vnq_comprehensive_diagnosis(void)
{
    uint8_t data, gsb;

    printf("\r\n=== VNQ7003 Comprehensive Diagnosis ===\r\n");

    // Read all important registers
    if (vnq_read_reg(VNQ7003_REG_CTLR, &data, &gsb)) {
        printf("CTRL (0x00): 0x%02X, GSB: 0x%02X\r\n", data, gsb);
    }

    if (vnq_read_reg(VNQ7003_REG_GENSR, &data, &gsb)) {
        printf("GENSR (0x34): 0x%02X, GSB: 0x%02X\r\n", data, gsb);
        if (data & VNQ7003_GENSR_VCCUV) printf("  - VCC Undervoltage detected!\r\n");
        if (data & VNQ7003_GENSR_RST) printf("  - Reset event occurred\r\n");
        if (data & VNQ7003_GENSR_SPIE) printf("  - SPI error detected\r\n");
    }

    if (vnq_read_reg(VNQ7003_REG_CONFIG, &data, &gsb)) {
        printf("CONFIG (0x3F): 0x%02X, GSB: 0x%02X\r\n", data, gsb);
    }

    // Try to read device ID
    if (vnq_read_reg(VNQ7003_REG_COMPANY_CODE, &data, &gsb)) {
        printf("Company Code: 0x%02X (should be 0x00)\r\n", data);
    }

    if (vnq_read_reg(VNQ7003_REG_DEVICE_FAMILY, &data, &gsb)) {
        printf("Device Family: 0x%02X (should be 0x01)\r\n", data);
    }

    printf("========================================\r\n\r\n");
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

// Function to check if the issue is VCC voltage
static bool vnq_check_power_supply(void)
{
    uint8_t gensr, gsb;
    bool power_ok = true;

    printf("VNQ7003: Checking power supply status...\r\n");

    if (vnq_read_reg(VNQ7003_REG_GENSR, &gensr, &gsb)) {
        printf("VNQ7003: GENSR=0x%02X, GSB=0x%02X\r\n", gensr, gsb);

        if (gsb & 0x80) {
            printf("VNQ7003: ERROR - VCC Undervoltage! GSB bit 7 set\r\n");
            printf("VNQ7003: VCC voltage is below minimum threshold\r\n");
            printf("VNQ7003: Required: 8V-28V, Typical: 12V\r\n");
            power_ok = false;
        }

        if (gensr & VNQ7003_GENSR_VCCUV) {
            printf("VNQ7003: ERROR - GENSR Undervoltage flag set\r\n");
            power_ok = false;
        }

        if (gsb & 0x01) {
            printf("VNQ7003: WARNING - Device in Fail-Safe mode\r\n");
            printf("VNQ7003: This could be due to undervoltage or watchdog timeout\r\n");
        }
    } else {
        printf("VNQ7003: ERROR - Cannot read GENSR (SPI communication failure)\r\n");
        return false;
    }

    if (power_ok) {
        printf("VNQ7003: Power supply appears OK\r\n");
    } else {
        printf("VNQ7003: POWER SUPPLY ISSUE DETECTED!\r\n");
        printf("VNQ7003: Actions to take:\r\n");
        printf("  1. Measure VCC pin voltage with multimeter\r\n");
        printf("  2. Check power supply capacity (device draws up to 80A)\r\n");
        printf("  3. Verify power supply connections\r\n");
        printf("  4. Check for voltage drops in wiring\r\n");
    }

    return power_ok;
}

// Function to test SPI communication
static bool vnq_test_spi_communication(void)
{
    uint8_t company_code, device_family, version, gsb;
    bool spi_ok = true;

    printf("VNQ7003: Testing SPI communication...\r\n");

    // Try to read ROM registers (these should always work)
    if (vnq_read_reg(VNQ7003_REG_COMPANY_CODE, &company_code, &gsb)) {
        printf("VNQ7003: Company Code: 0x%02X (expected: 0x00)\r\n", company_code);
        if (company_code != 0x00) {
            printf("VNQ7003: WARNING - Unexpected company code\r\n");
            spi_ok = false;
        }
    } else {
        printf("VNQ7003: ERROR - Cannot read Company Code\r\n");
        spi_ok = false;
    }

    if (vnq_read_reg(VNQ7003_REG_DEVICE_FAMILY, &device_family, &gsb)) {
        printf("VNQ7003: Device Family: 0x%02X (expected: 0x01)\r\n", device_family);
        if (device_family != 0x01) {
            printf("VNQ7003: WARNING - Unexpected device family\r\n");
            spi_ok = false;
        }
    } else {
        printf("VNQ7003: ERROR - Cannot read Device Family\r\n");
        spi_ok = false;
    }

    if (vnq_read_reg(VNQ7003_REG_VERSION, &version, &gsb)) {
        printf("VNQ7003: Silicon Version: 0x%02X\r\n", version);
    } else {
        printf("VNQ7003: ERROR - Cannot read Version\r\n");
        spi_ok = false;
    }

    if (spi_ok) {
        printf("VNQ7003: SPI communication is working\r\n");
    } else {
        printf("VNQ7003: SPI COMMUNICATION ISSUES DETECTED!\r\n");
        printf("VNQ7003: Check:\r\n");
        printf("  1. SPI wiring (SCK, MOSI, MISO, CS)\r\n");
        printf("  2. SPI clock frequency (max 10 MHz)\r\n");
        printf("  3. Ground connections\r\n");
        printf("  4. Signal integrity\r\n");
    }

    return spi_ok;
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

// Modified initialization function with better error handling
bool VNQ7003_Init_For_DI_Control_Enhanced(void)
{
    printf("VNQ7003: Enhanced initialization with diagnostics...\r\n");

    // Step 1: Test SPI communication first
    if (!vnq_test_spi_communication()) {
        printf("VNQ7003: SPI test failed - check hardware connections\r\n");
        return false;
    }

    // Step 2: Check power supply
    if (!vnq_check_power_supply()) {
        printf("VNQ7003: Power supply issues detected\r\n");
        printf("VNQ7003: Continuing anyway to test functionality...\r\n");
        // Don't return false here - try to continue
    }

    // Step 3: Run comprehensive diagnosis
    vnq_comprehensive_diagnosis();

    // Step 4: Try enhanced normal mode entry
    if (!vnq_enter_normal_mode()) {
        printf("VNQ7003: Enhanced normal mode entry failed\r\n");

        // Final diagnostic attempt
        printf("VNQ7003: Final diagnostic - trying to understand the issue...\r\n");
        uint8_t ctrl, gsb;
        if (vnq_read_reg(VNQ7003_REG_CTLR, &ctrl, &gsb)) {
            printf("VNQ7003: Final CTRL=0x%02X, GSB=0x%02X\r\n", ctrl, gsb);

            if (gsb & 0x80) {
                printf("VNQ7003: ROOT CAUSE: VCC voltage too low!\r\n");
                printf("VNQ7003: The device cannot enter Normal mode with low VCC\r\n");
                printf("VNQ7003: FIX: Increase VCC to at least 8V (12V recommended)\r\n");
            } else if (gsb & 0x01) {
                printf("VNQ7003: ROOT CAUSE: Device stuck in Fail-Safe mode\r\n");
                printf("VNQ7003: This could be due to watchdog timeout or protection\r\n");
            } else {
                printf("VNQ7003: ROOT CAUSE: Unknown - device may be defective\r\n");
            }
        }

        return false;
    }

    // Step 5: Continue with rest of initialization if Normal mode worked
    printf("VNQ7003: Normal mode successful, continuing initialization...\r\n");

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

    printf("VNQ7003: Enhanced initialization completed successfully\r\n");
    return true;
}

// Enhanced VNQ7003 Diagnostic - Complete register scan
void vnq_complete_diagnostic(void)
{
    printf("\r\n=== COMPLETE VNQ7003 DIAGNOSTIC ===\r\n");

    // 1. Read all status registers to find the hidden fault
    printf("1. Complete Status Register Scan:\r\n");

    uint8_t reg_data, gsb;

    // GENSR (0x34) - Generic Status Register
    if (vnq_read_reg(VNQ7003_REG_GENSR, &reg_data, &gsb)) {
        printf("   GENSR (0x34): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      VCCUV (bit 7): %s\r\n", (reg_data & 0x80) ? "UNDERVOLTAGE!" : "OK");
        printf("      RST (bit 6): %s\r\n", (reg_data & 0x40) ? "Reset occurred" : "OK");
        printf("      SPIE (bit 5): %s\r\n", (reg_data & 0x20) ? "SPI Error" : "OK");
    } else {
        printf("   GENSR: READ FAILED\r\n");
    }

    // CHFBSR (0x30) - Channel Feedback Status Register
    if (vnq_read_reg(VNQ7003_REG_CHFBSR, &reg_data, &gsb)) {
        printf("   CHFBSR (0x30): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      CH0 fault: %s\r\n", (reg_data & 0x01) ? "YES" : "No");
        printf("      CH1 fault: %s\r\n", (reg_data & 0x02) ? "YES" : "No");
        printf("      CH2 fault: %s\r\n", (reg_data & 0x04) ? "YES" : "No");
        printf("      CH3 fault: %s\r\n", (reg_data & 0x08) ? "YES" : "No");
    } else {
        printf("   CHFBSR: READ FAILED\r\n");
    }

    // STKFLTR (0x31) - Open-load OFF-state/Stuck to VCC Status
    if (vnq_read_reg(VNQ7003_REG_STKFLTR, &reg_data, &gsb)) {
        printf("   STKFLTR (0x31): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      CH0 open/stuck: %s\r\n", (reg_data & 0x01) ? "YES" : "No");
        printf("      CH1 open/stuck: %s\r\n", (reg_data & 0x02) ? "YES" : "No");
        printf("      CH2 open/stuck: %s\r\n", (reg_data & 0x04) ? "YES" : "No");
        printf("      CH3 open/stuck: %s\r\n", (reg_data & 0x08) ? "YES" : "No");
    } else {
        printf("   STKFLTR: READ FAILED\r\n");
    }

    // CHLOFFSR (0x32) - Channels Latch-off Status Register
    if (vnq_read_reg(VNQ7003_REG_CHLOFFSR, &reg_data, &gsb)) {
        printf("   CHLOFFSR (0x32): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      CH0 latch-off: %s\r\n", (reg_data & 0x01) ? "YES" : "No");
        printf("      CH1 latch-off: %s\r\n", (reg_data & 0x02) ? "YES" : "No");
        printf("      CH2 latch-off: %s\r\n", (reg_data & 0x04) ? "YES" : "No");
        printf("      CH3 latch-off: %s\r\n", (reg_data & 0x08) ? "YES" : "No");
    } else {
        printf("   CHLOFFSR: READ FAILED\r\n");
    }

    // VDSFSR (0x33) - VDS Feedback Status Register
    if (vnq_read_reg(VNQ7003_REG_VDSFSR, &reg_data, &gsb)) {
        printf("   VDSFSR (0x33): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      CH0 VDS high: %s\r\n", (reg_data & 0x01) ? "YES" : "No");
        printf("      CH1 VDS high: %s\r\n", (reg_data & 0x02) ? "YES" : "No");
        printf("      CH2 VDS high: %s\r\n", (reg_data & 0x04) ? "YES" : "No");
        printf("      CH3 VDS high: %s\r\n", (reg_data & 0x08) ? "YES" : "No");
    } else {
        printf("   VDSFSR: READ FAILED\r\n");
    }

    // 2. Check all control registers
    printf("\r\n2. Control Register Status:\r\n");

    // CTLR (0x00) - Control Register
    if (vnq_read_reg(VNQ7003_REG_CTLR, &reg_data, &gsb)) {
        printf("   CTLR (0x00): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      EN: %s\r\n", (reg_data & 0x01) ? "Normal Mode" : "Fail-Safe Mode");
        printf("      CTDTH0: %s\r\n", (reg_data & 0x02) ? "Set" : "Clear");
        printf("      CTDTH1: %s\r\n", (reg_data & 0x04) ? "Set" : "Clear");
        printf("      UNLOCK: %s\r\n", (reg_data & 0x10) ? "Set" : "Clear");
        printf("      GOSTBY: %s\r\n", (reg_data & 0x20) ? "Standby Mode" : "Clear");
    } else {
        printf("   CTLR: READ FAILED\r\n");
    }

    // CONFIG (0x3F) - Configuration Register
    if (vnq_read_reg(VNQ7003_REG_CONFIG, &reg_data, &gsb)) {
        printf("   CONFIG (0x3F): 0x%02X, GSB: 0x%02X\r\n", reg_data, gsb);
        printf("      WDTB: %s\r\n", (reg_data & 0x08) ? "Set" : "Clear");
        printf("      VDS Masks: CH0=%s CH1=%s CH2=%s CH3=%s\r\n",
               (reg_data & 0x10) ? "Masked" : "Active",
               (reg_data & 0x20) ? "Masked" : "Active",
               (reg_data & 0x40) ? "Masked" : "Active",
               (reg_data & 0x80) ? "Masked" : "Active");
    } else {
        printf("   CONFIG: READ FAILED\r\n");
    }

    // 3. Test the CSN-only GSB read (faster method)
    printf("\r\n3. Quick GSB Test (CSN pulse method):\r\n");
    for (int i = 0; i < 5; i++) {
        // Just pulse CSN to get GSB without full SPI transaction
        HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_RESET);
        __NOP(); __NOP(); __NOP(); __NOP(); // Small delay
        HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET);

        // Now do a read to get the GSB
        if (vnq_read_reg(VNQ7003_REG_CTLR, &reg_data, &gsb)) {
            printf("   Test %d: GSB=0x%02X (bits: ", i+1, gsb);
            for (int bit = 7; bit >= 0; bit--) {
                printf("%d", (gsb >> bit) & 1);
            }
            printf(")\r\n");
        }
        HAL_Delay(50);
    }

    // 4. Voltage analysis
    printf("\r\n4. Power Supply Deep Analysis:\r\n");
    if (vnq_read_reg(VNQ7003_REG_GENSR, &reg_data, &gsb)) {
        printf("   GENSR VCCUV flag: %s\r\n", (reg_data & 0x80) ? "UNDERVOLTAGE" : "OK");
        printf("   GSB Global fault: %s\r\n", (gsb & 0x80) ? "FAULT PRESENT" : "OK");

        if ((gsb & 0x80) && !(reg_data & 0x80)) {
            printf("   🔍 ANALYSIS: GSB shows fault but GENSR shows VCC OK\r\n");
            printf("      This suggests the fault is NOT VCC undervoltage!\r\n");
            printf("      Check other status registers above for the real cause.\r\n");
        } else if ((gsb & 0x80) && (reg_data & 0x80)) {
            printf("   ⚠️  CONFIRMED: VCC Undervoltage is the problem!\r\n");
            printf("      Measure VCC pin voltage - must be 8-28V\r\n");
        }
    }

    // 5. Clear all status registers attempt
    printf("\r\n5. Attempting to Clear Status Registers:\r\n");

    // Try the clear-all-status command
    uint8_t clear_cmd = VNQ7003_CMD_CLEAR_STATUS;  // 0xBF
    printf("   Sending clear status command (0xBF)...\r\n");

    HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &clear_cmd, 1, 100);
    HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET);

    HAL_Delay(100);

    // Check if GSB cleared
    if (vnq_read_reg(VNQ7003_REG_CTLR, &reg_data, &gsb)) {
        printf("   After clear: GSB=0x%02X %s\r\n", gsb,
               (gsb & 0x80) ? "Still faulted" : "Cleared!");
    }

    printf("\r\n=== DIAGNOSTIC COMPLETE ===\r\n");
    printf("Key findings:\r\n");
    printf("• If GENSR bit 7 = 1: VCC voltage problem\r\n");
    printf("• If other status registers show faults: Hardware/wiring issue\r\n");
    printf("• If GSB clears after command: Temporary fault\r\n");
    printf("• If nothing clears GSB bit 7: Persistent hardware problem\r\n");
    printf("==============================\r\n\r\n");
}
