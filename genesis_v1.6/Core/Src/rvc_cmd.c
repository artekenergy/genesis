#include "rvc_cmd.h"
#include "rvc_defs.h"
#include "vnq7003sys.h"   // Use the unified VNQ7003 driver
#include "main.h"
#include <string.h>
#include <stdio.h>

/*============================================================================
 * External HAL Handles (declared in main.c)
 *============================================================================*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef   htim1;
extern TIM_HandleTypeDef   htim3;

/*============================================================================
 * "Global" RV-C State
 *============================================================================*/
static struct {
    bool     initialized;
    uint32_t message_count;
    uint32_t error_count;
    struct {
        uint8_t source_address;
        uint8_t device_type;
    } config;
} rvc_state = { 0 };

/*============================================================================
 * Channel State Tracking (4 channels)
 *============================================================================*/
typedef struct {
    bool    on_off_state;        // digital ON/OFF
    uint8_t pwm_percent;         // 0..100%
    uint32_t last_command_ms;    // HAL_GetTick() at last command
} channel_state_t;

static channel_state_t channel_states[4] = { 0 };

/*============================================================================
 * Private Constants
 *============================================================================*/
#define RVC_OP_STATUS_ON_ACTIVE   0x05    // "On, Active"
#define RVC_LAMP_YELLOW_OFF       0x00    // "Yellow lamp off"
#define RVC_DEVICE_TYPE_DC_LOAD   0xFF    // Device type (DSA)

/*============================================================================
 * Private Prototypes
 *============================================================================*/
static void     rvc_configure_can_filters(void);
static bool     rvc_send_can_message(uint32_t can_id, const uint8_t *data, uint8_t length);
static void     rvc_handle_request_message(const rvc_message_t *msg);
static void     RVC_UpdateChannelState(uint8_t channel, bool on_off, uint8_t pwm_percent);
static void     RVC_ApplyChannelControl(uint8_t channel);
static bool     RVC_ValidateChannel(uint8_t channel);
static void     RVC_SendStatusResponse(uint16_t dgn, uint8_t channel);

/*============================================================================
 * Public: RVC_Init
 *============================================================================*/
void RVC_Init(void)
{
    rvc_state.message_count = 0;
    rvc_state.error_count   = 0;
    rvc_state.initialized   = false;

    rvc_state.config.source_address = RVC_SRC_ADDR;
    rvc_state.config.device_type    = RVC_DEVICE_TYPE_DC_LOAD;

    printf("RV-C: Initializing (SA=0x%02X, DSA=0x%02X)...\r\n",
           rvc_state.config.source_address,
           rvc_state.config.device_type);

    // 1) Configure CAN filters
    rvc_configure_can_filters();

    // 2) Activate RX FIFO0 notification
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0) != HAL_OK)
    {
        printf("RV-C: Failed to activate FDCAN notifications\r\n");
        Error_Handler();
    }

    // 3) Start FDCAN1
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        printf("RV-C: Failed to start FDCAN\r\n");
        Error_Handler();
    }

    // 4) Start PWM on all channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 5) Initialize all channels OFF
    for (uint8_t ch = 0; ch < 4; ch++) {
        RVC_UpdateChannelState(ch, false, 0);
        RVC_ApplyChannelControl(ch);
    }

    rvc_state.initialized = true;
    printf("RV-C: Initialization complete, listening for commands\r\n");
}

/*============================================================================
 * Private: Configure 4 FDCAN Filters for Load/Dimmer (broadcast + directed)
 *============================================================================*/
static void rvc_configure_can_filters(void)
{
    FDCAN_FilterTypeDef filter_config;

    // Mask for exact DGN and DA matching
    uint32_t common_mask =
         (1U << 25)     // reserved bit = 0
       | (1U << 24)     // DP bit = 0
       | (0xFFU << 16)  // PF bits (DGN high) must match
       | (0xFFU << 8);  // PS bits (DA) must match

    // Base filter configuration
    filter_config.IdType       = FDCAN_EXTENDED_ID;
    filter_config.FilterType   = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    // Filter 0: Load Command → Broadcast
    filter_config.FilterIndex = 0;
    filter_config.FilterID1   = rvc_build_can_id(
                                   RVC_PRIORITY,
                                   (DGN_LOAD_COMMAND >> 8) & 0xFF,
                                   0xFF,    // DA=broadcast
                                   0x00     // SA=don't care
                               );
    filter_config.FilterID2 = common_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        printf("RV-C: Failed to configure filter 0 (Load, Broadcast)\r\n");
        Error_Handler();
    }

    // Filter 1: Load Command → This Node
    filter_config.FilterIndex = 1;
    filter_config.FilterID1   = rvc_build_can_id(
                                   RVC_PRIORITY,
                                   (DGN_LOAD_COMMAND >> 8) & 0xFF,
                                   rvc_state.config.source_address,
                                   0x00
                               );
    filter_config.FilterID2 = common_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        printf("RV-C: Failed to configure filter 1 (Load, Directed)\r\n");
        Error_Handler();
    }

    // Filter 2: Dimmer Command → Broadcast
    filter_config.FilterIndex = 2;
    filter_config.FilterID1   = rvc_build_can_id(
                                   RVC_PRIORITY,
                                   (DGN_DIMMER_COMMAND >> 8) & 0xFF,
                                   0xFF,    // DA=broadcast
                                   0x00
                               );
    filter_config.FilterID2 = common_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        printf("RV-C: Failed to configure filter 2 (Dimmer, Broadcast)\r\n");
        Error_Handler();
    }

    // Filter 3: Dimmer Command → This Node
    filter_config.FilterIndex = 3;
    filter_config.FilterID1   = rvc_build_can_id(
                                   RVC_PRIORITY,
                                   (DGN_DIMMER_COMMAND >> 8) & 0xFF,
                                   rvc_state.config.source_address,
                                   0x00
                               );
    filter_config.FilterID2 = common_mask;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        printf("RV-C: Failed to configure filter 3 (Dimmer, Directed)\r\n");
        Error_Handler();
    }
}

/*============================================================================
 * Public: HAL_FDCAN_RxFifo0MsgPendingCallback
 *============================================================================*/
void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1 || !rvc_state.initialized) {
        return;
    }

    uint8_t RxData[8];
    FDCAN_RxHeaderTypeDef RxHeader;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        rvc_state.error_count++;
        return;
    }

    if (RxHeader.IdType != FDCAN_EXTENDED_ID) {
        rvc_state.error_count++;
        return;
    }

    rvc_state.message_count++;
    uint32_t fullId  = RxHeader.Identifier;
    uint32_t dgn_high = RVC_GET_DGN_HIGH(fullId);
    uint32_t dgn_low  = RVC_GET_DGN_LOW(fullId);

    // Reconstruct the 18-bit DGN
    uint32_t dgn = (dgn_low == 0xFF)
                     ? ((dgn_high << 8) | 0xFF)   // broadcast form
                     : (dgn_high << 8);           // directed form

    uint8_t da     = RVC_GET_DGN_LOW(fullId);
    uint8_t sa     = RVC_GET_SOURCE_ADDR(fullId);
    uint8_t length = (uint8_t)(RxHeader.DataLength >> 16);

    printf("RV-C: RX 0x%08lX → DGN=0x%05lX DA=0x%02X SA=0x%02X Len=%d\r\n",
           fullId, dgn, da, sa, length);

    // Dispatch based on DGN
    switch (dgn) {
        case DGN_LOAD_COMMAND:
            RVC_HandleLoadCommand((uint16_t)dgn, RxData, length);
            break;

        case DGN_DIMMER_COMMAND:
            RVC_HandleDimmerCommand((uint16_t)dgn, RxData, length);
            break;

        case DGN_INFO_REQUEST:
            {
                rvc_message_t msg;
                msg.priority       = RVC_GET_PRIORITY(fullId);
                msg.dgn_high       = RVC_GET_DGN_HIGH(fullId);
                msg.dgn_low        = RVC_GET_DGN_LOW(fullId);
                msg.source_address = sa;
                msg.data_length    = length;
                memcpy(msg.data, RxData, 8);
                rvc_handle_request_message(&msg);
            }
            break;

        default:
            printf("RV-C: Unknown DGN=0x%05lX\r\n", dgn);
            break;
    }
}

/*============================================================================
 * RVC_HandleLoadCommand
 *============================================================================*/
void RVC_HandleLoadCommand(uint16_t dgn, uint8_t *data, uint8_t length)
{
    if (!rvc_state.initialized || (data == NULL) || (length < 2)) {
        rvc_state.error_count++;
        return;
    }

    uint8_t channel   = data[0];
    uint8_t onOffByte = data[1];

    printf("RV-C: LoadCommand ch=%d, state=%d\r\n", channel, onOffByte);

    if (!RVC_ValidateChannel(channel)) {
        printf("RV-C: Invalid channel %d\r\n", channel);
        rvc_state.error_count++;
        return;
    }

    bool enable = (onOffByte != 0);

    // Update state and apply control
    RVC_UpdateChannelState(channel, enable, enable ? 100 : 0);
    RVC_ApplyChannelControl(channel);

    // Send status response
    RVC_SendStatusResponse(dgn, channel);
}

/*============================================================================
 * RVC_HandleDimmerCommand
 *============================================================================*/
void RVC_HandleDimmerCommand(uint16_t dgn, uint8_t *data, uint8_t length)
{
    if (!rvc_state.initialized || (data == NULL) || (length < 2)) {
        rvc_state.error_count++;
        return;
    }

    uint8_t channel = data[0];
    uint8_t percent = data[1];

    printf("RV-C: DimmerCommand ch=%d, pct=%d%%\r\n", channel, percent);

    if (!RVC_ValidateChannel(channel)) {
        printf("RV-C: Invalid channel %d\r\n", channel);
        rvc_state.error_count++;
        return;
    }

    if (percent > 100) percent = 100;
    bool channel_on = (percent > 0);

    RVC_UpdateChannelState(channel, channel_on, percent);
    RVC_ApplyChannelControl(channel);

    RVC_SendStatusResponse(dgn, channel);
}

/*============================================================================
 * rvc_handle_request_message
 *============================================================================*/
static void rvc_handle_request_message(const rvc_message_t *msg)
{
    if (msg == NULL || msg->data_length < 3) {
        return;
    }

    uint32_t requested_dgn = ((uint32_t)msg->data[2] << 16)
                           | ((uint32_t)msg->data[1] << 8)
                           | (uint32_t)msg->data[0];

    printf("RV-C: InfoRequest for DGN=0x%05lX\r\n", requested_dgn);

    if (requested_dgn == DGN_DM_RV) {
        rvc_send_diagnostic(RVC_OP_STATUS_ON_ACTIVE, RVC_LAMP_YELLOW_OFF);
    }
}

/*============================================================================
 * rvc_send_diagnostic
 *============================================================================*/
bool rvc_send_diagnostic(uint8_t operating_status, uint8_t lamp_status)
{
    if (!rvc_state.initialized) return false;

    rvc_message_t msg;
    msg.priority        = RVC_PRIORITY;
    msg.dgn_high        = (DGN_DM_RV >> 8) & 0xFF;
    msg.dgn_low         = DGN_DM_RV & 0xFF;
    msg.source_address  = rvc_state.config.source_address;
    msg.data_length     = 8;

    // Fill with 0xFF (not available)
    for (int i = 0; i < 8; i++) {
        msg.data[i] = 0xFF;
    }

    // Byte 0: operating_status | lamp_status
    msg.data[0] = operating_status | lamp_status;
    // Byte 1: DSA
    msg.data[1] = rvc_state.config.device_type;
    // Byte 6: 0xFF (no DSA extension)
    msg.data[6] = 0xFF;
    // Byte 7: 0x0F (bank select = not applicable)
    msg.data[7] = 0x0F;

    return rvc_send_message(&msg);
}

/*============================================================================
 * rvc_send_message
 *============================================================================*/
bool rvc_send_message(const rvc_message_t *msg)
{
    if (!rvc_state.initialized || msg == NULL) {
        return false;
    }

    uint32_t can_id = rvc_build_can_id(
                         msg->priority,
                         msg->dgn_high,
                         msg->dgn_low,
                         msg->source_address);
    return rvc_send_can_message(can_id, msg->data, msg->data_length);
}

/*============================================================================
 * rvc_send_can_message
 *============================================================================*/
static bool rvc_send_can_message(uint32_t can_id, const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    memset(tx_data, 0xFF, 8);  // pad with 0xFF

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        return false;
    }

    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    if (data && length > 0) {
        memcpy(tx_data, data, (length > 8) ? 8 : length);
    }

    HAL_StatusTypeDef res = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
    if (res != HAL_OK) {
        rvc_state.error_count++;
        return false;
    }
    return true;
}

/*============================================================================
 * Public Status and State Functions
 *============================================================================*/
void RVC_GetStats(uint32_t *messages, uint32_t *errors)
{
    if (messages) *messages = rvc_state.message_count;
    if (errors)   *errors   = rvc_state.error_count;
}

bool RVC_GetChannelState(uint8_t channel, bool *on_off, uint8_t *pwm_percent)
{
    if (!RVC_ValidateChannel(channel)) return false;
    if (on_off)      *on_off     = channel_states[channel].on_off_state;
    if (pwm_percent) *pwm_percent = channel_states[channel].pwm_percent;
    return true;
}

/*============================================================================
 * Private Helper Functions
 *============================================================================*/
static void RVC_UpdateChannelState(uint8_t channel, bool on_off, uint8_t pwm_percent)
{
    if (!RVC_ValidateChannel(channel)) return;
    channel_states[channel].on_off_state    = on_off;
    channel_states[channel].pwm_percent     = pwm_percent;
    channel_states[channel].last_command_ms = HAL_GetTick();
}

static void RVC_ApplyChannelControl(uint8_t channel)
{
    if (!RVC_ValidateChannel(channel)) return;
    channel_state_t *st = &channel_states[channel];

    // 1) Control VNQ7003 channel via Direct Input
    if (!VNQ7003_SetChannel_DI(channel, st->on_off_state)) {
        printf("RV-C: VNQ7003_SetChannel_DI failed for ch=%d\r\n", channel);
        rvc_state.error_count++;
        return;
    }

    // 2) Control PWM for dimming (only if channel is ON)
    if (st->on_off_state) {
        // Map 0..100% → 0..999 (assuming ARR=999)
        uint32_t duty = ((uint32_t)st->pwm_percent * 999U) / 100U;

        switch (channel) {
            case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty); break;
            case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty); break;
            case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty); break;
            case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty); break;
        }

        printf("RV-C: CH%d ON @ %d%% (PWM=%lu)\r\n",
               channel, st->pwm_percent, duty);
    } else {
        // Channel OFF: set PWM to 0
        switch (channel) {
            case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); break;
            case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); break;
            case 2: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); break;
            case 3: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); break;
        }

        printf("RV-C: CH%d OFF\r\n", channel);
    }
}

static bool RVC_ValidateChannel(uint8_t channel)
{
    return (channel < 4);
}

static void RVC_SendStatusResponse(uint16_t dgn, uint8_t channel)
{
    // TODO: Implement proper status response transmission
    printf("RV-C: (TODO) Send status response for DGN=0x%04X, ch=%d\r\n", dgn, channel);
}
