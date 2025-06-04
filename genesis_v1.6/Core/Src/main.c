/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Compatible with Existing Codebase
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rvc_cmd.h"
#include "rvc_defs.h"
#include "vnq7003sys.h"        // Use the unified driver
#include "vnq7003sys_regs.h"   // Use the correct filename
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void check_fdcan_status(void);
void test_manual_command(void);
void print_system_status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Printf redirection to UART3 - REQUIRED FOR CONSOLE OUTPUT
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    if (ptr && len > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        return len;
    }
    return 0;
}
#endif

// Check FDCAN status periodically
void check_fdcan_status(void)
{
    static uint32_t last_check = 0;
    if (HAL_GetTick() - last_check > 5000) {  // Every 5 seconds
        uint32_t messages, errors;
        RVC_GetStats(&messages, &errors);
        printf("RV-C Stats: %lu msgs, %lu errors\r\n", messages, errors);

        // Also check VNQ7003 status
        vnq_status_t vnq_status = vnq_get_status();
        printf("VNQ7003 Status: %s\r\n",
               vnq_status == VNQ_STATUS_HEALTHY ? "HEALTHY" :
               vnq_status == VNQ_STATUS_ERROR ? "ERROR" :
               vnq_status == VNQ_STATUS_FAILSAFE ? "FAILSAFE" : "COMM_ERROR");

        last_check = HAL_GetTick();
    }
}

// Test manual command to verify logic works
void test_manual_command(void)
{
    static bool test_done = false;
    static uint32_t test_start = 0;

    if (!test_done && HAL_GetTick() > 15000) {  // Wait 15 seconds after startup
        printf("=== Testing Manual Load Command ===\r\n");
        uint8_t test_data[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        RVC_HandleLoadCommand(0x1FFBC, test_data, 8);
        test_start = HAL_GetTick();
        test_done = true;
    }

    // Turn off after 3 seconds
    if (test_done && test_start > 0 && HAL_GetTick() - test_start > 3000) {
        printf("=== Turning Off Test Load ===\r\n");
        uint8_t test_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        RVC_HandleLoadCommand(0x1FFBC, test_data, 8);
        test_start = 0;  // Prevent repeat
    }
}

// Enhanced system status monitoring
void print_system_status(void)
{
    static uint32_t last_status = 0;
    if (HAL_GetTick() - last_status > 10000) {  // Every 10 seconds
        printf("\r\n=== System Status Report ===\r\n");
        printf("Uptime: %lu seconds\r\n", HAL_GetTick() / 1000);

        // VNQ7003 detailed status
        vnq_print_simple_status();

        // RV-C statistics
        uint32_t messages, errors;
        RVC_GetStats(&messages, &errors);
        printf("RV-C: %lu messages processed, %lu errors\r\n", messages, errors);

        // Channel states
        for (uint8_t ch = 0; ch < 4; ch++) {
            bool on_off;
            uint8_t pwm_percent;
            if (RVC_GetChannelState(ch, &on_off, &pwm_percent)) {
                printf("Channel %d: %s @ %d%%\r\n",
                       ch, on_off ? "ON" : "OFF", pwm_percent);
            }
        }

        printf("========================\r\n\r\n");
        last_status = HAL_GetTick();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Small delay to let console settle
  HAL_Delay(100);

  printf("\r\n=== RV-C DC Load Controller - STARTUP ===\r\n");
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  printf("Build: %s %s\r\n", __DATE__, __TIME__);
  printf("Initializing subsystems...\r\n");

  // Initialize VNQ7003 driver first with enhanced diagnostics
  printf("Initializing VNQ7003 with diagnostics...\r\n");

  // Try the enhanced initialization for better diagnostics
  if (!VNQ7003_Init_For_DI_Control()) {
      printf("ERROR: VNQ7003 standard initialization failed!\r\n");
      printf("Trying basic initialization as fallback...\r\n");

      // Fallback to basic init
      if (!vnq_init()) {
          printf("CRITICAL: VNQ7003 completely failed to initialize!\r\n");
          printf("Check hardware connections and power supply\r\n");
          Error_Handler();
      }
  }
  printf("VNQ7003 initialized successfully\r\n");

  // Initialize RV-C command handling
  printf("Initializing RV-C command handler...\r\n");
  RVC_Init();
  printf("RV-C system ready\r\n");

  printf("\r\n=== SYSTEM READY ===\r\n");
  printf("Listening for RV-C commands on address 0x%02X\r\n", RVC_SRC_ADDR);
  printf("Test commands:\r\n");
  printf("  Load ON:  CAN ID 0x19FFBCFF, Data: 00 01 00 00 00 00 00 00\r\n");
  printf("  Load OFF: CAN ID 0x19FFBCFF, Data: 00 00 00 00 00 00 00 00\r\n");
  printf("  Dimmer:   CAN ID 0x19FFB9FF, Data: 00 32 00 00 00 00 00 00 (50%%)\r\n");
  printf("=====================\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // CRITICAL: Service VNQ7003 watchdog every loop iteration
    vnq_task();

    // Check FDCAN and system status periodically
    check_fdcan_status();

    // Test manual command (for testing - can be commented out)
    test_manual_command();

    // Print comprehensive system status
    print_system_status();

    // Short delay to prevent overwhelming the system
    HAL_Delay(5);  // 5ms loop timing
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_GPIO_Port, SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SEL_Pin */
  GPIO_InitStruct.Pin = SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP_1_Pin DIP_0_Pin */
  GPIO_InitStruct.Pin = DIP_1_Pin|DIP_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_STBY_Pin */
  GPIO_InitStruct.Pin = CAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STBY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  printf("CRITICAL ERROR: System halted\r\n");
  while (1)
  {
    // Flash LED if available to indicate error
    #ifdef LED_RED_GPIO_Port
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_Delay(200);
    #else
    // Just wait if no LED available
    HAL_Delay(1000);
    #endif
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Assert failed: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
