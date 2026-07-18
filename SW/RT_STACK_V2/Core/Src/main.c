/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "board_config.h"
#include "board_safety_stm32.h"
#include "diagnostics_can.h"
#include "engine_control.h"
#include "engine_watchdog_stm32.h"
#include "trigger_capture_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Injection/ignition types live in injection.c / spark.c; the hardware maps
 * live in engine_config.h. */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TMG_CH_COUNT is defined in trigger_decoder.h (sizes the tmg_* arrays).
 * Engine calibration (cylinders, channels, thresholds) is in engine_config.h. */

/* Measured analog reference voltage, in mV (ADC conversions) */
#define VDDA_MV         3200U

enum
{
  ADC1_IDX_COMP1 = 0U,
  ADC1_IDX_COMP2 = 1U,
  ADC1_IDX_COMP3 = 2U,
  ADC1_IDX_VBUS  = 3U,
  ADC1_IDX_COMP6 = 4U,
  ADC1_IDX_TEMP  = 5U,
  ADC1_IDX_VREF  = 6U,
  ADC1_IDX_COMP4 = 7U,
  ADC3_IDX_COMP5 = 0U,
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp5;
COMP_HandleTypeDef hcomp6;
COMP_HandleTypeDef hcomp7;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DAC_HandleTypeDef hdac3;
DAC_HandleTypeDef hdac4;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
volatile uint16_t ADC_VAL_adc1[8];
volatile uint16_t ADC_VAL_adc3[1];
static uint16_t adc2_vbatt_raw;

typedef struct
{
  uint16_t comp1_raw;
  uint16_t comp2_raw;
  uint16_t comp3_raw;
  uint16_t comp4_raw;
  uint16_t comp5_raw;
  uint16_t comp6_raw;
  uint16_t vbus_raw;
  uint16_t temp_raw;
  uint16_t vref_raw;
  uint16_t vbat_raw;
} adc_snapshot_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_COMP4_Init(void);
static void MX_COMP5_Init(void);
static void MX_COMP6_Init(void);
static void MX_COMP7_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_DAC3_Init(void);
static void MX_DAC4_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_COMP1_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Force boot from flash via option bytes (one-shot).
 *
 * The board has no pull-down on BOOT0/PB8: if the pin samples high at reset
 * the ST bootloader runs and the application is not executed. This sets
 * nSWBOOT0=0 (boot ignores the pin) and nBOOT0=1 (boot from flash). If the
 * option bytes are already correct it does nothing; otherwise it programs them
 * and HAL_FLASH_OB_Launch() triggers a reset (does not return).
 */
static void boot_from_flash_ob_init(void)
{
  if (((FLASH->OPTR & FLASH_OPTR_nSWBOOT0) == 0U) &&
      ((FLASH->OPTR & FLASH_OPTR_nBOOT0) != 0U))
  {
    return;   /* already configured: always boots from flash */
  }

  FLASH_OBProgramInitTypeDef ob = {0};

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  ob.OptionType = OPTIONBYTE_USER;
  ob.USERType   = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
  ob.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;

  if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK)
  {
    HAL_FLASH_OB_Launch();   /* apply and reset the MCU: does not return */
  }

  /* Programming failed: re-lock the flash and signal */
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
  Error_Handler();
}

static void injector_comparator_hysteresis_init(void)
{
  COMP_HandleTypeDef *const comparators[] = {
    &hcomp1, &hcomp2, &hcomp3, &hcomp4, &hcomp5, &hcomp6,
  };

  for (uint32_t i = 0U; i < (sizeof(comparators) / sizeof(comparators[0])); ++i)
  {
    comparators[i]->Init.Hysteresis = BOARD_INJECTOR_COMPARATOR_HYSTERESIS;
    MODIFY_REG(comparators[i]->Instance->CSR, COMP_CSR_HYST,
               BOARD_INJECTOR_COMPARATOR_HYSTERESIS);
  }
}

static adc_snapshot_t adc_snapshot_read(void)
{
  if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != 0U)
  {
    adc2_vbatt_raw = (uint16_t)HAL_ADC_GetValue(&hadc2);
  }

  adc_snapshot_t adc = {
    .comp1_raw = ADC_VAL_adc1[ADC1_IDX_COMP1],
    .comp2_raw = ADC_VAL_adc1[ADC1_IDX_COMP2],
    .comp3_raw = ADC_VAL_adc1[ADC1_IDX_COMP3],
    .comp4_raw = ADC_VAL_adc1[ADC1_IDX_COMP4],
    .comp5_raw = ADC_VAL_adc3[ADC3_IDX_COMP5],
    .comp6_raw = ADC_VAL_adc1[ADC1_IDX_COMP6],
    .vbus_raw  = ADC_VAL_adc1[ADC1_IDX_VBUS],
    .temp_raw  = ADC_VAL_adc1[ADC1_IDX_TEMP],
    .vref_raw  = ADC_VAL_adc1[ADC1_IDX_VREF],
    .vbat_raw  = adc2_vbatt_raw,
  };

  return adc;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Force the vector table into flash. With VTOR=0 (G4 default) the vectors
   * depend on how BOOT0 (PB8) is sampled at reset: if it is not pulled low,
   * exceptions land in the ST bootloader's table and SysTick/EXTI never reach
   * the application handlers. */
  SCB->VTOR = FLASH_BASE;
  __DSB();
  /* The PCB has no external actuator pull-downs/global enable.  Reduce the
   * high-impedance boot window before HAL, clocks, or option-byte work. */
  board_force_actuator_pins_low_early();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  boot_from_flash_ob_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* The current clock source is HSI16.  Do not enable HSE clock security until
   * the fitted crystal frequency has been confirmed and SystemClock_Config
   * has intentionally been migrated to HSE. */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_COMP4_Init();
  MX_COMP5_Init();
  MX_COMP6_Init();
  MX_COMP7_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_DAC3_Init();
  MX_DAC4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_COMP1_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /* CubeMX currently emits COMP_HYSTERESIS_NONE. Apply the board-level value
   * from a preserved user block before injection can start a comparator. */
  injector_comparator_hysteresis_init();

  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
  /* ADC DMA priority at the minimum (the IRQs stay disabled anyway) */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 0);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 15, 0);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VAL_adc1, 8U) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);

  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ADC_VAL_adc3, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_HT | DMA_IT_TC);

  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

#if (BOARD_FDCAN1_TRANSCEIVER_WIRING_FIXED != 0U)
  if ((diagnostics_can_init(&hfdcan1) != HAL_OK) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
  {
    Error_Handler();
  }
#endif

  /* Initialize the consumer before enabling any trigger producer.  TIM2 is a
   * common clock for captured/EXTI edges and actuator hard deadlines. */
  const trigger_decoder_config_t *trigger_config =
      trigger_decoder_default_config();
  (void)engine_control_init(trigger_config, trigger_capture_timestamp_hz());
  /* Start supervision before trigger IRQs. A noisy/stuck input must not be
   * able to starve foreground activation of the watchdog. */
  if (!engine_watchdog_stm32_start())
  {
    engine_control_emergency_fault_isr(ENGINE_FAULT_PLATFORM);
    Error_Handler();
  }
  if (!trigger_capture_stm32_init(trigger_config))
  {
    engine_control_emergency_fault_isr(ENGINE_FAULT_PLATFORM);
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    engine_control_state_t engine;
    engine_control_service();
    engine_control_get_state(&engine);

#if (BOARD_FDCAN1_TRANSCEIVER_WIRING_FIXED != 0U)
    adc_snapshot_t adc = adc_snapshot_read();
    uint32_t vdda_mv = (adc.vref_raw != 0U) ?
        __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc.vref_raw, ADC_RESOLUTION_12B) :
        VDDA_MV;
    int32_t temp_celsius = __HAL_ADC_CALC_TEMPERATURE(
        vdda_mv, adc.temp_raw, ADC_RESOLUTION_12B);

    diagnostics_sensor_snapshot_t sensors = {
      .mcu_temperature_c = temp_celsius,
      .external_vbatt_raw = adc.vbat_raw,
      .vbus_raw = adc.vbus_raw,
      .vdda_mv = (uint16_t)vdda_mv,
    };
    diagnostics_can_service(HAL_GetTick(), &engine, &sensors);
#else
    /* Clear ADC2 EOC/overrun and keep its latest sample available to a
     * debugger even while the unsafe CAN route is disabled. */
    (void)adc_snapshot_read();
#endif

    /* Refresh only after the complete foreground control/diagnostics pass.
     * A deadlock or unbounded service therefore causes a hardware reset. */
    engine_watchdog_stm32_refresh();

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  /* External VBATT divider: allow its source impedance to settle. */
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */
  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */
  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */
  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */
  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */
  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */
  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */
  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */
  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */
  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */
  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */
  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */
  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief COMP5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP5_Init(void)
{

  /* USER CODE BEGIN COMP5_Init 0 */
  /* USER CODE END COMP5_Init 0 */

  /* USER CODE BEGIN COMP5_Init 1 */
  /* USER CODE END COMP5_Init 1 */
  hcomp5.Instance = COMP5;
  hcomp5.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp5.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH1;
  hcomp5.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp5.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp5.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp5.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP5_Init 2 */
  /* USER CODE END COMP5_Init 2 */

}

/**
  * @brief COMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */
  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */
  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp6.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH2;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */
  /* USER CODE END COMP6_Init 2 */

}

/**
  * @brief COMP7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP7_Init(void)
{

  /* USER CODE BEGIN COMP7_Init 0 */
  /* USER CODE END COMP7_Init 0 */

  /* USER CODE BEGIN COMP7_Init 1 */
  /* USER CODE END COMP7_Init 1 */
  hcomp7.Instance = COMP7;
  hcomp7.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp7.Init.InputMinus = COMP_INPUT_MINUS_DAC2_CH1;
  hcomp7.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp7.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp7.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp7.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP7_Init 2 */
  /* USER CODE END COMP7_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */
  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */
  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */
  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */
  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */
  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */
  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */
  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief DAC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC4_Init(void)
{

  /* USER CODE BEGIN DAC4_Init 0 */
  /* USER CODE END DAC4_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC4_Init 1 */
  /* USER CODE END DAC4_Init 1 */

  /** DAC Initialization
  */
  hdac4.Instance = DAC4;
  if (HAL_DAC_Init(&hdac4) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac4, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac4, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC4_Init 2 */
  /* USER CODE END DAC4_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
#if (BOARD_FDCAN1_TRANSCEIVER_WIRING_FIXED == 0U)
  /* Leave both contested logic-side CAN nets passive on the affected PCB.
   * This early return is in a CubeMX user block so regeneration preserves the
   * electrical safety gate even though the .ioc still describes FDCAN1. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  return;
#endif
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
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 29;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */
  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */
  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */
  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */
  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SEL12_Pin|SEL34_Pin|SEL56_Pin|SEL78_Pin
                          |SEL1112_Pin|IGN2_Pin|IGN1_Pin|IGN4_Pin
                          |IGN3_Pin|IGN6_Pin|IGN5_Pin|IGN8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEL910_Pin|BOOST34_Pin|BOOST12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BOOST910_Pin|BOOST1112_Pin|BOOST78_Pin|BOOST56_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IGN7_Pin|IGN10_Pin|IGN9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, IGN12_Pin|IGN11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEL12_Pin SEL34_Pin SEL56_Pin SEL78_Pin
                           SEL1112_Pin IGN2_Pin IGN1_Pin IGN4_Pin
                           IGN3_Pin IGN6_Pin IGN5_Pin IGN8_Pin */
  GPIO_InitStruct.Pin = SEL12_Pin|SEL34_Pin|SEL56_Pin|SEL78_Pin
                          |SEL1112_Pin|IGN2_Pin|IGN1_Pin|IGN4_Pin
                          |IGN3_Pin|IGN6_Pin|IGN5_Pin|IGN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL910_Pin BOOST34_Pin BOOST12_Pin */
  GPIO_InitStruct.Pin = SEL910_Pin|BOOST34_Pin|BOOST12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOST910_Pin BOOST1112_Pin BOOST78_Pin BOOST56_Pin */
  GPIO_InitStruct.Pin = BOOST910_Pin|BOOST1112_Pin|BOOST78_Pin|BOOST56_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IGN7_Pin IGN10_Pin IGN9_Pin */
  GPIO_InitStruct.Pin = IGN7_Pin|IGN10_Pin|IGN9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IGN12_Pin IGN11_Pin */
  GPIO_InitStruct.Pin = IGN12_Pin|IGN11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Trigger pins start as passive inputs.  trigger_capture_stm32_init() later
   * applies the enabled channels and edge polarity from the wheel list. */
  GPIO_InitStruct.Pin = TMG_OUT1_Pin|TMG_OUT6_Pin|TMG_OUT2_Pin|TMG_OUT7_Pin
                          |TMG_OUT4_Pin|TMG_OUT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TMG_OUT3_Pin */
  GPIO_InitStruct.Pin = TMG_OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TMG_OUT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMG_OUT9_Pin TMG_OUT5_Pin */
  GPIO_InitStruct.Pin = TMG_OUT9_Pin|TMG_OUT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Trap: do not continue with clock/peripherals in an undefined state.
   * Note: this can be called before MX_GPIO_Init (e.g. SystemClock_Config
   * failure), so do not drive GPIO/LED here. */
  engine_control_emergency_fault_isr(ENGINE_FAULT_PLATFORM);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
