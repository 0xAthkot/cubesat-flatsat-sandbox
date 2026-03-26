/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "nrf24l01p.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// CAN message IDs
#define CAN_ID_CDH_CMD         0x100  // Commands from CDH to EPS (button, etc.)
#define CAN_ID_CDH_STATUS_REQ  0x101  // CDH asks EPS for status
#define CAN_ID_EPS_CMD         0x200  // Commands from EPS to CDH (button, etc.)
#define CAN_ID_EPS_TELEM       0x201  // Telemetry from EPS (temperature, etc.)
#define CAN_ID_EPS_STATUS_RSP  0x202  // EPS responds with LED state

// Determine TX ID based on node type
#ifdef NODE_CDH
  #define CAN_TX_ID  CAN_ID_CDH_CMD
#endif

#ifdef NODE_EPS
  #define CAN_TX_ID  CAN_ID_EPS_CMD
#endif

// Sanity check
#if !defined(NODE_CDH) && !defined(NODE_EPS)
  #error "Define NODE_CDH or NODE_EPS in CMake"
#endif

// NRF24 settings (must match Raspberry Pi)
#define NRF_CHANNEL     2476  // RF channel in MHz (2400 + 76)
#define NRF_DATA_RATE   _2Mbps

// NRF24 payload message types (byte 0) - CDH to ground
#define NRF_MSG_BUTTON  0x01
#define NRF_MSG_TEMP    0x02

// Ground command types (byte 0) - ground to CDH
#define GND_CMD_TOGGLE_CDH_LED  0x01
#define GND_CMD_TOGGLE_EPS_LED  0x02
#define GND_CMD_STATUS_REQ      0x03

// CDH to ground response types
#define NRF_MSG_STATUS          0x03

// EPS alive timeout (ms)
#define EPS_ALIVE_TIMEOUT       5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t prev_button_state = 0;
volatile uint8_t can_rx_flag = 0;
volatile uint8_t can_rx_led_state = 0;
volatile uint8_t can_rx_telem_flag = 0;
volatile int16_t can_rx_temperature = 0;  // Temperature in tenths of °C
volatile uint8_t last_eps_led_cmd = 0;    // Last LED state sent to EPS
volatile uint32_t last_eps_can_tick = 0;  // Tick of last CAN message from EPS
#ifdef NODE_CDH
volatile uint8_t eps_status_flag = 0;     // EPS status response received
volatile uint8_t eps_status_led = 0;      // EPS LED state from status response
#endif
#ifdef NODE_EPS
volatile uint8_t eps_status_req_flag = 0; // CDH requested our status
#endif

// CAN task
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

#ifdef NODE_CDH
// NRF TX queue: CAN task -> Radio task
osMessageQueueId_t nrfTxQueueHandle;

// Radio task
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartCanTask(void *argument);
#ifdef NODE_CDH
void StartRadioTask(void *argument);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void can_tx_header_init(FDCAN_TxHeaderTypeDef *hdr, uint32_t id, uint32_t dlc)
{
  hdr->Identifier = id;
  hdr->IdType = FDCAN_STANDARD_ID;
  hdr->TxFrameType = FDCAN_DATA_FRAME;
  hdr->DataLength = dlc;
  hdr->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr->BitRateSwitch = FDCAN_BRS_OFF;
  hdr->FDFormat = FDCAN_CLASSIC_CAN;
  hdr->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr->MessageMarker = 0;
}

#ifdef NODE_CDH
// Send payload over NRF24, returns 1 on success, 0 on failure
static uint8_t nrf24_send_and_listen(uint8_t *payload)
{
  uint8_t success = 0;

  // Switch to TX mode
  nrf24l01p_power_down();
  nrf24l01p_ptx_mode();
  nrf24l01p_power_up();
  osDelay(2);  // 1.5ms power-up delay

  nrf24l01p_tx_transmit(payload);

  // Poll for TX_DS (success) or MAX_RT (failure)
  for (int i = 0; i < 10; i++)
  {
    osDelay(1);
    uint8_t status = nrf24l01p_get_status();
    if (status & 0x20)  // TX_DS: transmit succeeded
    {
      nrf24l01p_clear_tx_ds();
      success = 1;
      break;
    }
    if (status & 0x10)  // MAX_RT: max retransmits, no ACK
    {
      nrf24l01p_clear_max_rt();
      nrf24l01p_flush_tx_fifo();
      break;
    }
  }

  // Switch back to RX mode
  nrf24l01p_power_down();
  nrf24l01p_prx_mode();
  nrf24l01p_power_up();
  osDelay(2);

  return success;
}
#endif

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
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC for accurate temperature readings
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Configure RX filter
  FDCAN_FilterTypeDef filterConfig;
  filterConfig.IdType = FDCAN_STANDARD_ID;
  filterConfig.FilterIndex = 0;
  filterConfig.FilterType = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
#ifdef NODE_CDH
  // Accept 0x200-0x203 (mask 0x7FC ignores bits 0-1)
  filterConfig.FilterID1 = 0x200;
  filterConfig.FilterID2 = 0x7FC;
#endif
#ifdef NODE_EPS
  // Accept 0x100-0x101 (mask 0x7FE ignores bit 0)
  filterConfig.FilterID1 = 0x100;
  filterConfig.FilterID2 = 0x7FE;
#endif
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable RX FIFO 0 new message interrupt
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  // Start FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
#ifdef NODE_CDH
  nrfTxQueueHandle = osMessageQueueNew(4, NRF24L01P_PAYLOAD_LENGTH, NULL);
#endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);
#ifdef NODE_CDH
  radioTaskHandle = osThreadNew(StartRadioTask, NULL, &radioTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_C13_Btn_Pin */
  GPIO_InitStruct.Pin = KEY_C13_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_C13_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C6_LED_Builtin_Pin */
  GPIO_InitStruct.Pin = C6_LED_Builtin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(C6_LED_Builtin_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// FDCAN RX interrupt callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      uint32_t id = rxHeader.Identifier;

#ifdef NODE_CDH
      if (id == CAN_ID_EPS_CMD)
      {
        can_rx_led_state = rxData[0];
        can_rx_flag = 1;
        last_eps_can_tick = HAL_GetTick();
      }
      else if (id == CAN_ID_EPS_TELEM)
      {
        can_rx_temperature = (int16_t)(rxData[0] | (rxData[1] << 8));
        can_rx_telem_flag = 1;
        last_eps_can_tick = HAL_GetTick();
      }
      else if (id == CAN_ID_EPS_STATUS_RSP)
      {
        eps_status_led = rxData[0];
        eps_status_flag = 1;
        last_eps_can_tick = HAL_GetTick();
      }
#endif

#ifdef NODE_EPS
      if (id == CAN_ID_CDH_CMD)
      {
        can_rx_led_state = rxData[0];
        can_rx_flag = 1;
      }
      else if (id == CAN_ID_CDH_STATUS_REQ)
      {
        eps_status_req_flag = 1;
      }
#endif
    }
  }
}

void StartCanTask(void *argument)
{
#ifdef NODE_CDH
  char *node_name = "CDH";
#else
  char *node_name = "EPS";
#endif

  for(;;)
  {
    // Read button and send over CAN on change
    uint8_t current_button_state = HAL_GPIO_ReadPin(KEY_C13_Btn_GPIO_Port, KEY_C13_Btn_Pin);

    if (current_button_state != prev_button_state)
    {
      prev_button_state = current_button_state;

      FDCAN_TxHeaderTypeDef txHeader;
      can_tx_header_init(&txHeader, CAN_TX_ID, FDCAN_DLC_BYTES_1);

      uint8_t txData[1] = { current_button_state };
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

#ifdef NODE_CDH
      last_eps_led_cmd = current_button_state;
#endif

      char msg[50];
      sprintf(msg, "[%s] CAN  INFO: TX Button: %d\r\n", node_name, current_button_state);
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

#ifdef NODE_CDH
      // Queue button state for radio task
      uint8_t nrf_btn[NRF24L01P_PAYLOAD_LENGTH] = {0};
      nrf_btn[0] = NRF_MSG_BUTTON;
      nrf_btn[1] = current_button_state;
      osMessageQueuePut(nrfTxQueueHandle, nrf_btn, 0, 0);
#endif
    }

    // Handle received CAN messages
    if (can_rx_flag)
    {
      can_rx_flag = 0;
      HAL_GPIO_WritePin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin, can_rx_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

      char msg[50];
      sprintf(msg, "[%s] CAN  INFO: RX LED: %d\r\n", node_name, can_rx_led_state);
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }

#ifdef NODE_CDH
    // Cache EPS temperature (sent to ground via status response)
    if (can_rx_telem_flag)
    {
      can_rx_telem_flag = 0;

      char msg[60];
      sprintf(msg, "[CDH] CAN  INFO: EPS temp: %d.%d C\r\n",
              can_rx_temperature / 10,
              can_rx_temperature >= 0 ? can_rx_temperature % 10 : (-can_rx_temperature) % 10);
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }
#endif

#ifdef NODE_EPS
    // Read internal temperature and send over CAN every 2 seconds
    {
      static uint32_t last_telem_tick = 0;
      uint32_t now = osKernelGetTickCount();
      if (now - last_telem_tick >= 2000)
      {
        last_telem_tick = now;

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
          uint32_t raw = HAL_ADC_GetValue(&hadc1);
          int16_t temp_x10 = (int16_t)(__HAL_ADC_CALC_TEMPERATURE(3300, raw, ADC_RESOLUTION_12B) * 10);

          FDCAN_TxHeaderTypeDef txHeader;
          can_tx_header_init(&txHeader, CAN_ID_EPS_TELEM, FDCAN_DLC_BYTES_2);

          uint8_t txData[2] = { (uint8_t)(temp_x10 & 0xFF), (uint8_t)((temp_x10 >> 8) & 0xFF) };
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

          char msg[50];
          sprintf(msg, "[EPS] CAN  INFO: Temp: %d.%d C\r\n", temp_x10 / 10,
                  temp_x10 >= 0 ? temp_x10 % 10 : (-temp_x10) % 10);
          CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        }
        HAL_ADC_Stop(&hadc1);
      }
    }

    // Respond to CDH status request with current LED state
    if (eps_status_req_flag)
    {
      eps_status_req_flag = 0;

      uint8_t led_state = HAL_GPIO_ReadPin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin);

      FDCAN_TxHeaderTypeDef txHeader;
      can_tx_header_init(&txHeader, CAN_ID_EPS_STATUS_RSP, FDCAN_DLC_BYTES_1);

      uint8_t txData[1] = { led_state };
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

      char msg[50];
      sprintf(msg, "[EPS] CAN  INFO: Status RSP LED: %d\r\n", led_state);
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }
#endif

    osDelay(10);
  }
}

#ifdef NODE_CDH
void StartRadioTask(void *argument)
{
  // Initialize NRF24 in RX mode (listening for ground station)
  nrf24l01p_rx_init(NRF_CHANNEL, NRF_DATA_RATE);

  for(;;)
  {
    // Check for outbound payloads from CAN task
    uint8_t nrf_payload[NRF24L01P_PAYLOAD_LENGTH];
    if (osMessageQueueGet(nrfTxQueueHandle, nrf_payload, NULL, 10) == osOK)
    {
      if (nrf24_send_and_listen(nrf_payload))
      {
        const char *ok = "[CDH] RF   INFO: TX OK\r\n";
        CDC_Transmit_FS((uint8_t*)ok, strlen(ok));
      }
      else
      {
        const char *err = "[CDH] RF   WARN: TX failed (no ACK)\r\n";
        CDC_Transmit_FS((uint8_t*)err, strlen(err));
      }
    }

    // Drain all packets from NRF24 RX FIFO
    while (!(nrf24l01p_get_fifo_status() & 0x01))  // bit 0 = RX_EMPTY
    {
      uint8_t nrf_rx[NRF24L01P_PAYLOAD_LENGTH] = {0};
      nrf24l01p_rx_receive(nrf_rx);

      char msg[80];
      sprintf(msg, "[CDH] RF   INFO: GND RX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              nrf_rx[0], nrf_rx[1], nrf_rx[2], nrf_rx[3],
              nrf_rx[4], nrf_rx[5], nrf_rx[6], nrf_rx[7]);
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

      if (nrf_rx[0] == GND_CMD_TOGGLE_CDH_LED)
      {
        HAL_GPIO_TogglePin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin);
        uint8_t state = HAL_GPIO_ReadPin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin);
        sprintf(msg, "[CDH] RF   INFO: CDH LED toggled: %d\r\n", state);
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      }
      else if (nrf_rx[0] == GND_CMD_TOGGLE_EPS_LED)
      {
        last_eps_led_cmd ^= 1;

        FDCAN_TxHeaderTypeDef txHeader;
        can_tx_header_init(&txHeader, CAN_ID_CDH_CMD, FDCAN_DLC_BYTES_1);

        uint8_t txData[1] = { last_eps_led_cmd };
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

        sprintf(msg, "[CDH] RF   INFO: EPS LED toggled: %d\r\n", last_eps_led_cmd);
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      }
      else if (nrf_rx[0] == GND_CMD_STATUS_REQ)
      {
        // Ask EPS for its actual LED state via CAN
        eps_status_flag = 0;
        {
          FDCAN_TxHeaderTypeDef txHeader;
          can_tx_header_init(&txHeader, CAN_ID_CDH_STATUS_REQ, FDCAN_DLC_BYTES_0);
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, NULL);
        }

        // Wait up to 50ms for EPS response
        uint8_t eps_led = 0xFF;  // 0xFF = unknown / CAN down
        uint8_t eps_alive = 0;
        for (int i = 0; i < 50; i++)
        {
          if (eps_status_flag)
          {
            eps_led = eps_status_led;
            eps_alive = 1;
            break;
          }
          osDelay(1);
        }

        uint8_t cdh_led = HAL_GPIO_ReadPin(C6_LED_Builtin_GPIO_Port, C6_LED_Builtin_Pin);

        uint8_t resp[NRF24L01P_PAYLOAD_LENGTH] = {0};
        resp[0] = NRF_MSG_STATUS;
        resp[1] = cdh_led;
        resp[2] = eps_led;
        resp[3] = (uint8_t)(can_rx_temperature & 0xFF);
        resp[4] = (uint8_t)((can_rx_temperature >> 8) & 0xFF);
        resp[5] = eps_alive;

        nrf24_send_and_listen(resp);

        sprintf(msg, "[CDH] RF   INFO: Status sent (EPS: %s)\r\n",
                eps_alive ? "alive" : "CAN down");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
      }
    }
  }
}
#endif

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */

#ifdef NODE_CDH
  char *node_name = "CDH";
#else
  char *node_name = "EPS";
#endif

  // Startup message
  char boot_msg[40];
  sprintf(boot_msg, "[%s] SYS  INFO: node booted\r\n", node_name);
  osDelay(500);  // Let USB enumerate
  CDC_Transmit_FS((uint8_t*)boot_msg, strlen(boot_msg));

  // USB init done, nothing left for this task
  osThreadSuspend(NULL);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
