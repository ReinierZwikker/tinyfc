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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "main_conf.h"
#include "crsf.h"
#include "crsf_parser.h"
#include "usbd_cdc_if.h"
#include "utils.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
crsf_parser_t crsf_parser;
uart_buffer_t uart3_rb = {0};
uint8_t *uart3_rx_char;

crsf_frame_link_stats_t radio_link_stats;
uint32_t radio_link_stats_last_update = UINT32_MAX;

uint16_t received_crsf_channels[CRSF_CHANNEL_COUNT];

int16_t current_actuator_channels[ACTUATOR_CHANNEL_COUNT];

uint16_t output_actuator_channels[ACTUATOR_CHANNEL_COUNT];

int16_t previous_rotor_throttle = -NORM_RANGE;

uint8_t armed = false;
uint8_t pre_armed = false;
uint8_t arm_blocked = true;

enum flight_mode_t {
  FLIGHT_MODE_DIRECT = 0,
  FLIGHT_MODE_RATE = 1,
  FLIGHT_MODE_ANGLE = 2,
  FLIGHT_MODE_POSITION = 3
} flight_mode = FLIGHT_MODE_DIRECT;

enum pit_mode_t {
  PIT_MODE_OFF = 0,
  PIT_MODE_ON = 1,
} pit_mode = PIT_MODE_OFF;

enum led_state_t {
  LED_STATE_OFF = 0,
  LED_STATE_ON = 1,
} status_led_state = LED_STATE_OFF;

uint8_t led_blinks = 0;
uint32_t led_blink_time_next_change = 0;
enum led_blink_state_t {
  LED_BLINK_WAITING = 0,
  LED_BLINK_PRE_OFF = 1,
  LED_BLINK_ON = 2,
  LED_BLINK_POST_OFF = 3,
} led_blink_state = LED_BLINK_WAITING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void enable_status_led();
void disable_status_led();
void set_status_led(enum led_state_t state);
void add_blinks(uint8_t blinks);
void send_debug_message(char *name, uint8_t name_length, uint8_t id, int16_t value);
void set_PWM0_duty(uint16_t duty_cycle_hundredths);
void set_PWM1_duty(uint16_t duty_cycle_hundredths);
void set_PWM2_duty(uint16_t duty_cycle_hundredths);
void set_PWM3_duty(uint16_t pulse);
void set_PWM4_duty(uint16_t pulse);
//uint16_t get_PWM0_duty();
//uint16_t get_PWM1_duty();
//uint16_t get_PWM2_duty();
//uint16_t get_PWM3_duty();
//uint16_t get_PWM4_duty();
void update_PWM();
void update_Oneshot125();
void on_crsf_frame(uint8_t type,
                   uint8_t dest_addr, // 0 for non-extended frames
                   uint8_t orig_addr, // 0 for non-extended frames
                   const uint8_t *payload,
                   uint8_t payload_len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  set_status_led(LED_STATE_OFF);

  set_PWM0_duty(PWM_MID);
  set_PWM1_duty(PWM_MID);
  set_PWM2_duty(PWM_MID);
  set_PWM3_duty(ONESHOT125_DISARMED);
  set_PWM4_duty(ONESHOT125_DISARMED);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  crsf_parser_init(&crsf_parser, CRSF_ADDRESS_FC, on_crsf_frame);

  uart3_rx_char = (uint8_t *)malloc(8);
  HAL_UART_Receive_IT(&huart3, uart3_rx_char, 8);

  uint32_t last_pwm_update = HAL_GetTick();

  memset(received_crsf_channels, 0, sizeof(received_crsf_channels));
  memset(current_actuator_channels, 0, sizeof(current_actuator_channels));

  output_actuator_channels[ACTUATOR_SWASH_LEFT] = PWM_MID;
  output_actuator_channels[ACTUATOR_SWASH_RIGHT] = PWM_MID;
  output_actuator_channels[ACTUATOR_SWASH_AFT] = PWM_MID;
  output_actuator_channels[ACTUATOR_MAIN_ROTOR] = ONESHOT125_DISARMED;
  output_actuator_channels[ACTUATOR_TAIL_ROTOR] = ONESHOT125_DISARMED;


  uint8_t init_msg[] = "HELI FC Started up!\r\n";
  CDC_Transmit_FS(init_msg, sizeof(init_msg) - 1);

  add_blinks(2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {

    uint32_t current_tick = HAL_GetTick();

    // Process received data from CSRF/UART3
    if (uart3_rb.head != uart3_rb.tail)
    {
      uint16_t available = (uart3_rb.head - uart3_rb.tail + UART3_RX_BUFFER_SIZE) % UART3_RX_BUFFER_SIZE;

      if (available > 0)
      {
        uint8_t temp_buffer[UART3_RX_BUFFER_SIZE];
        uint16_t read_len = 0;

        // Extract data from ring buffer
        while (uart3_rb.tail != uart3_rb.head && read_len < available)
        {
          temp_buffer[read_len++] = uart3_rb.buffer[uart3_rb.tail];
          uart3_rb.tail = (uart3_rb.tail + 1) % UART3_RX_BUFFER_SIZE;
        }

        // Feed to CRSF parser
        crsf_parser_feed(&crsf_parser, temp_buffer, read_len);
      }
    }

    // Handle periodic PWM update
    if (current_tick - last_pwm_update > PWM_UPDATE_PERIOD_MS) {
      last_pwm_update = current_tick;
      update_PWM();
      update_Oneshot125();
    }

    // Handle LED blinking
    switch (led_blink_state) {
      default:
      case LED_BLINK_WAITING:
        if (led_blinks > 0) {
          led_blinks--;
          led_blink_state = LED_BLINK_PRE_OFF;
          set_status_led(LED_STATE_OFF);
          led_blink_time_next_change = current_tick + BLINK_TIME;
        }
        break;
      case LED_BLINK_PRE_OFF:
        if (current_tick >= led_blink_time_next_change) {
          led_blink_state = LED_BLINK_ON;
          set_status_led(LED_STATE_ON);
          led_blink_time_next_change = current_tick + BLINK_TIME * 2;
        }
        break;
      case LED_BLINK_ON:
        if (current_tick >= led_blink_time_next_change) {
          led_blink_state = LED_BLINK_POST_OFF;
          set_status_led(LED_STATE_OFF);
          led_blink_time_next_change = current_tick + BLINK_TIME;
        }
        break;
      case LED_BLINK_POST_OFF:
        if (current_tick >= led_blink_time_next_change) {
          led_blink_state = LED_BLINK_WAITING;
          set_status_led(status_led_state);
        }
        break;
    }

    if (arm_blocked) {
      if (led_blinks == 0) { led_blinks = 1; }
    }

//    if (current_tick % 1000 == 0) {
//      CDC_Transmit_FS((uint8_t *)"DEBUG", 5);
//
//      for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; i++) {
//        send_debug_message("RADIO", 5, i, (int16_t) received_crsf_channels[i]);
//      }
//      for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++) {
//        send_debug_message("PWM  ", 5, i, (int16_t) current_pwm_channels[i]);
//      }
//
//      send_debug_message("LINK LASTMES", 12, 0, (int16_t) (current_tick - radio_link_stats_last_update));
//      send_debug_message("LINK SNR UP ", 12, 1, (int16_t) radio_link_stats.snr_uplink_db);
//      send_debug_message("LINK RSS UP1", 12, 2, (int16_t) radio_link_stats.rssi_uplink_ant1_dbm);
//      send_debug_message("LINK RSS UP2", 12, 3, (int16_t) radio_link_stats.rssi_uplink_ant2_dbm);
//      send_debug_message("LINK QLY UP ", 12, 4, (int16_t) radio_link_stats.link_quality_percentage_uplink);
//      send_debug_message("LINK SNR DW ", 12, 5, (int16_t) radio_link_stats.snr_downlink_db);
//      send_debug_message("LINK RSS DW ", 12, 6, (int16_t) radio_link_stats.rssi_downlink_dbm);
//      send_debug_message("LINK QLY DW ", 12, 7, (int16_t) radio_link_stats.link_quality_percentage_downlink);
//
//      send_debug_message("ARMED", 5, 0, (int16_t) armed);
//      send_debug_message("BLOCK", 5, 1, (int16_t) arm_blocked);
//
//      CDC_Transmit_FS((uint8_t *)"\r\n\r\n\r\n", 6);
//
//    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 49;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1920;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 23999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 6000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart3.Init.BaudRate = 420000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// LED FUNCTIONS
void enable_status_led() {
  status_led_state = LED_STATE_ON;
  set_status_led(status_led_state);
}
void disable_status_led() {
  status_led_state = LED_STATE_OFF;
  set_status_led(status_led_state);
}
void set_status_led(enum led_state_t state) {
  if (state == LED_STATE_ON) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  }
}
void add_blinks(uint8_t blinks) {
  led_blinks += blinks;
}

// DEBUG USB
void send_debug_message(char *name, uint8_t name_length, uint8_t id, int16_t value) {
  uint8_t *message = malloc(name_length + 2 + 3 + 3 + 5 + 2);
  sprintf((char *)message, "%s (%u): %d\r\n", name, id, value);
  CDC_Transmit_FS(message, name_length + 2 + 3 + 5 + 2);
  free(message);
}

/**
  * PWM FUNCTIONS
  * @brief Set PWM duty cycle with 0.01% resolution
  * @param duty_cycle_hundredths: Duty cycle in hundredths of a percent (e.g., 1000 = 10.00%)
  * @retval None
  */
void set_PWM0_duty(uint16_t duty_cycle_hundredths)
{
  uint32_t pulse = (19200UL * duty_cycle_hundredths) / 10000UL;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 19200UL - pulse);
}
void set_PWM1_duty(uint16_t duty_cycle_hundredths)
{
  uint32_t pulse = (19200UL * duty_cycle_hundredths) / 10000UL;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 19200UL - pulse);
}
void set_PWM2_duty(uint16_t duty_cycle_hundredths)
{
  uint32_t pulse = (19200UL * duty_cycle_hundredths) / 10000UL;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 19200UL - pulse);
}
void set_PWM3_duty(uint16_t pulse)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t) pulse);
}
void set_PWM4_duty(uint16_t pulse)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t) pulse);
}
//uint16_t get_PWM0_duty()
//{
//  uint16_t pulse = (10000UL * __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2)) / 19200UL;
//  return pulse;
//}
//uint16_t get_PWM1_duty()
//{
//  uint16_t pulse = (10000UL * __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3)) / 19200UL;
//  return pulse;
//}
//uint16_t get_PWM2_duty()
//{
//  uint16_t pulse = (10000UL * __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4)) / 19200UL;
//  return pulse;
//}
//uint16_t get_PWM3_duty()
//{
//  uint16_t pulse = (10000UL * __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1)) / 19200UL;
//  return pulse;
//}
//uint16_t get_PWM4_duty()
//{
//  uint16_t pulse = (10000UL * __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2)) / 19200UL;
//  return pulse;
//}

void update_PWM() {



  // LIMITS
  for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++) {
    if (output_actuator_channels[i] < PWM_MIN) { output_actuator_channels[i] = PWM_MIN; }
    if (output_actuator_channels[i] > PWM_MAX) { output_actuator_channels[i] = PWM_MAX; }
  }

  // SWASHPLATE SERVOS
  set_PWM0_duty(output_actuator_channels[ACTUATOR_SWASH_LEFT]);
  set_PWM1_duty(output_actuator_channels[ACTUATOR_SWASH_RIGHT]);
  set_PWM2_duty(output_actuator_channels[ACTUATOR_SWASH_AFT]);

}

void update_Oneshot125() {

  // LIMITS
  for (uint8_t i = PWM_CHANNEL_COUNT - 1; i < OS1_CHANNEL_COUNT; i++) {
    if (output_actuator_channels[i] < ONESHOT125_MIN) { output_actuator_channels[i] = ONESHOT125_MIN; }
    if (output_actuator_channels[i] > ONESHOT125_MAX) { output_actuator_channels[i] = ONESHOT125_MAX; }
  }

  if (armed) {
    set_PWM3_duty(output_actuator_channels[ACTUATOR_MAIN_ROTOR]);
    set_PWM4_duty(output_actuator_channels[ACTUATOR_TAIL_ROTOR]);
  } else {
    set_PWM3_duty(ONESHOT125_DISARMED);
    set_PWM4_duty(ONESHOT125_DISARMED);
  }
}

uint8_t ready_to_arm() {

  if (received_crsf_channels[CRSF_CHANNEL_THROTTLE] > CRSF_MIN + 50) {
    arm_blocked = true;
  }

  if (radio_link_stats_last_update > HAL_GetTick() || radio_link_stats_last_update + MAX_RADIO_STATS_AGE < HAL_GetTick()) {
    arm_blocked = true;
  } else {
    if (radio_link_stats.link_quality_percentage_uplink < MIN_LINK_QLY) {
      arm_blocked = true;
    }
  }

  return !arm_blocked;
}

void update_channels() {
  // AUXILIARY ACTIONS
  if (received_crsf_channels[CRSF_CHANNEL_PREARM] < CRSF_MID) {
    pre_armed = false;
  }

  if (received_crsf_channels[CRSF_CHANNEL_ARM] <= CRSF_MID) {
    if (armed) {
      armed = false;
      previous_rotor_throttle = -NORM_RANGE;
      current_actuator_channels[ACTUATOR_MAIN_ROTOR] = -NORM_RANGE;
      current_actuator_channels[ACTUATOR_TAIL_ROTOR] = -NORM_RANGE;
      disable_status_led();
    } else {
      if (received_crsf_channels[CRSF_CHANNEL_PREARM] > CRSF_MID) {
        pre_armed = true;
      }
      if (arm_blocked) {
        arm_blocked = false;
        ready_to_arm();
      }
    }
  } else if (received_crsf_channels[CRSF_CHANNEL_ARM] > CRSF_MID) {
    if (!armed && pre_armed && ready_to_arm()) {
      previous_rotor_throttle = -NORM_RANGE;
      current_actuator_channels[ACTUATOR_MAIN_ROTOR] = -NORM_RANGE;
      current_actuator_channels[ACTUATOR_TAIL_ROTOR] = -NORM_RANGE;
      armed = true;
      enable_status_led();
    }
  }

  if        (   received_crsf_channels[CRSF_CHANNEL_FLIGHT_MODE] <= CRSF_ONE_THIRD) {
    if (flight_mode != FLIGHT_MODE_DIRECT) {
      flight_mode = FLIGHT_MODE_DIRECT;
      add_blinks(1);
    }
  } else if (   received_crsf_channels[CRSF_CHANNEL_FLIGHT_MODE] >  CRSF_ONE_THIRD
             && received_crsf_channels[CRSF_CHANNEL_FLIGHT_MODE] <= CRSF_TWO_THIRD) {
    if (flight_mode != FLIGHT_MODE_RATE) {
      // NOT YET IMPLEMENTED
      add_blinks(5);
      flight_mode = FLIGHT_MODE_RATE;
    }
  } else if (   received_crsf_channels[CRSF_CHANNEL_FLIGHT_MODE] >  CRSF_TWO_THIRD) {
    if (flight_mode != FLIGHT_MODE_ANGLE) {
      // NOT YET IMPLEMENTED
      add_blinks(5);
      flight_mode = FLIGHT_MODE_ANGLE;
    }
  }

  if        (received_crsf_channels[CRSF_CHANNEL_PIT_MODE] <= CRSF_MID) {
    if (pit_mode != PIT_MODE_OFF) {
      pit_mode = PIT_MODE_OFF;
      add_blinks(2);
    }
  } else if (received_crsf_channels[CRSF_CHANNEL_PIT_MODE] > CRSF_MID) {
    if (pit_mode != PIT_MODE_ON) {
      pit_mode = PIT_MODE_ON;
      add_blinks(2);

    }
  }

  // HELI MIXER

  // INPUTS
  int16_t mixer_input_channels[MIXER_INPUT_CHANNEL_COUNT] = {0};

  mixer_input_channels[INPUT_CHANNEL_LON_CYC] =
          (int16_t) (-1 * (normalize_crsf(received_crsf_channels[CRSF_CHANNEL_LON_CYC]) + ROTOR_LON_TRIM));
  mixer_input_channels[INPUT_CHANNEL_LAT_CYC] =
          (int16_t)       (normalize_crsf(received_crsf_channels[CRSF_CHANNEL_LAT_CYC]) + ROTOR_LAT_TRIM);
  mixer_input_channels[INPUT_CHANNEL_COLLECTIVE] = (int16_t) (normalize_crsf(received_crsf_channels[CRSF_CHANNEL_COLLECTIVE]));
  mixer_input_channels[INPUT_CHANNEL_PEDALS] = (int16_t) (normalize_crsf(received_crsf_channels[CRSF_CHANNEL_PEDALS]));
  mixer_input_channels[INPUT_CHANNEL_THROTTLE] = (int16_t) (normalize_crsf(received_crsf_channels[CRSF_CHANNEL_THROTTLE]));

  #define SIN_60(x)  ((x) - (x)/8 - (x)/128 - (x)/512)   //  1000*sin(60) = 866.02... ~= 865

  mixer_input_channels[INPUT_CHANNEL_LAT_CYC] = SIN_60(mixer_input_channels[INPUT_CHANNEL_LAT_CYC]);

  int16_t mixer_output_channels[ACTUATOR_CHANNEL_COUNT] = {0};
  int16_t lon_cyc_half = (int16_t) (mixer_input_channels[INPUT_CHANNEL_LON_CYC] / 2);

  // SWASH PLATE
  mixer_output_channels[ACTUATOR_SWASH_LEFT] = (int16_t)
          (  mixer_input_channels[INPUT_CHANNEL_COLLECTIVE]
           + lon_cyc_half
           + mixer_input_channels[INPUT_CHANNEL_LAT_CYC]);

  mixer_output_channels[ACTUATOR_SWASH_RIGHT] = (int16_t) (-1 *
          (  mixer_input_channels[INPUT_CHANNEL_COLLECTIVE]
           + lon_cyc_half
           - mixer_input_channels[INPUT_CHANNEL_LAT_CYC]));

  mixer_output_channels[ACTUATOR_SWASH_AFT] = (int16_t)
          (  mixer_input_channels[INPUT_CHANNEL_COLLECTIVE]
           - mixer_input_channels[INPUT_CHANNEL_LON_CYC]);

  // ROTOR THROTTLE
  if (mixer_input_channels[INPUT_CHANNEL_THROTTLE] > previous_rotor_throttle + ACTUATOR_MAIN_MAX_DELTA) {
    mixer_input_channels[INPUT_CHANNEL_THROTTLE] = (int16_t) (previous_rotor_throttle + ACTUATOR_MAIN_MAX_DELTA);
  }
  if (mixer_input_channels[INPUT_CHANNEL_THROTTLE] < previous_rotor_throttle - ACTUATOR_MAIN_MAX_DELTA) {
    mixer_input_channels[INPUT_CHANNEL_THROTTLE] = (int16_t) (previous_rotor_throttle - ACTUATOR_MAIN_MAX_DELTA);
  }
  previous_rotor_throttle = mixer_input_channels[INPUT_CHANNEL_THROTTLE];

  mixer_output_channels[ACTUATOR_MAIN_ROTOR] = mixer_input_channels[INPUT_CHANNEL_THROTTLE];

  mixer_output_channels[ACTUATOR_TAIL_ROTOR] = (int16_t)
          (  mixer_input_channels[INPUT_CHANNEL_THROTTLE] / ROTOR_MAIN_TO_PEDAL_INV_GAIN
           - mixer_input_channels[INPUT_CHANNEL_PEDALS]
           + ROTOR_PEDAL_TRIM);

  // LIMITS
  for (uint8_t i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
    if (mixer_output_channels[i] > 1000) { mixer_output_channels[i] = 1000; }
    if (mixer_output_channels[i] < -1000) { mixer_output_channels[i] = -1000; }
  }

  // SWASHPLATE SERVOS
  // output = ((G-1 * old) + 1 * new) / G  <-- Low pass filter
  current_actuator_channels[ACTUATOR_SWASH_LEFT] = (int16_t)
          ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_LEFT]
            +                            1  *     mixer_output_channels[ACTUATOR_SWASH_LEFT])
                                                                                  / ACTUATOR_SWASH_LP_PARAM);
  current_actuator_channels[ACTUATOR_SWASH_RIGHT] = (int16_t)
          ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_RIGHT]
            +                            1  *     mixer_output_channels[ACTUATOR_SWASH_RIGHT])
                                                                                  / ACTUATOR_SWASH_LP_PARAM);
  current_actuator_channels[ACTUATOR_SWASH_AFT] = (int16_t)
          ((  (ACTUATOR_SWASH_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_SWASH_AFT]
            +                            1  *     mixer_output_channels[ACTUATOR_SWASH_AFT])
                                                                                  / ACTUATOR_SWASH_LP_PARAM);

  // MAIN ROTOR THROTTLE
  current_actuator_channels[ACTUATOR_MAIN_ROTOR] = (int16_t)
          ((  (ACTUATOR_MAIN_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_MAIN_ROTOR]
            +                           1  *     mixer_output_channels[ACTUATOR_MAIN_ROTOR]) / ACTUATOR_MAIN_LP_PARAM);

  // TAIL ROTOR THROTTLE
  current_actuator_channels[ACTUATOR_TAIL_ROTOR] = (int16_t)
          ((  (ACTUATOR_TAIL_LP_PARAM - 1) * current_actuator_channels[ACTUATOR_TAIL_ROTOR]
            +                           1  *     mixer_output_channels[ACTUATOR_TAIL_ROTOR]) / ACTUATOR_TAIL_LP_PARAM);

  // OUTPUTS
  switch (flight_mode) {
    case FLIGHT_MODE_POSITION:
    case FLIGHT_MODE_ANGLE:
    case FLIGHT_MODE_RATE:
      // NOT YET IMPLEMENTED
    default:
    case FLIGHT_MODE_DIRECT:
      output_actuator_channels[ACTUATOR_SWASH_LEFT]  = denormalize_pwm(current_actuator_channels[ACTUATOR_SWASH_LEFT]);
      output_actuator_channels[ACTUATOR_SWASH_RIGHT] = denormalize_pwm(current_actuator_channels[ACTUATOR_SWASH_RIGHT]);
      output_actuator_channels[ACTUATOR_SWASH_AFT]   = denormalize_pwm(current_actuator_channels[ACTUATOR_SWASH_AFT]);
      output_actuator_channels[ACTUATOR_MAIN_ROTOR]  = denormalize_os1(current_actuator_channels[ACTUATOR_MAIN_ROTOR]);
      output_actuator_channels[ACTUATOR_TAIL_ROTOR]  = denormalize_os1(current_actuator_channels[ACTUATOR_TAIL_ROTOR]);
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    for (uint16_t i = 0; i < 8; i++) {

      // Store received byte in circular buffer
      uint16_t next_head = (uart3_rb.head + 1) % UART3_RX_BUFFER_SIZE;

      // Check if buffer is not full
      if (next_head != uart3_rb.tail) {
        uart3_rb.buffer[uart3_rb.head] = uart3_rx_char[i];
        uart3_rb.head = next_head;
      }
    }

    // Re-enable reception for next byte
    HAL_UART_Receive_IT(&huart3, uart3_rx_char, 8);
  }
}

void on_crsf_frame(uint8_t type,
                   uint8_t dest_addr, // 0 for non-extended frames
                   uint8_t orig_addr, // 0 for non-extended frames
                   const uint8_t *payload,
                   uint8_t payload_len)
{
  switch (type) {
    case CRSF_FRAME_TYPE_CHN: {
      crsf_frame_channels_t channel_frame;
      crsf_parse_payload_channels(payload, &channel_frame);
      for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        received_crsf_channels[i] = channel_frame.channels[i];
      }
      break;
    }
    case CRSF_FRAME_TYPE_LNK: {
      crsf_parse_payload_link_stats(payload, &radio_link_stats);
      radio_link_stats_last_update = HAL_GetTick();
      break;
    }

    default:
      break;

  }

  update_channels();
}
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
  while (1) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
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
