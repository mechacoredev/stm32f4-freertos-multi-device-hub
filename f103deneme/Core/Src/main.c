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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define COMMUNICATION_RX_TIMEOUT_MS 750
#define COMMUNICATION_RECOVERY_PERIOD_MS 1000
#define CAN1_RUNTIME_NOTIFICATIONS (CAN_IT_RX_FIFO0_MSG_PENDING | \
        CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | \
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | \
        CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
volatile CAN_RxHeaderTypeDef g_can1_rx_header;
volatile uint8_t g_can1_rx_data[8];
volatile uint32_t g_can1_rx_count;
volatile uint32_t g_can1_rx_fifo_overrun_count;
volatile uint32_t g_can1_tx_ok_count;
volatile uint32_t g_can1_tx_error_count;
volatile HAL_StatusTypeDef g_can1_last_tx_status;
volatile uint32_t g_can1_led_toggle_count;
volatile uint8_t g_f103_nrf_rx_data[NRF24L01_MAX_PAYLOAD_SIZE];
volatile uint8_t g_f103_nrf_rx_length;
volatile uint32_t g_f103_nrf_rx_count;
volatile uint32_t g_f103_nrf_tx_count;
volatile uint32_t g_f103_nrf_tx_error_count;
volatile bool g_f103_nrf_initialized;
volatile uint32_t g_f103_can_rx_last_tick;
volatile uint32_t g_f103_nrf_rx_last_tick;
volatile uint32_t g_f103_can_recovery_attempt_count;
volatile uint32_t g_f103_can_recovery_success_count;
volatile uint32_t g_f103_can_recovery_failure_count;
volatile uint32_t g_f103_can_error_irq_count;
volatile uint32_t g_f103_can_last_error_code;
volatile uint32_t g_f103_can_last_esr;
volatile uint32_t g_f103_can_last_msr;
volatile bool g_f103_can_recovery_requested;
volatile bool g_f103_can_rx_timeout;
volatile bool g_f103_nrf_rx_timeout;
volatile uint8_t g_can1_response_pending;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef CAN1_StartRuntime(void);
static bool CAN1_RecoverRuntime(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef CAN1_StartRuntime(void)
{
  CAN_FilterTypeDef can_filter = {0};
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) return HAL_ERROR;
  if(HAL_CAN_Start(&hcan) != HAL_OK) return HAL_ERROR;
  if(HAL_CAN_ActivateNotification(&hcan, CAN1_RUNTIME_NOTIFICATIONS) !=
          HAL_OK){
    (void)HAL_CAN_Stop(&hcan);
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  return HAL_OK;
}

static bool CAN1_RecoverRuntime(void)
{
  /* HAL state ERROR veya controller bus-off iken Stop/Start yeterli degildir.
   * DeInit handle'i RESET durumuna, RCC reset ise mailbox ve hata mantigini
   * donanimsal baslangic durumuna getirir. */
  (void)HAL_CAN_DeactivateNotification(&hcan, CAN1_RUNTIME_NOTIFICATIONS);
  (void)HAL_CAN_AbortTxRequest(&hcan,
          CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  (void)HAL_CAN_DeInit(&hcan);
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();
  HAL_Delay(2);

  if(HAL_CAN_Init(&hcan) != HAL_OK) return false;
  if(CAN1_StartRuntime() != HAL_OK) return false;

  g_can1_response_pending = 0;
  g_f103_can_recovery_requested = false;
  return true;
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
  MX_CAN_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  if (CAN1_StartRuntime() != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader.StdId = 0x103; // Mesajımızın ID'si (F407 bu ID'yi okuyacak)
  TxHeader.ExtId = 0x01;  // Kullanmıyoruz ama sıfırlayalım
  TxHeader.IDE = CAN_ID_STD; // Standart ID kullanıyoruz (11-bit)
  TxHeader.RTR = CAN_RTR_DATA; // Remote frame değil, Data frame (Veri) gönderiyoruz
  TxHeader.TransmitGlobalTime = DISABLE;

  nrf24l01_config_t nrf_config = {
    .spi = &hspi2,
    .csn_port = NRF24_CSN_GPIO_Port,
    .csn_pin = NRF24_CSN_Pin,
    .ce_port = NRF24_CE_GPIO_Port,
    .ce_pin = NRF24_CE_Pin
  };
  g_f103_nrf_initialized = nrf24l01_init(&nrf_config);
  uint32_t communication_start_tick = HAL_GetTick();
  uint32_t last_can_recovery_tick = 0;
  uint32_t last_nrf_recovery_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();
      uint8_t can_response = g_can1_response_pending;
      if(can_response != 0){
        static const uint8_t high_message[] = "HIGH";
        static const uint8_t low_message[] = "LOW";
        const uint8_t* response;
        uint8_t response_length;
        g_can1_response_pending = 0;
        if(can_response == 1){
          response = low_message;
          response_length = sizeof(low_message) - 1;
        }
        else{
          response = high_message;
          response_length = sizeof(high_message) - 1;
        }
        TxHeader.StdId = 0x103;
        TxHeader.DLC = response_length;
        g_can1_last_tx_status = HAL_CAN_AddTxMessage(&hcan, &TxHeader,
                (uint8_t*)response, &TxMailbox);
        if(g_can1_last_tx_status == HAL_OK) g_can1_tx_ok_count++;
        else g_can1_tx_error_count++;
      }

      uint8_t nrf_response = 0;
      if(g_f103_nrf_initialized){
        /* IRQ falling-edge kacirilsa bile pin low kaldigi surece FIFO okunur. */
        if(g_nrf24l01_debug.irq_pending ||
                HAL_GPIO_ReadPin(NRF24_IRQ_GPIO_Port,
                        NRF24_IRQ_Pin) == GPIO_PIN_RESET){
          for(;;){
            uint8_t received[NRF24L01_MAX_PAYLOAD_SIZE] = {0};
            uint8_t received_length = 0;
            nrf24l01_status_t receive_status = nrf24l01_receive(received,
                    &received_length);
            if(receive_status != NRF24L01_OK) break;
            memcpy((void*)g_f103_nrf_rx_data, received, received_length);
            g_f103_nrf_rx_length = received_length;
            g_f103_nrf_rx_count++;
            if(received_length == 4 && memcmp(received, "HIGH", 4) == 0){
              HAL_GPIO_WritePin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin,
                      GPIO_PIN_SET);
              g_f103_nrf_rx_last_tick = HAL_GetTick();
              g_f103_nrf_rx_timeout = false;
              nrf_response = 1;
            }
            else if(received_length == 3 && memcmp(received, "LOW", 3) == 0){
              HAL_GPIO_WritePin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin,
                      GPIO_PIN_RESET);
              g_f103_nrf_rx_last_tick = HAL_GetTick();
              g_f103_nrf_rx_timeout = false;
              nrf_response = 2;
            }
          }
        }

        if(nrf_response != 0){
          static const uint8_t high_message[] = "HIGH";
          static const uint8_t low_message[] = "LOW";
          const uint8_t* response;
          uint8_t response_length;
          if(nrf_response == 1){
            response = low_message;
            response_length = sizeof(low_message) - 1;
          }
          else{
            response = high_message;
            response_length = sizeof(high_message) - 1;
          }
          /* F407'nin TX_DS interruptini isleyip tekrar RX moduna gecmesi icin
             kisa bir donus suresi birakilir. */
          HAL_Delay(5);
          nrf24l01_status_t send_status = nrf24l01_send(response,
                  response_length, 150);
          if(send_status == NRF24L01_OK) g_f103_nrf_tx_count++;
          else g_f103_nrf_tx_error_count++;
        }
      }

      uint32_t can_reference_tick = g_f103_can_rx_last_tick;
      if(can_reference_tick == 0) can_reference_tick = communication_start_tick;
      g_f103_can_rx_timeout = (now - can_reference_tick) >=
              COMMUNICATION_RX_TIMEOUT_MS;

      uint32_t nrf_reference_tick = g_f103_nrf_rx_last_tick;
      if(nrf_reference_tick == 0) nrf_reference_tick = communication_start_tick;
      g_f103_nrf_rx_timeout = (now - nrf_reference_tick) >=
              COMMUNICATION_RX_TIMEOUT_MS;

      if((g_f103_can_rx_timeout || g_f103_can_recovery_requested) &&
         (now - last_can_recovery_tick) >= COMMUNICATION_RECOVERY_PERIOD_MS){
        last_can_recovery_tick = now;
        g_f103_can_recovery_attempt_count++;
        if(CAN1_RecoverRuntime()){
          g_f103_can_recovery_success_count++;
        }else{
          g_f103_can_recovery_failure_count++;
        }
      }

      if(g_f103_nrf_rx_timeout &&
         (now - last_nrf_recovery_tick) >= COMMUNICATION_RECOVERY_PERIOD_MS){
        last_nrf_recovery_tick = now;
        g_f103_nrf_initialized = nrf24l01_init(&nrf_config);
      }

      HAL_Delay(2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if(HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, NRF24_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, NRF24_CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = NRF24_CSN_Pin|NRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CAN_LINK_LED_Pin|NRF_LINK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  if (can_handle->Instance != CAN1)
  {
    return;
  }

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  g_can1_rx_header = rx_header;

  for (uint32_t index = 0; index < 8U; index++)
  {
    g_can1_rx_data[index] = rx_data[index];
  }

  g_can1_rx_count++;

  /* HIGH/LOW komutu idempotenttir: bir paket kaybolsa bile sonraki paket
     LED'i dogru faza getirir. ISR icinde yalnizca GPIO ve tani verisi
     guncellenir; recovery ana dongude yapilir. */
  if (rx_header.IDE == CAN_ID_STD && rx_header.StdId == 0x407)
  {
    if(rx_header.DLC == 4 && memcmp(rx_data, "HIGH", 4) == 0){
      HAL_GPIO_WritePin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin,
              GPIO_PIN_SET);
      g_f103_can_rx_last_tick = HAL_GetTick();
      g_f103_can_rx_timeout = false;
      g_can1_led_toggle_count++;
      g_can1_response_pending = 1;
    }
    else if(rx_header.DLC == 3 && memcmp(rx_data, "LOW", 3) == 0){
      HAL_GPIO_WritePin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin,
              GPIO_PIN_RESET);
      g_f103_can_rx_last_tick = HAL_GetTick();
      g_f103_can_rx_timeout = false;
      g_can1_led_toggle_count++;
      g_can1_response_pending = 2;
    }
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *can_handle)
{
  if (can_handle->Instance == CAN1)
  {
    g_can1_rx_fifo_overrun_count++;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can_handle)
{
  if(can_handle == NULL || can_handle->Instance != CAN1) return;

  g_f103_can_error_irq_count++;
  g_f103_can_last_error_code = HAL_CAN_GetError(can_handle);
  g_f103_can_last_esr = can_handle->Instance->ESR;
  g_f103_can_last_msr = can_handle->Instance->MSR;
  g_f103_can_recovery_requested = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == NRF24_IRQ_Pin)
  {
    nrf24l01_irq_callback();
  }
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
