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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "rc522.h"
#include "nrf24l01.h"
#include "mpu6500.h"
#include "ili9341.h"
#include "system_health.h"
#include "string.h"
#include "can_manager.h"
#include "adxl345.h"
#include "ssd1306_spi.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
  bool active;
  uint32_t start_tick;
  i2c_job_t job;
}i2c_dispatch_slot_t;

typedef struct{
  bool active;
  uint32_t start_tick;
  spi_job_t job;
}spi_dispatch_slot_t;

typedef struct{
  float temperature_c;
  float humidity_percent;
  float pressure_pa;
  bool valid;
  uint32_t update_count;
}bme280_display_data_t;

typedef struct{
  float accel_g[3];
  float gyro_dps[3];
  bool valid;
  uint32_t update_count;
}mpu6500_display_data_t;

typedef struct{
  char tx_text[NRF24L01_PAYLOAD_SIZE + 1];
  char rx_text[NRF24L01_PAYLOAD_SIZE + 1];
  uint8_t tx_length;
  uint8_t rx_length;
  bool tx_valid;
  bool rx_valid;
  uint32_t tx_update_count;
  uint32_t rx_update_count;
}nrf24l01_display_data_t;

typedef struct{
  char tx_text[17];
  char rx_text[17];
  uint32_t tx_update_count;
  uint32_t rx_update_count;
}can_display_data_t;

typedef struct{
  volatile uint32_t can_rx_last_tick;
  volatile uint32_t nrf_rx_last_tick;
  volatile uint32_t nrf_reinit_attempt_count;
  volatile uint32_t nrf_reinit_success_count;
  volatile uint32_t nrf_reinit_failure_count;
  volatile bool can_rx_timeout;
  volatile bool nrf_rx_timeout;
}communication_link_debug_t;

typedef struct{
  bool card_found;
  char block_text[17];
  uint32_t update_count;
}rc522_display_data_t;

typedef struct{
  float x_g;
  float y_g;
  float z_g;
  bool valid;
  uint32_t update_count;
}adxl345_display_data_t;

typedef struct{
  uint32_t scl_level;
  uint32_t sda_level;
  uint32_t sr1;
  uint32_t sr2;
  uint32_t cr1;
  uint32_t cr2;
  uint32_t queue_depth;
  uint32_t dma_active_count;
  uint32_t busy_requeue_count;
  uint32_t runtime_recovery_count;
  uint32_t runtime_recovery_success_count;
  uint32_t transaction_count;
  uint32_t completed_count;
  uint32_t timeout_count;
  uint32_t hardware_error_count;
  uint32_t bus_error_count;
  uint32_t arbitration_lost_count;
  uint32_t overrun_count;
  uint32_t dma_error_count;
  uint32_t controller_berr_ignored_count;
  uint32_t stop_wait_timeout_count;
  uint32_t software_reset_count;
  uint32_t physical_recovery_count;
  uint32_t physical_recovery_success_count;
  uint32_t recovery_failed_count;
  uint8_t device_address;
  uint8_t register_address;
  i2c_manager_phase_t phase;
  i2c_manager_phase_t last_error_phase;
  i2c_manager_return_status last_result;
  bool bus_free_after_cleanup;
}runtime_i2c_debug_t;

typedef struct{
  uint32_t snapshot_tick;
  runtime_i2c_debug_t i2c1;
  uint32_t bme280_update_count;
  uint32_t bme280_data_age_ms;
  uint32_t mpu6500_update_count;
  uint32_t mpu6500_data_age_ms;
  uint32_t mpu6500_dma_success_count;
  uint32_t mpu6500_dma_error_count;
  uint32_t mpu6500_queue_error_count;
  uint32_t mpu6500_irq_count;
  uint32_t mpu6500_irq_timeout_count;
  uint32_t mpu6500_poll_fallback_count;
  uint32_t mpu6500_int_status_clear_success_count;
  uint32_t mpu6500_int_status_clear_error_count;
  uint8_t mpu6500_last_int_status;
  uint32_t mpu6500_last_irq_tick;
  uint32_t rc522_liveness_age_ms;
  uint32_t rc522_liveness_error_count;
  uint8_t rc522_expected_version;
  uint8_t rc522_last_observed_version;
  uint32_t nrf_tx_success_age_ms;
  uint32_t nrf_tx_success_count;
  uint32_t nrf_tx_error_count;
  uint32_t nrf_rx_success_age_ms;
  uint32_t nrf_rx_packet_count;
  uint32_t ili9341_success_age_ms;
  uint32_t ili9341_dma_success_count;
  uint32_t ili9341_dma_error_count;
  uint32_t stale_sensor_mask;
  uint32_t stale_subsystem_mask;
  bool sensor_data_fresh;
  bool all_monitored_subsystems_healthy;
  uint32_t heartbeat_mask;
  uint32_t missing_heartbeat_mask;
  uint32_t monitor_cycle_count;
  uint32_t reset_cause;
  uint32_t watchdog_feed_count;
  uint32_t watchdog_start_failure_count;
  bool last_reset_was_watchdog;
  bool watchdog_active;
  bool startup_grace_active;
  bool normal_monitoring_started;
  bool watchdog_feed_allowed;
  bool bme280_initialized;
  bool mpu6500_initialized;
  bool rc522_initialized;
  bool nrf_tx_initialized;
  bool nrf_rx_initialized;
  bool ili9341_initialized;
}runtime_debug_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DMA_DISPATCH_TIMEOUT_MS 1000

#define I2C_DMA_NEW_JOB_FLAG  0x01
#define I2C1_DMA_DONE_FLAG    0x02
#define I2C2_DMA_DONE_FLAG    0x04
#define I2C3_DMA_DONE_FLAG    0x08
#define I2C_DMA_ALL_FLAGS     0x0F
#define I2C_WORKER_READY_FLAG  0x10
#define I2C_RUNTIME_READY_FLAG 0x20

#define SPI_DMA_NEW_JOB_FLAG  0x01
#define SPI1_DMA_DONE_FLAG    0x02
#define SPI2_DMA_DONE_FLAG    0x04
#define SPI3_DMA_DONE_FLAG    0x08
#define SPI_DMA_ALL_FLAGS     0x0F

#define ILI9341_DMA_SUCCESS_FLAG 0x01
#define ILI9341_DMA_ERROR_FLAG   0x02
#define ILI9341_DMA_ALL_FLAGS    0x03

#define BME280_DMA_SUCCESS_FLAG  0x01
#define BME280_DMA_ERROR_FLAG    0x02
#define BME280_DMA_ALL_FLAGS     0x03
#define MPU6500_DATA_READY_FLAG  0x01
#define MPU6500_DMA_SUCCESS_FLAG 0x02
#define MPU6500_DMA_ERROR_FLAG   0x04
#define MPU6500_ALL_FLAGS        0x07
#define ADXL345_DMA_SUCCESS_FLAG 0x01
#define ADXL345_DMA_ERROR_FLAG   0x02
#define SSD1306_SPI_SUCCESS_FLAG 0x01
#define SSD1306_SPI_ERROR_FLAG   0x02

#define MPU6500_USE_DATA_READY_INTERRUPT 1
#define I2C1_SENSOR_CLIENT_COUNT 2
#define I2C2_SENSOR_CLIENT_COUNT 1
#define I2C_SENSOR_ERROR_BACKOFF_MS 50
#define SENSOR_DATA_STALE_TIMEOUT_MS 2000
#define RC522_LIVENESS_CHECK_PERIOD_MS 1000
#define RC522_LIVENESS_STALE_TIMEOUT_MS 3000
#define NRF_PROGRESS_STALE_TIMEOUT_MS 4000
#define NRF_RADIO_IRQ_FLAG 0x01
#define NRF_RADIO_DMA_SUCCESS_FLAG 0x02
#define NRF_RADIO_DMA_ERROR_FLAG 0x04
#define NRF_RADIO_TX_REQUEST_FLAG 0x08
#define DISPLAY_PROGRESS_STALE_TIMEOUT_MS 2000
#define WATCHDOG_STARTUP_GRACE_MS 5000
#define BUS_RECOVERY_RETRY_PERIOD_MS 1000
#define BUS_RECOVERY_RESET_TIMEOUT_MS 30000
#define CAN_ACK_TIMEOUT_MS 3000
#define COMMUNICATION_PERIOD_MS 500
#define COMMUNICATION_RX_TIMEOUT_MS 750
#define NRF_REINIT_RETRY_MS 1000
#define SENSOR_STALE_BME280_MASK 0x01
#define SENSOR_STALE_MPU6500_MASK 0x02
#define SPI_STALE_RC522_MASK 0x04
#define SPI_STALE_NRF_TX_MASK 0x08
#define SPI_STALE_NRF_RX_MASK 0x10
#define SPI_STALE_DISPLAY_MASK 0x20
#define SENSOR_STALE_ADXL345_MASK 0x40
#define SPI_STALE_SSD1306_MASK 0x100

/* Tam sistem dogrulamasi: I2C sensorleri, SPI1 RC522/tek NRF, SPI3 iki
 * ILI9341/SSD1306 ve CAN2 ayni firmware icinde etkindir. */
#define I2C_ONLY_TEST 0
#define CAN2_RUNTIME_ENABLED 1

/* Tam sistem videosu: NRF ve CAN task heartbeat'leri, veri ilerlemesi,
 * baslatma sonucu ve bus hata/recovery durumlari watchdog kapsamindadir. */
#define WATCHDOG_MONITOR_NRF 1
#define WATCHDOG_MONITOR_CAN 1

#if WATCHDOG_MONITOR_NRF
#define NRF_HEALTH_TASK_MASK \
  (SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_NRF_TX) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_NRF_RX))
#else
#define NRF_HEALTH_TASK_MASK 0U
#endif

#if WATCHDOG_MONITOR_CAN
#define CAN_HEALTH_TASK_MASK \
  (SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_CAN_RX) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_CAN_TX))
#else
#define CAN_HEALTH_TASK_MASK 0U
#endif

#if I2C_ONLY_TEST
#define ACTIVE_HEALTH_MASK \
  (SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_BME280) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_MPU6500) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_I2C_DISPATCH) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_ADXL345))
#else
#define ACTIVE_HEALTH_MASK \
  (SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_BME280) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_MPU6500) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_I2C_DISPATCH) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_RC522) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_SPI_DISPATCH) | \
   NRF_HEALTH_TASK_MASK | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_DISPLAY) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_ADXL345) | \
   SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_SSD1306_SPI) | \
   CAN_HEALTH_TASK_MASK)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

/* Definitions for bme280task */
osThreadId_t bme280taskHandle;
const osThreadAttr_t bme280task_attributes = {
  .name = "bme280task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2cdmatask */
osThreadId_t i2cdmataskHandle;
const osThreadAttr_t i2cdmatask_attributes = {
  .name = "i2cdmatask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rc522task */
osThreadId_t rc522taskHandle;
const osThreadAttr_t rc522task_attributes = {
  .name = "rc522task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6500task */
osThreadId_t mpu6500taskHandle;
const osThreadAttr_t mpu6500task_attributes = {
  .name = "mpu6500task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spidmatask */
osThreadId_t spidmataskHandle;
const osThreadAttr_t spidmatask_attributes = {
  .name = "spidmatask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for nrf24l01TXtask */
osThreadId_t nrf24l01TXtaskHandle;
const osThreadAttr_t nrf24l01TXtask_attributes = {
  .name = "nrf24l01TXtask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nrf24l01RXtask */
osThreadId_t nrf24l01RXtaskHandle;
const osThreadAttr_t nrf24l01RXtask_attributes = {
  .name = "nrf24l01RXtask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ili9341task */
osThreadId_t ili9341taskHandle;
const osThreadAttr_t ili9341task_attributes = {
  .name = "ili9341task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for healthtask */
osThreadId_t healthtaskHandle;
const osThreadAttr_t healthtask_attributes = {
  .name = "healthtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2ReadTask */
osThreadId_t CAN2ReadTaskHandle;
const osThreadAttr_t CAN2ReadTask_attributes = {
  .name = "CAN2ReadTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CAN2WriteTask */
osThreadId_t CAN2WriteTaskHandle;
const osThreadAttr_t CAN2WriteTask_attributes = {
  .name = "CAN2WriteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADXL345 task */
osThreadId_t adxl345taskHandle;
const osThreadAttr_t adxl345task_attributes = {
  .name = "adxl345task", .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t ssd1306spitaskHandle;
const osThreadAttr_t ssd1306spitask_attributes = {
  .name = "ssd1306spi", .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c_job_queue */
osMessageQueueId_t i2c_job_queueHandle;
const osMessageQueueAttr_t i2c_job_queue_attributes = {
  .name = "i2c_job_queue"
};
/* Definitions for spi_job_queue */
osMessageQueueId_t spi_job_queueHandle;
const osMessageQueueAttr_t spi_job_queue_attributes = {
  .name = "spi_job_queue"
};
/* Definitions for i2c_mutex */
osMutexId_t i2c_mutexHandle;
const osMutexAttr_t i2c_mutex_attributes = {
  .name = "i2c_mutex"
};
/* Definitions for i2c_init_mutex */
osMutexId_t i2c_init_mutexHandle;
const osMutexAttr_t i2c_init_mutex_attributes = {
  .name = "i2c_init_mutex"
};
osMutexId_t i2c2_mutexHandle;
const osMutexAttr_t i2c2_mutex_attributes = { .name = "i2c2_mutex" };
osMutexId_t nrf_mode_mutexHandle;
const osMutexAttr_t nrf_mode_mutex_attributes = { .name = "nrf_mode_mutex" };
/* Definitions for spi_mutex */
osMutexId_t spi_mutexHandle;
const osMutexAttr_t spi_mutex_attributes = {
  .name = "spi_mutex"
};
/* Definitions for spi3_mutex */
osMutexId_t spi3_mutexHandle;
const osMutexAttr_t spi3_mutex_attributes = {
  .name = "spi3_mutex"
};
/* Definitions for spi2_mutex */
osMutexId_t spi2_mutexHandle;
const osMutexAttr_t spi2_mutex_attributes = {
  .name = "spi2_mutex"
};
/* Definitions for sensor_data_mutex */
osMutexId_t sensor_data_mutexHandle;
const osMutexAttr_t sensor_data_mutex_attributes = {
  .name = "sensor_data_mutex"
};
/* Definitions for dma_semaphore */
osSemaphoreId_t dma_semaphoreHandle;
const osSemaphoreAttr_t dma_semaphore_attributes = {
  .name = "dma_semaphore"
};
osSemaphoreId_t i2c2_dma_semaphoreHandle;
const osSemaphoreAttr_t i2c2_dma_semaphore_attributes = {
  .name = "i2c2_dma_semaphore"
};
/* Definitions for spi_semaphore */
osSemaphoreId_t spi_semaphoreHandle;
const osSemaphoreAttr_t spi_semaphore_attributes = {
  .name = "spi_semaphore"
};
/* Definitions for spi3_semaphore */
osSemaphoreId_t spi3_semaphoreHandle;
const osSemaphoreAttr_t spi3_semaphore_attributes = {
  .name = "spi3_semaphore"
};
/* Definitions for spi2_semaphore */
osSemaphoreId_t spi2_semaphoreHandle;
const osSemaphoreAttr_t spi2_semaphore_attributes = {
  .name = "spi2_semaphore"
};
/* USER CODE BEGIN PV */
bme280_handle_t my_bme280 = NULL;
rc522_handle_t my_rc522 = NULL;
mpu6500_handle_t my_mpu6500 = NULL;
nrf24l01_handle_t my_nrf2401 = NULL;
uint8_t sayac=0;
nrf24l01_handle_t my_nrf_tx = NULL;
nrf24l01_handle_t my_nrf_rx = NULL;
uint8_t nrf_pending_tx[NRF24L01_PAYLOAD_SIZE] = {0};
uint8_t nrf_pending_tx_length = 0;
volatile bool nrf_tx_request_pending = false;
adxl345_handle_t my_adxl345 = NULL;
ssd1306_spi_handle_t my_ssd1306_spi = NULL;
adxl345_display_data_t adxl345_display_data = {0};
volatile bool adxl345_init_completed = false;
volatile bool ssd1306_spi_init_completed = false;
volatile uint32_t adxl345_dma_success_count = 0;
volatile uint32_t adxl345_dma_error_count = 0;
volatile uint32_t ssd1306_spi_refresh_count = 0;
volatile uint32_t adxl345_last_success_tick = 0;
volatile uint32_t ssd1306_spi_last_success_tick = 0;
volatile uint32_t nrf_tx_success_count = 0;
volatile uint32_t nrf_tx_error_count = 0;
volatile uint32_t nrf_rx_packet_count = 0;
volatile uint8_t nrf_rx_fifo_count = 0;
volatile uint8_t nrf_rx_debug_write_index = 0;
volatile bool nrf_rx_fifo_was_full = false;
uint8_t nrf_rx_fifo[NRF24L01_FIFO_DEPTH][NRF24L01_PAYLOAD_SIZE] = {0};
uint8_t nrf_rx_payload_lengths[NRF24L01_FIFO_DEPTH] = {0};
volatile uint8_t i2c_dma_active_count = 0;
volatile uint8_t i2c_dma_max_concurrent = 0;
volatile bool i2c1_startup_recovery_completed = false;
volatile bool i2c1_startup_recovery_success = false;
volatile bool i2c1_startup_was_busy = false;
volatile bool i2c1_startup_sda_was_low = false;
volatile uint32_t i2c1_startup_recovery_count = 0;
volatile uint32_t i2c1_startup_clock_pulse_count = 0;
volatile uint32_t i2c1_startup_sr2_before = 0;
volatile uint32_t i2c1_startup_sr2_after = 0;
volatile uint32_t i2c1_runtime_recovery_count = 0;
volatile uint32_t i2c1_runtime_recovery_success_count = 0;
volatile uint32_t i2c1_runtime_recovery_clock_pulse_count = 0;
volatile uint32_t i2c1_runtime_recovery_sr2_before = 0;
volatile uint32_t i2c1_runtime_recovery_sr2_after = 0;
volatile uint32_t i2c1_runtime_busy_requeue_count = 0;
volatile uint32_t i2c_init_mutex_acquire_count = 0;
volatile uint32_t i2c_init_mutex_release_count = 0;
const char* volatile i2c_init_mutex_owner_name = NULL;
volatile bool i2c_runtime_ready = false;
volatile uint32_t i2c_sensor_client_ready_count = 0;
volatile uint32_t i2c1_sensor_client_ready_count = 0;
volatile uint32_t i2c2_sensor_client_ready_count = 0;
volatile bool i2c1_runtime_ready = false;
volatile bool i2c2_runtime_ready = false;
volatile uint32_t i2c_runtime_start_count = 0;
volatile uint32_t i2c_polling_job_count = 0;
volatile uint32_t i2c_dma_job_count = 0;
volatile i2c_manager_return_status i2c_last_polling_job_status =
        _i2c_manager_ok;
volatile uint8_t spi_dma_active_count = 0;
volatile uint8_t spi_dma_max_concurrent = 0;
ili9341_handle_t my_tft1 = NULL;
ili9341_handle_t my_tft2 = NULL;
volatile bool ili9341_init_completed = false;
volatile bool ili9341_test_passed = false;
volatile uint32_t ili9341_dma_success_count = 0;
volatile uint32_t ili9341_dma_error_count = 0;
volatile ili9341_return_status ili9341_last_status = _ili9341_ok;
bme280_display_data_t bme280_display_data = {0};
mpu6500_display_data_t mpu6500_display_data = {0};
nrf24l01_display_data_t nrf24l01_display_data = {0};
rc522_display_data_t rc522_display_data = {0};
can_display_data_t can_display_data = {0};
volatile communication_link_debug_t g_communication_link_debug = {0};
volatile bool bme280_init_completed = false;
volatile bool mpu6500_init_completed = false;
volatile uint32_t mpu6500_dma_success_count = 0;
volatile uint32_t mpu6500_dma_error_count = 0;
volatile uint32_t mpu6500_queue_error_count = 0;
volatile uint32_t mpu6500_irq_level_recovery_count = 0;
volatile uint32_t mpu6500_irq_timeout_count = 0;
volatile uint32_t mpu6500_poll_fallback_count = 0;
volatile uint32_t mpu6500_int_status_clear_success_count = 0;
volatile uint32_t mpu6500_int_status_clear_error_count = 0;
volatile uint8_t mpu6500_last_int_status = 0;
volatile uint32_t mpu6500_last_irq_tick = 0;
volatile bool mpu6500_startup_int_clear_success = false;
volatile bool mpu6500_startup_int_was_data_ready = false;
volatile uint32_t bme280_last_success_tick = 0;
volatile uint32_t mpu6500_last_success_tick = 0;
volatile uint32_t rc522_last_liveness_tick = 0;
volatile uint32_t rc522_liveness_error_count = 0;
volatile uint8_t rc522_last_observed_version = 0;
volatile uint32_t nrf_tx_last_success_tick = 0;
volatile uint32_t nrf_rx_last_success_tick = 0;
volatile uint32_t ili9341_last_success_tick = 0;
volatile uint32_t sensor_data_stale_mask = 0;
volatile uint32_t subsystem_stale_mask = 0;
volatile bool health_startup_grace_active = true;
volatile bool health_normal_monitoring_started = false;
volatile bool health_watchdog_feed_allowed = false;
volatile runtime_debug_t g_runtime_debug = {0};
volatile bool rc522_init_completed = false;
volatile uint8_t rc522_version = 0;
volatile uint8_t rc522_last_request_status = MI_NOTAGERR;
volatile uint8_t rc522_last_anticoll_status = MI_NOTAGERR;
volatile uint8_t rc522_last_uid[5] = {0};
volatile uint32_t rc522_detection_count = 0;
volatile bool spi_dispatch_ready = false;
volatile bool nrf_tx_init_completed = false;
volatile bool nrf_rx_init_completed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
#if CAN2_RUNTIME_ENABLED
static void MX_CAN2_Init(void);
#endif
void startbme280task(void *argument);
void starti2cdmatask(void *argument);
void startrc522task(void *argument);
void startmpu6500task(void *argument);
void startspidmatask(void *argument);
void startnrf24l01TXtask(void *argument);
void startnrf24l01RXtask(void *argument);
void startili9341task(void *argument);
void starthealthtask(void *argument);
void StartCAN2ReadTask(void *argument);
void StartCAN2WriteTask(void *argument);
void startadxl345task(void *argument);
void startssd1306spitask(void *argument);

/* USER CODE BEGIN PFP */

static int8_t get_i2c_dispatch_index(I2C_TypeDef* i2c_handle);
static bool recover_i2c1_bus_once(void);
static bool recover_i2c2_bus_runtime(I2C_TypeDef* i2c);
static int8_t get_spi_dispatch_index(SPI_TypeDef* spi_handle);
static osStatus_t submit_i2c_dma_job(i2c_job_t* job);
static osStatus_t submit_spi_dma_job(spi_job_t* job);
static ili9341_return_status run_ili9341_dma_operation(
        ili9341_handle_t display);
static ili9341_return_status draw_ili9341_text(ili9341_handle_t display,
        uint16_t x, uint16_t y, const char* text);
static ili9341_return_status draw_ili9341_colored_text(
        ili9341_handle_t display, uint16_t x, uint16_t y, const char* text,
        uint16_t color);
static ili9341_return_status draw_sensor_display_layout(void);
static ili9341_return_status refresh_bme280_display(
        const bme280_display_data_t* data);
static ili9341_return_status refresh_mpu6500_display(
        const mpu6500_display_data_t* data);
static ili9341_return_status refresh_adxl345_display(
        const adxl345_display_data_t* data);
static ili9341_return_status refresh_rc522_display(
        const rc522_display_data_t* data);
static void format_fixed_2(char* output, uint8_t field_width,
        float value, const char* unit);
static void format_text_field(char* output, uint8_t field_width,
        const char* text);
static void copy_display_text(char* output, uint8_t output_size,
        const uint8_t* input, uint8_t input_length);
static void refresh_i2c_dma_debug(i2c_dispatch_slot_t* active_jobs);
static void refresh_spi_dma_debug(spi_dispatch_slot_t* active_jobs);
static void refresh_runtime_debug(uint32_t now,
        uint32_t stale_sensor_mask);
static bool validate_rc522_spi(void* context);
static bool validate_nrf24_spi(void* context);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static bool validate_rc522_spi(void* context){
  rc522_handle_t device = (rc522_handle_t)context;
  if(device == NULL) return false;

  uint8_t version = rc522_get_version(device);
  return (version != 0) && (version != 0xFF);
}

static bool validate_nrf24_spi(void* context){
  return nrf24l01_validate((nrf24l01_handle_t)context);
}

static int8_t get_i2c_dispatch_index(I2C_TypeDef* i2c_handle){
  if(i2c_handle == I2C1) return 0;
  if(i2c_handle == I2C2) return 1;
  if(i2c_handle == I2C3) return 2;
  return -1;
}

static void startup_delay_us(uint32_t delay_us){
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t cycles_per_us = SystemCoreClock / 1000000;
  if(cycles_per_us == 0) cycles_per_us = 1;
  uint32_t start_cycle = DWT->CYCCNT;
  uint32_t delay_cycles = cycles_per_us * delay_us;
  while((DWT->CYCCNT - start_cycle) < delay_cycles) { }
}

static bool wait_i2c1_line_high(uint32_t pin, uint32_t timeout_us){
  uint32_t elapsed_us = 0;
  while(LL_GPIO_IsInputPinSet(GPIOB, pin) == 0){
    if(elapsed_us >= timeout_us) return false;
    startup_delay_us(10);
    elapsed_us += 10;
  }
  return true;
}

static bool recover_i2c1_bus_lines(volatile uint32_t* pulse_count,
        volatile bool* was_busy, volatile bool* sda_was_low){
  const uint32_t i2c1_pins = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  const uint32_t scl_pin = LL_GPIO_PIN_6;
  const uint32_t sda_pin = LL_GPIO_PIN_7;
  LL_GPIO_InitTypeDef gpio = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  // ODR bitlerini once HIGH yapmak, AF'den GPIO'ya gecerken LOW darbesini onler.
  LL_GPIO_SetOutputPin(GPIOB, i2c1_pins);
  gpio.Pin = i2c1_pins;
  gpio.Mode = LL_GPIO_MODE_OUTPUT;
  gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &gpio);
  startup_delay_us(20);

  bool scl_was_low = LL_GPIO_IsInputPinSet(GPIOB, scl_pin) == 0;
  bool current_sda_low = LL_GPIO_IsInputPinSet(GPIOB, sda_pin) == 0;
  if(sda_was_low != NULL) *sda_was_low = current_sda_low;
  if(was_busy != NULL) *was_busy = scl_was_low || current_sda_low;

  // Slave bir baytin ortasinda kaldiysa en fazla dokuz clock ile bayti bitirt.
  for(uint32_t pulse = 0;
      pulse < 9 && LL_GPIO_IsInputPinSet(GPIOB, sda_pin) == 0;
      pulse++){
    LL_GPIO_ResetOutputPin(GPIOB, scl_pin);
    startup_delay_us(10);
    LL_GPIO_SetOutputPin(GPIOB, scl_pin);
    wait_i2c1_line_high(scl_pin, 1000);
    startup_delay_us(10);
    if(pulse_count != NULL) (*pulse_count)++;
  }

  // Belirgin bir STOP olustur: SDA LOW, SCL HIGH, sonra SDA HIGH.
  LL_GPIO_ResetOutputPin(GPIOB, sda_pin);
  startup_delay_us(10);
  LL_GPIO_SetOutputPin(GPIOB, scl_pin);
  bool scl_released = wait_i2c1_line_high(scl_pin, 1000);
  startup_delay_us(10);
  LL_GPIO_SetOutputPin(GPIOB, sda_pin);
  startup_delay_us(10);

  bool sda_released = LL_GPIO_IsInputPinSet(GPIOB, sda_pin) != 0;
  return scl_released && sda_released;
}

static bool recover_i2c1_bus_once(void){
  i2c1_startup_recovery_count++;
  i2c1_startup_sr2_before = I2C1->SR2;
  return recover_i2c1_bus_lines(&i2c1_startup_clock_pulse_count,
          &i2c1_startup_was_busy, &i2c1_startup_sda_was_low);
}

static bool recover_i2c1_bus_runtime(I2C_TypeDef* i2c){
  if(i2c != I2C1) return false;

  i2c1_runtime_recovery_count++;
  i2c1_runtime_recovery_sr2_before = I2C1->SR2;
  bool lines_released = recover_i2c1_bus_lines(
          &i2c1_runtime_recovery_clock_pulse_count, NULL, NULL);

  /* GPIO pinlerini tekrar AF4 open-drain yapar ve I2C1 registerlarini
   * CubeMX'teki 400 kHz ayarlariyla yeniden kurar. */
  MX_I2C1_Init();
  i2c1_runtime_recovery_sr2_after = I2C1->SR2;

  bool recovered = lines_released &&
          (i2c1_runtime_recovery_sr2_after & I2C_SR2_BUSY) == 0;
  if(recovered) i2c1_runtime_recovery_success_count++;
  return recovered;
}

static bool recover_i2c2_bus_runtime(I2C_TypeDef* i2c){
  if(i2c != I2C2) return false;
  const uint32_t scl_pin = LL_GPIO_PIN_10;
  const uint32_t sda_pin = LL_GPIO_PIN_11;
  LL_GPIO_InitTypeDef gpio = {0};

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);
  LL_GPIO_SetOutputPin(GPIOB, scl_pin | sda_pin);
  gpio.Pin = scl_pin | sda_pin;
  gpio.Mode = LL_GPIO_MODE_OUTPUT;
  gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &gpio);
  startup_delay_us(20);

  for(uint32_t pulse = 0;
      pulse < 9 && LL_GPIO_IsInputPinSet(GPIOB, sda_pin) == 0;
      pulse++){
    LL_GPIO_ResetOutputPin(GPIOB, scl_pin);
    startup_delay_us(10);
    LL_GPIO_SetOutputPin(GPIOB, scl_pin);
    startup_delay_us(10);
  }

  LL_GPIO_ResetOutputPin(GPIOB, sda_pin);
  startup_delay_us(10);
  LL_GPIO_SetOutputPin(GPIOB, scl_pin);
  startup_delay_us(10);
  LL_GPIO_SetOutputPin(GPIOB, sda_pin);
  startup_delay_us(10);
  bool lines_released = LL_GPIO_IsInputPinSet(GPIOB, scl_pin) != 0 &&
          LL_GPIO_IsInputPinSet(GPIOB, sda_pin) != 0;
  MX_I2C2_Init();
  return lines_released && !LL_I2C_IsActiveFlag_BUSY(I2C2);
}

static int8_t get_spi_dispatch_index(SPI_TypeDef* spi_handle){
  if(spi_handle == SPI1) return 0;
  if(spi_handle == SPI2) return 1;
  if(spi_handle == SPI3) return 2;
  return -1;
}

static osStatus_t submit_i2c_dma_job(i2c_job_t* job){
  if(job == NULL) return osErrorParameter;

  /* Bir hattaki sensorun init basarisi, ayni hattaki diger sensorun DMA
   * istegini engellememelidir. Dispatcher her peripheral icin islemleri
   * zaten siralastirir; worker-ready bayragi da hat baslangic kurtarmasi
   * tamamlanmadan sensor task'larini serbest birakmaz. */

  osStatus_t queue_status;
  queue_status = osMessageQueuePut(i2c_job_queueHandle, job, 0, 0);
  if(queue_status == osOK && i2cdmataskHandle != NULL){
    osThreadFlagsSet(i2cdmataskHandle, I2C_DMA_NEW_JOB_FLAG);
  }
  return queue_status;
}

static void complete_i2c_job(i2c_job_t* job,
        i2c_manager_return_status result){
  if(job == NULL) return;
  if(job->completion_result != NULL){
    *job->completion_result = result;
  }

  uint32_t result_flag = result == _i2c_manager_ok ?
          job->notify_flag : job->error_flag;
  if(job->notify_task != NULL && result_flag != 0){
    osThreadFlagsSet(job->notify_task, result_flag);
  }
}

static bool requeue_i2c_job(i2c_job_t* job){
  if(job == NULL) return false;
  if(osMessageQueuePut(i2c_job_queueHandle, job, 0, 0) == osOK){
    return true;
  }
  complete_i2c_job(job, _i2c_manager_busy);
  return false;
}

static void refresh_runtime_debug(uint32_t now,
        uint32_t stale_mask){
  volatile i2c_manager_diagnostics_t* manager =
          &g_i2c_manager_diagnostics[0];

  g_runtime_debug.snapshot_tick = now;
  g_runtime_debug.i2c1.scl_level =
          LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6) != 0;
  g_runtime_debug.i2c1.sda_level =
          LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7) != 0;
  g_runtime_debug.i2c1.sr1 = I2C1->SR1;
  g_runtime_debug.i2c1.sr2 = I2C1->SR2;
  g_runtime_debug.i2c1.cr1 = I2C1->CR1;
  g_runtime_debug.i2c1.cr2 = I2C1->CR2;
  if(i2c_job_queueHandle == NULL){
    g_runtime_debug.i2c1.queue_depth = 0;
  }else{
    g_runtime_debug.i2c1.queue_depth =
            osMessageQueueGetCount(i2c_job_queueHandle);
  }
  g_runtime_debug.i2c1.dma_active_count = i2c_dma_active_count;
  g_runtime_debug.i2c1.busy_requeue_count =
          i2c1_runtime_busy_requeue_count;
  g_runtime_debug.i2c1.runtime_recovery_count =
          i2c1_runtime_recovery_count;
  g_runtime_debug.i2c1.runtime_recovery_success_count =
          i2c1_runtime_recovery_success_count;
  g_runtime_debug.i2c1.transaction_count = manager->transaction_count;
  g_runtime_debug.i2c1.completed_count = manager->completed_count;
  g_runtime_debug.i2c1.timeout_count = manager->timeout_count;
  g_runtime_debug.i2c1.hardware_error_count = manager->hardware_error_count;
  g_runtime_debug.i2c1.bus_error_count = manager->bus_error_count;
  g_runtime_debug.i2c1.arbitration_lost_count =
          manager->arbitration_lost_count;
  g_runtime_debug.i2c1.overrun_count = manager->overrun_count;
  g_runtime_debug.i2c1.dma_error_count = manager->dma_error_count;
  g_runtime_debug.i2c1.controller_berr_ignored_count =
          manager->controller_berr_ignored_count;
  g_runtime_debug.i2c1.stop_wait_timeout_count =
          manager->stop_wait_timeout_count;
  g_runtime_debug.i2c1.software_reset_count = manager->software_reset_count;
  g_runtime_debug.i2c1.physical_recovery_count =
          manager->physical_recovery_count;
  g_runtime_debug.i2c1.physical_recovery_success_count =
          manager->physical_recovery_success_count;
  g_runtime_debug.i2c1.recovery_failed_count =
          manager->recovery_failed_count;
  g_runtime_debug.i2c1.device_address = manager->device_address;
  g_runtime_debug.i2c1.register_address = manager->register_address;
  g_runtime_debug.i2c1.phase = manager->phase;
  g_runtime_debug.i2c1.last_error_phase = manager->last_error_phase;
  g_runtime_debug.i2c1.last_result = manager->last_result;
  g_runtime_debug.i2c1.bus_free_after_cleanup =
          manager->bus_free_after_cleanup;

  g_runtime_debug.bme280_update_count = bme280_display_data.update_count;
  if(bme280_last_success_tick == 0) g_runtime_debug.bme280_data_age_ms = UINT32_MAX;
  else g_runtime_debug.bme280_data_age_ms = now - bme280_last_success_tick;
  g_runtime_debug.mpu6500_update_count = mpu6500_display_data.update_count;
  if(mpu6500_last_success_tick == 0) g_runtime_debug.mpu6500_data_age_ms = UINT32_MAX;
  else g_runtime_debug.mpu6500_data_age_ms = now - mpu6500_last_success_tick;
  g_runtime_debug.mpu6500_dma_success_count = mpu6500_dma_success_count;
  g_runtime_debug.mpu6500_dma_error_count = mpu6500_dma_error_count;
  g_runtime_debug.mpu6500_queue_error_count = mpu6500_queue_error_count;
  g_runtime_debug.mpu6500_irq_count = mpu6500_irq_count;
  g_runtime_debug.mpu6500_irq_timeout_count = mpu6500_irq_timeout_count;
  g_runtime_debug.mpu6500_poll_fallback_count =
          mpu6500_poll_fallback_count;
  g_runtime_debug.mpu6500_int_status_clear_success_count =
          mpu6500_int_status_clear_success_count;
  g_runtime_debug.mpu6500_int_status_clear_error_count =
          mpu6500_int_status_clear_error_count;
  g_runtime_debug.mpu6500_last_int_status = mpu6500_last_int_status;
  g_runtime_debug.mpu6500_last_irq_tick = mpu6500_last_irq_tick;
  if(rc522_last_liveness_tick == 0) g_runtime_debug.rc522_liveness_age_ms = UINT32_MAX;
  else g_runtime_debug.rc522_liveness_age_ms = now - rc522_last_liveness_tick;
  g_runtime_debug.rc522_liveness_error_count =
          rc522_liveness_error_count;
  g_runtime_debug.rc522_expected_version = rc522_version;
  g_runtime_debug.rc522_last_observed_version =
          rc522_last_observed_version;
  if(nrf_tx_last_success_tick == 0) g_runtime_debug.nrf_tx_success_age_ms = UINT32_MAX;
  else g_runtime_debug.nrf_tx_success_age_ms = now - nrf_tx_last_success_tick;
  g_runtime_debug.nrf_tx_success_count = nrf_tx_success_count;
  g_runtime_debug.nrf_tx_error_count = nrf_tx_error_count;
  if(nrf_rx_last_success_tick == 0) g_runtime_debug.nrf_rx_success_age_ms = UINT32_MAX;
  else g_runtime_debug.nrf_rx_success_age_ms = now - nrf_rx_last_success_tick;
  g_runtime_debug.nrf_rx_packet_count = nrf_rx_packet_count;
  if(ili9341_last_success_tick == 0) g_runtime_debug.ili9341_success_age_ms = UINT32_MAX;
  else g_runtime_debug.ili9341_success_age_ms = now - ili9341_last_success_tick;
  g_runtime_debug.ili9341_dma_success_count = ili9341_dma_success_count;
  g_runtime_debug.ili9341_dma_error_count = ili9341_dma_error_count;
  g_runtime_debug.stale_sensor_mask = stale_mask &
          (SENSOR_STALE_BME280_MASK | SENSOR_STALE_MPU6500_MASK |
           SENSOR_STALE_ADXL345_MASK);
  g_runtime_debug.stale_subsystem_mask = stale_mask;
  g_runtime_debug.sensor_data_fresh =
          g_runtime_debug.stale_sensor_mask == 0;
  g_runtime_debug.all_monitored_subsystems_healthy = stale_mask == 0;
  g_runtime_debug.heartbeat_mask = g_system_health.heartbeat_mask;
  g_runtime_debug.missing_heartbeat_mask =
          g_system_health.missing_heartbeat_mask;
  g_runtime_debug.monitor_cycle_count = g_system_health.monitor_cycle_count;
  g_runtime_debug.reset_cause = g_system_health.reset_cause;
  g_runtime_debug.watchdog_feed_count = g_system_health.watchdog_feed_count;
  g_runtime_debug.watchdog_start_failure_count =
          g_system_health.watchdog_start_failure_count;
  g_runtime_debug.last_reset_was_watchdog =
          (g_system_health.reset_cause & RCC_CSR_IWDGRSTF) != 0;
  g_runtime_debug.watchdog_active = g_system_health.watchdog_active;
  g_runtime_debug.startup_grace_active = health_startup_grace_active;
  g_runtime_debug.normal_monitoring_started =
          health_normal_monitoring_started;
  g_runtime_debug.watchdog_feed_allowed =
          health_watchdog_feed_allowed;
  g_runtime_debug.bme280_initialized = bme280_init_completed;
  g_runtime_debug.mpu6500_initialized = mpu6500_init_completed;
  g_runtime_debug.rc522_initialized = rc522_init_completed;
  g_runtime_debug.nrf_tx_initialized = nrf_tx_init_completed;
  g_runtime_debug.nrf_rx_initialized = nrf_rx_init_completed;
  g_runtime_debug.ili9341_initialized = ili9341_init_completed;
}

static bool acquire_i2c_init_lock(void){
  if(i2c_init_mutexHandle == NULL) return false;
  if(osMutexAcquire(i2c_init_mutexHandle, osWaitForever) != osOK){
    return false;
  }

  i2c_init_mutex_acquire_count++;
  i2c_init_mutex_owner_name = osThreadGetName(osThreadGetId());
  return true;
}

static void release_i2c_init_lock(void){
  if(i2c_init_mutexHandle == NULL) return;
  if(osMutexRelease(i2c_init_mutexHandle) == osOK){
    i2c_init_mutex_release_count++;
    i2c_init_mutex_owner_name = NULL;
  }
}

static void register_i2c_sensor_client_ready(I2C_TypeDef* i2c){
  /* Her fiziksel I2C hatti yalnizca kendi istemcilerini bekler. I2C2'deki
     eksik bir cihaz I2C1 sensorlerinin runtime fazini bloke edemez. */
  if(i2c == I2C1){
    if(i2c1_sensor_client_ready_count < I2C1_SENSOR_CLIENT_COUNT){
      i2c1_sensor_client_ready_count++;
      i2c_sensor_client_ready_count++;
    }
    if(i2c1_sensor_client_ready_count >= I2C1_SENSOR_CLIENT_COUNT &&
            !i2c1_runtime_ready){
      i2c1_runtime_ready = true;
      i2c_runtime_start_count++;
      osThreadFlagsSet(bme280taskHandle, I2C_RUNTIME_READY_FLAG);
      osThreadFlagsSet(mpu6500taskHandle, I2C_RUNTIME_READY_FLAG);
    }
  }
  else if(i2c == I2C2){
    if(i2c2_sensor_client_ready_count < I2C2_SENSOR_CLIENT_COUNT){
      i2c2_sensor_client_ready_count++;
      i2c_sensor_client_ready_count++;
    }
    if(i2c2_sensor_client_ready_count >= I2C2_SENSOR_CLIENT_COUNT &&
            !i2c2_runtime_ready){
      i2c2_runtime_ready = true;
      i2c_runtime_start_count++;
      osThreadFlagsSet(adxl345taskHandle, I2C_RUNTIME_READY_FLAG);
    }
  }
  i2c_runtime_ready = i2c1_runtime_ready && i2c2_runtime_ready;
}

static osStatus_t submit_spi_dma_job(spi_job_t* job){
  if(job == NULL) return osErrorParameter;

  osStatus_t queue_status;
  queue_status = osMessageQueuePut(spi_job_queueHandle, job, 0, 0);
  if(queue_status == osOK && spidmataskHandle != NULL){
    osThreadFlagsSet(spidmataskHandle, SPI_DMA_NEW_JOB_FLAG);
  }
  return queue_status;
}

static ili9341_return_status run_ili9341_dma_operation(
        ili9341_handle_t display){
  if(display == NULL) return _ili9341_fail;

  while(ili9341_operation_is_complete(display) == false){
    spi_job_t job;
    ili9341_return_status display_status;
    display_status = ili9341_build_next_dma_job(display,
            ili9341taskHandle, ILI9341_DMA_SUCCESS_FLAG,
            ILI9341_DMA_ERROR_FLAG, &job);
    if(display_status == _ili9341_complete) return _ili9341_ok;
    if(display_status != _ili9341_ok) return display_status;

    osThreadFlagsClear(ILI9341_DMA_ALL_FLAGS);

    osStatus_t queue_status;
    queue_status = submit_spi_dma_job(&job);
    if(queue_status != osOK){
      ili9341_finish_dma_job(display, false);
      ili9341_dma_error_count++;
      return _ili9341_fail;
    }

    uint32_t result_flags;
    result_flags = osThreadFlagsWait(ILI9341_DMA_ALL_FLAGS,
                                    osFlagsWaitAny,
                                    DMA_DISPATCH_TIMEOUT_MS + 500);
    bool wait_error = (result_flags & osFlagsError) != 0;
    bool dma_succeeded = (result_flags & ILI9341_DMA_SUCCESS_FLAG) != 0;
    bool dma_failed = (result_flags & ILI9341_DMA_ERROR_FLAG) != 0;

    if(wait_error || dma_failed) dma_succeeded = false;

    display_status = ili9341_finish_dma_job(display, dma_succeeded);
    if(display_status != _ili9341_ok){
      ili9341_dma_error_count++;
      return display_status;
    }

    ili9341_dma_success_count++;
    ili9341_last_success_tick = osKernelGetTickCount();
  }

  return _ili9341_ok;
}

static ili9341_return_status draw_ili9341_text(ili9341_handle_t display,
        uint16_t x, uint16_t y, const char* text){
  return draw_ili9341_colored_text(display, x, y, text,
          ILI9341_COLOR_WHITE);
}

static ili9341_return_status draw_ili9341_colored_text(
        ili9341_handle_t display, uint16_t x, uint16_t y, const char* text,
        uint16_t color){
  ili9341_return_status display_status;
  display_status = ili9341_begin_string(display, x, y, text,
          color, ILI9341_COLOR_BLACK);
  if(display_status != _ili9341_ok) return display_status;

  display_status = run_ili9341_dma_operation(display);
  return display_status;
}

static void copy_display_text(char* output, uint8_t output_size,
        const uint8_t* input, uint8_t input_length){
  if(output == NULL || input == NULL || output_size == 0) return;

  uint8_t copy_length = input_length;
  if(copy_length >= output_size) copy_length = output_size - 1;

  uint8_t index = 0;
  while(index < copy_length){
    uint8_t character = input[index];
    if(character < 32 || character > 126) character = ' ';
    output[index] = character;
    index++;
  }
  output[index] = '\0';
}

static void format_text_field(char* output, uint8_t field_width,
        const char* text){
  if(output == NULL || text == NULL || field_width == 0) return;

  uint8_t index = 0;
  while(index < field_width && text[index] != '\0'){
    output[index] = text[index];
    index++;
  }
  while(index < field_width){
    output[index] = ' ';
    index++;
  }
  output[field_width] = '\0';
}

static uint8_t append_unsigned_number(char* output, uint8_t output_index,
        uint32_t value){
  char reverse_digits[10];
  uint8_t digit_count = 0;

  do{
    reverse_digits[digit_count] = '0' + (value % 10);
    digit_count++;
    value /= 10;
  }while(value > 0 && digit_count < sizeof(reverse_digits));

  while(digit_count > 0){
    digit_count--;
    output[output_index] = reverse_digits[digit_count];
    output_index++;
  }

  return output_index;
}

static void format_fixed_2(char* output, uint8_t field_width,
        float value, const char* unit){
  if(output == NULL || unit == NULL || field_width == 0) return;

  bool negative = value < 0.0f;
  float absolute_value = value;
  if(negative) absolute_value = -absolute_value;

  uint32_t scaled_value = (uint32_t)(absolute_value * 100.0f + 0.5f);
  uint32_t integer_part = scaled_value / 100;
  uint32_t fraction_part = scaled_value % 100;

  char formatted_value[32];
  uint8_t output_index = 0;
  if(negative){
    formatted_value[output_index] = '-';
    output_index++;
  }

  output_index = append_unsigned_number(formatted_value, output_index,
          integer_part);
  formatted_value[output_index] = '.';
  output_index++;
  formatted_value[output_index] = '0' + (fraction_part / 10);
  output_index++;
  formatted_value[output_index] = '0' + (fraction_part % 10);
  output_index++;
  formatted_value[output_index] = ' ';
  output_index++;

  uint8_t unit_index = 0;
  while(unit[unit_index] != '\0' && output_index < sizeof(formatted_value) - 1){
    formatted_value[output_index] = unit[unit_index];
    output_index++;
    unit_index++;
  }
  formatted_value[output_index] = '\0';

  format_text_field(output, field_width, formatted_value);
}

static void format_fixed_3(char* output, uint8_t field_width,
        float value, const char* unit){
  if(output == NULL || unit == NULL || field_width == 0) return;

  bool negative = value < 0.0f;
  float absolute_value = negative ? -value : value;
  uint32_t scaled_value = (uint32_t)(absolute_value * 1000.0f + 0.5f);
  uint32_t integer_part = scaled_value / 1000;
  uint32_t fraction_part = scaled_value % 1000;

  char formatted_value[32];
  uint8_t output_index = 0;
  if(negative) formatted_value[output_index++] = '-';

  output_index = append_unsigned_number(formatted_value, output_index,
          integer_part);
  formatted_value[output_index++] = '.';
  formatted_value[output_index++] = '0' + (fraction_part / 100);
  formatted_value[output_index++] = '0' + ((fraction_part / 10) % 10);
  formatted_value[output_index++] = '0' + (fraction_part % 10);
  formatted_value[output_index++] = ' ';

  uint8_t unit_index = 0;
  while(unit[unit_index] != '\0' && output_index < sizeof(formatted_value) - 1){
    formatted_value[output_index++] = unit[unit_index++];
  }
  formatted_value[output_index] = '\0';

  format_text_field(output, field_width, formatted_value);
}

static ili9341_return_status draw_sensor_display_layout(void){
  ili9341_return_status display_status;
  char wait_bme[16];
  char wait_mpu[11];
  char wait_card[22];
  format_text_field(wait_bme, 15, "WAIT");
  format_text_field(wait_mpu, 10, "WAIT");
  format_text_field(wait_card, 21, "WAIT");

  display_status = draw_ili9341_text(my_tft1, 124, 8, "BME280");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 0, 40, "SIC:");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 0, 64, "NEM:");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 0, 88, "BAS:");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 40, wait_bme);
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 64, wait_bme);
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 88, wait_bme);
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 112, 120, "MPU6500");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 0, 144, "IVME");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 160, 144, "JIRO");
  if(display_status != _ili9341_ok) return display_status;

  const uint16_t mpu_y_positions[3] = {168, 192, 216};
  const char* axis_labels[3] = {"X:", "Y:", "Z:"};
  for(uint8_t index = 0; index < 3; index++){
    display_status = draw_ili9341_text(my_tft1, 0,
            mpu_y_positions[index], axis_labels[index]);
    if(display_status != _ili9341_ok) return display_status;
    display_status = draw_ili9341_text(my_tft1, 36,
            mpu_y_positions[index], wait_mpu);
    if(display_status != _ili9341_ok) return display_status;
    display_status = draw_ili9341_text(my_tft1, 160,
            mpu_y_positions[index], axis_labels[index]);
    if(display_status != _ili9341_ok) return display_status;
    display_status = draw_ili9341_text(my_tft1, 196,
            mpu_y_positions[index], wait_mpu);
    if(display_status != _ili9341_ok) return display_status;
  }

  display_status = draw_ili9341_text(my_tft2, 112, 8, "ADXL345");
  if(display_status != _ili9341_ok) return display_status;
  for(uint8_t index = 0; index < 3; index++){
    display_status = draw_ili9341_text(my_tft2, 0, 40 + index * 24,
            axis_labels[index]);
    if(display_status != _ili9341_ok) return display_status;
    display_status = draw_ili9341_text(my_tft2, 36, 40 + index * 24,
            wait_mpu);
    if(display_status != _ili9341_ok) return display_status;
  }

  display_status = draw_ili9341_text(my_tft2, 124, 128, "RC522");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft2, 0, 160,
          "KART: BEKLENIYOR          ");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft2, 0, 192, "DATA:");
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft2, 60, 192, wait_card);
  if(display_status != _ili9341_ok) return display_status;

  return _ili9341_ok;
}

static ili9341_return_status refresh_bme280_display(
        const bme280_display_data_t* data){
  if(data == NULL || data->valid == false) return _ili9341_fail;

  char temperature_text[16];
  char humidity_text[16];
  char pressure_text[16];
  format_fixed_2(temperature_text, 15, data->temperature_c, "C");
  format_fixed_2(humidity_text, 15, data->humidity_percent, "%");
  format_fixed_2(pressure_text, 15, data->pressure_pa / 100.0f, "hPa");

  ili9341_return_status display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 40, temperature_text);
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 64, humidity_text);
  if(display_status != _ili9341_ok) return display_status;
  display_status = draw_ili9341_text(my_tft1, 60, 88, pressure_text);
  return display_status;
}

static ili9341_return_status refresh_mpu6500_display(
        const mpu6500_display_data_t* data){
  if(data == NULL || data->valid == false) return _ili9341_fail;

  const uint16_t y_positions[3] = {168, 192, 216};
  char value_text[11];
  ili9341_return_status display_status = _ili9341_ok;

  for(uint8_t index = 0; index < 3; index++){
    /* Yatay eksenlerdeki kucuk fakat gecerli olcumleri 0.00'a
     * yuvarlayip kaybetmemek icin ivmeyi uc ondalikla goster. */
    format_fixed_3(value_text, 10, data->accel_g[index], "g");
    display_status = draw_ili9341_text(my_tft1, 36,
            y_positions[index], value_text);
    if(display_status != _ili9341_ok) return display_status;
  }

  for(uint8_t index = 0; index < 3; index++){
    format_fixed_2(value_text, 10, data->gyro_dps[index], "dps");
    display_status = draw_ili9341_text(my_tft1, 196,
            y_positions[index], value_text);
    if(display_status != _ili9341_ok) return display_status;
  }

  return _ili9341_ok;
}

static ili9341_return_status refresh_adxl345_display(
        const adxl345_display_data_t* data){
  if(data == NULL || data->valid == false) return _ili9341_fail;

  const float values[3] = {data->x_g, data->y_g, data->z_g};
  char value_text[11];
  for(uint8_t index = 0; index < 3; index++){
    format_fixed_2(value_text, 10, values[index], "g");
    ili9341_return_status display_status = draw_ili9341_text(my_tft2, 36,
            40 + index * 24, value_text);
    if(display_status != _ili9341_ok) return display_status;
  }
  return _ili9341_ok;
}

static ili9341_return_status refresh_rc522_display(
        const rc522_display_data_t* data){
  if(data == NULL) return _ili9341_fail;

  ili9341_return_status display_status;
  if(data->card_found == false){
    display_status = draw_ili9341_text(my_tft2, 0, 160,
            "KART: YOK                 ");
    if(display_status != _ili9341_ok) return display_status;

    return draw_ili9341_text(my_tft2, 60, 192,
            "                     ");
  }

  display_status = draw_ili9341_text(my_tft2, 0, 160,
          "KART: BULUNDU             ");
  if(display_status != _ili9341_ok) return display_status;

  char text_field[22];
  format_text_field(text_field, 21, data->block_text);
  uint16_t text_color = ILI9341_COLOR_WHITE;
  if(strncmp(data->block_text, "Mavi", 4) == 0){
    text_color = ILI9341_COLOR_BLUE;
  }
  return draw_ili9341_colored_text(my_tft2, 60, 192, text_field,
          text_color);
}

static void refresh_i2c_dma_debug(i2c_dispatch_slot_t* active_jobs){
  uint8_t active_count = 0;
  for(uint8_t index = 0; index < 3; index++){
    if(active_jobs[index].active == false) continue;
    bool manager_busy;
    manager_busy = i2c_manager_is_dma_busy(active_jobs[index].job.i2c_handle);
    if(manager_busy) active_count++;
  }

  i2c_dma_active_count = active_count;
  if(active_count > i2c_dma_max_concurrent){
    i2c_dma_max_concurrent = active_count;
  }
}

static void refresh_spi_dma_debug(spi_dispatch_slot_t* active_jobs){
  uint8_t active_count = 0;
  for(uint8_t index = 0; index < 3; index++){
    if(active_jobs[index].active == false) continue;
    bool manager_busy;
    manager_busy = spi_manager_is_dma_busy(active_jobs[index].job.spi_handle);
    if(manager_busy) active_count++;
  }

  spi_dma_active_count = active_count;
  if(active_count > spi_dma_max_concurrent){
    spi_dma_max_concurrent = active_count;
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
  system_health_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Harici beslenen slave'leri, I2C1 ilk kez etkinlesmeden once kurtar.
  i2c1_startup_recovery_success = recover_i2c1_bus_once();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
#if CAN2_RUNTIME_ENABLED
  MX_CAN2_Init();
#endif
  /* USER CODE BEGIN 2 */
  i2c1_startup_sr2_after = I2C1->SR2;
  i2c1_startup_recovery_success = i2c1_startup_recovery_success &&
          (i2c1_startup_sr2_after & I2C_SR2_BUSY) == 0;
  i2c1_startup_recovery_completed = true;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of i2c_mutex */
  i2c_mutexHandle = osMutexNew(&i2c_mutex_attributes);

  /* creation of i2c_init_mutex */
  i2c_init_mutexHandle = osMutexNew(&i2c_init_mutex_attributes);

  i2c2_mutexHandle = osMutexNew(&i2c2_mutex_attributes);

  nrf_mode_mutexHandle = osMutexNew(&nrf_mode_mutex_attributes);

  /* creation of spi_mutex */
  spi_mutexHandle = osMutexNew(&spi_mutex_attributes);

  /* creation of spi3_mutex */
  spi3_mutexHandle = osMutexNew(&spi3_mutex_attributes);

  /* creation of spi2_mutex */
  spi2_mutexHandle = osMutexNew(&spi2_mutex_attributes);

  /* creation of sensor_data_mutex */
  sensor_data_mutexHandle = osMutexNew(&sensor_data_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dma_semaphore */
  dma_semaphoreHandle = osSemaphoreNew(1, 1, &dma_semaphore_attributes);

  i2c2_dma_semaphoreHandle = osSemaphoreNew(1, 1,
          &i2c2_dma_semaphore_attributes);

  /* creation of spi_semaphore */
  spi_semaphoreHandle = osSemaphoreNew(1, 1, &spi_semaphore_attributes);

  /* creation of spi3_semaphore */
  spi3_semaphoreHandle = osSemaphoreNew(1, 1, &spi3_semaphore_attributes);

  /* creation of spi2_semaphore */
  spi2_semaphoreHandle = osSemaphoreNew(1, 1, &spi2_semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  i2c_manager_assign_bus(I2C1, i2c_mutexHandle, dma_semaphoreHandle);
  i2c_manager_set_bus_recovery(I2C1, recover_i2c1_bus_runtime);
  i2c_manager_assign_bus(I2C2, i2c2_mutexHandle, i2c2_dma_semaphoreHandle);
  i2c_manager_set_bus_recovery(I2C2, recover_i2c2_bus_runtime);
  spi_manager_assign_bus(SPI1, spi_mutexHandle, spi_semaphoreHandle);
  spi_manager_assign_bus(SPI3, spi3_mutexHandle, spi3_semaphoreHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of i2c_job_queue */
  i2c_job_queueHandle = osMessageQueueNew (8, sizeof(i2c_job_t), &i2c_job_queue_attributes);

  /* creation of spi_job_queue */
  spi_job_queueHandle = osMessageQueueNew (8, sizeof(spi_job_t), &spi_job_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of bme280task */
  bme280taskHandle = osThreadNew(startbme280task, NULL, &bme280task_attributes);

  /* creation of i2cdmatask */
  i2cdmataskHandle = osThreadNew(starti2cdmatask, NULL, &i2cdmatask_attributes);

  /* creation of rc522task */
#if !I2C_ONLY_TEST
  rc522taskHandle = osThreadNew(startrc522task, NULL, &rc522task_attributes);
#endif

  /* creation of mpu6500task */
  mpu6500taskHandle = osThreadNew(startmpu6500task, NULL, &mpu6500task_attributes);

  /* creation of spidmatask */
#if !I2C_ONLY_TEST
  spidmataskHandle = osThreadNew(startspidmatask, NULL, &spidmatask_attributes);
#endif

  /* creation of nrf24l01TXtask */
#if !I2C_ONLY_TEST
  nrf24l01TXtaskHandle = osThreadNew(startnrf24l01TXtask, NULL, &nrf24l01TXtask_attributes);
#endif

  /* creation of nrf24l01RXtask */
#if !I2C_ONLY_TEST
  nrf24l01RXtaskHandle = osThreadNew(startnrf24l01RXtask, NULL, &nrf24l01RXtask_attributes);
#endif

  /* creation of ili9341task */
#if !I2C_ONLY_TEST
  ili9341taskHandle = osThreadNew(startili9341task, NULL, &ili9341task_attributes);
#endif

  /* creation of healthtask */
  healthtaskHandle = osThreadNew(starthealthtask, NULL, &healthtask_attributes);

  /* creation of CAN2ReadTask */
#if CAN2_RUNTIME_ENABLED
  CAN2ReadTaskHandle = osThreadNew(StartCAN2ReadTask, NULL, &CAN2ReadTask_attributes);
#endif

  /* creation of CAN2WriteTask */
#if CAN2_RUNTIME_ENABLED
  CAN2WriteTaskHandle = osThreadNew(StartCAN2WriteTask, NULL, &CAN2WriteTask_attributes);
#endif

  adxl345taskHandle = osThreadNew(startadxl345task, NULL,
          &adxl345task_attributes);
#if !I2C_ONLY_TEST
  ssd1306spitaskHandle = osThreadNew(startssd1306spitask, NULL,
          &ssd1306spitask_attributes);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  system_health_register_task(HEALTH_TASK_BME280, bme280taskHandle);
  system_health_register_task(HEALTH_TASK_I2C_DISPATCH, i2cdmataskHandle);
#if !I2C_ONLY_TEST
  system_health_register_task(HEALTH_TASK_RC522, rc522taskHandle);
#endif
  system_health_register_task(HEALTH_TASK_MPU6500, mpu6500taskHandle);
#if !I2C_ONLY_TEST
  system_health_register_task(HEALTH_TASK_SPI_DISPATCH, spidmataskHandle);
  system_health_register_task(HEALTH_TASK_NRF_TX, nrf24l01TXtaskHandle);
  system_health_register_task(HEALTH_TASK_NRF_RX, nrf24l01RXtaskHandle);
  system_health_register_task(HEALTH_TASK_DISPLAY, ili9341taskHandle);
  system_health_register_task(HEALTH_TASK_SSD1306_SPI, ssd1306spitaskHandle);
#endif
  system_health_register_task(HEALTH_TASK_ADXL345, adxl345taskHandle);
#if CAN2_RUNTIME_ENABLED
  system_health_register_task(HEALTH_TASK_CAN_RX, CAN2ReadTaskHandle);
  system_health_register_task(HEALTH_TASK_CAN_TX, CAN2WriteTaskHandle);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 160, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
#if CAN2_RUNTIME_ENABLED
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 8;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}
#endif

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 DMA Init */

  /* I2C1_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function (PB10=SCL, PB11=SDA)
  */
static void MX_I2C2_Init(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_7);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3,
          LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_7, LL_DMA_CHANNEL_7);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_7,
          LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

  NVIC_SetPriority(I2C2_EV_IRQn,
          NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn,
          NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* SPI1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  PC12   ------> SPI3_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */

  /* SPI3_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* SPI3_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_2, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  /* SPI3 interrupt Init */
  NVIC_SetPriority(SPI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* I2C2 RX */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* I2C2 TX */
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, TFT1_DC_Pin|TFT2_DC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(SSD1306_SPI_DC_GPIO_Port, SSD1306_SPI_DC_Pin);

  /**/
  LL_GPIO_ResetOutputPin(nrf24_ce_pin_spi1_GPIO_Port, nrf24_ce_pin_spi1_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOE, TFT1_RST_Pin|TFT1_CS_Pin|TFT2_RST_Pin|TFT2_CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(SSD1306_SPI_CS_GPIO_Port,
          SSD1306_SPI_CS_Pin | SSD1306_SPI_RST_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOD, nrf24_csn_pin_spi1_Pin|rc522_cs_pin_Pin|rc522_rst_pin_Pin);

  /* Haberlesme LED'leri baslangicta F103'un ilk fazina uygun olarak
     CAN LOW, NRF HIGH durumundadir. */
  LL_GPIO_ResetOutputPin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin);
  LL_GPIO_SetOutputPin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE7);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE2);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE3);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(mpu6500_irq_pin_GPIO_Port, mpu6500_irq_pin_Pin, LL_GPIO_PULL_DOWN);

  /**/
  LL_GPIO_SetPinPull(nrf24_irq_pin_spi1_GPIO_Port, nrf24_irq_pin_spi1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(rc522_irq_pin_GPIO_Port, rc522_irq_pin_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(mpu6500_irq_pin_GPIO_Port, mpu6500_irq_pin_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(nrf24_irq_pin_spi1_GPIO_Port, nrf24_irq_pin_spi1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(rc522_irq_pin_GPIO_Port, rc522_irq_pin_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = TFT1_DC_Pin|TFT1_RST_Pin|TFT1_CS_Pin|TFT2_DC_Pin
                          |TFT2_RST_Pin|TFT2_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SSD1306_SPI_CS_Pin | SSD1306_SPI_DC_Pin |
          SSD1306_SPI_RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = nrf24_ce_pin_spi1_Pin|rc522_cs_pin_Pin|rc522_rst_pin_Pin|
          CAN_LINK_LED_Pin|NRF_LINK_LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = nrf24_csn_pin_spi1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(nrf24_csn_pin_spi1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI2_IRQn);
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // Ekranlar SPI3'u ayri CS pinleriyle ortak kullanıyor. İkisini de başlangıçta seçimsiz bırak.
  LL_GPIO_SetOutputPin(GPIOE, TFT1_CS_Pin|TFT1_RST_Pin|TFT2_CS_Pin|TFT2_RST_Pin);
  LL_GPIO_ResetOutputPin(GPIOE, TFT1_DC_Pin|TFT2_DC_Pin);

  GPIO_InitStruct.Pin = TFT1_CS_Pin|TFT1_DC_Pin|TFT1_RST_Pin|
                        TFT2_CS_Pin|TFT2_DC_Pin|TFT2_RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startbme280task */
/**
  * @brief  Function implementing the bme280task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startbme280task */
void startbme280task(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t startup_flags = osThreadFlagsWait(I2C_WORKER_READY_FLAG,
	        osFlagsWaitAny, osWaitForever);
	if((startup_flags & I2C_WORKER_READY_FLAG) == 0){
	  for(;;) osDelay(1000);
	}

	bme280_user_configs bme_cfg = {0};

	bme_cfg.i2c_handle = I2C1;
	bme_cfg.dma_handle = DMA1;
	bme_cfg.dma_stream = LL_DMA_STREAM_0;
	bme_cfg.i2c_addr = bme280_i2c_addr0;

	bme_cfg.config_t.bits.t_sb = 0;
	bme_cfg.config_t.bits.filter = 0;
	bme_cfg.config_t.bits.spi_en = 0;
	bme_cfg.ctrlhum_t.bits.osrs_h = 1;
	bme_cfg.ctrlmeas_t.bits.osrs_t = 1;
	bme_cfg.ctrlmeas_t.bits.osrs_p = 1;
	bme_cfg.ctrlmeas_t.bits.mode = 3;

	while(my_bme280 == NULL){
	  system_health_heartbeat(HEALTH_TASK_BME280);
	  if(acquire_i2c_init_lock()){
	    my_bme280 = bme280_init(&bme_cfg);
	    release_i2c_init_lock();
	  }
	  if(my_bme280 == NULL) osDelay(1000);
	}
	bme280_init_completed = true;
	bme280_last_success_tick = osKernelGetTickCount();
	register_i2c_sensor_client_ready(I2C1);

	uint32_t runtime_flags = osThreadFlagsWait(I2C_RUNTIME_READY_FLAG,
	        osFlagsWaitAny, osWaitForever);
	if((runtime_flags & I2C_RUNTIME_READY_FLAG) == 0){
	  for(;;) osDelay(1000);
	}

	const uint32_t bme280_period_ms = 100;
	uint32_t bme280_next_wake = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  system_health_heartbeat(HEALTH_TASK_BME280);
	  bme280_next_wake += bme280_period_ms;
	  osDelayUntil(bme280_next_wake);

	  i2c_job_t job;
	  osThreadFlagsClear(BME280_DMA_ALL_FLAGS);
	  if(bme280_build_read_job(my_bme280, bme280taskHandle,
	          BME280_DMA_SUCCESS_FLAG, BME280_DMA_ERROR_FLAG,
	          &job) == _bme280_ok)
	  {
	      if(submit_i2c_dma_job(&job) == osOK)
	      {
	          uint32_t result_flags = osThreadFlagsWait(BME280_DMA_ALL_FLAGS,
	                  osFlagsWaitAny, 1000);
	          if((result_flags & BME280_DMA_SUCCESS_FLAG) != 0)
	          {
	              bme280_return_status bme_status;
	              bme_status = bme280_get_value(my_bme280);
	              if(bme_status == _bme280_ok){
	                float temperature;
	                float humidity;
	                float pressure;
	                bme280_get_measurements(my_bme280, &temperature,
	                        &humidity, &pressure);
	                bme280_last_success_tick = osKernelGetTickCount();

	                osStatus_t mutex_status;
	                mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
	                if(mutex_status == osOK){
	                  bme280_display_data.temperature_c = temperature;
	                  bme280_display_data.humidity_percent = humidity;
	                  bme280_display_data.pressure_pa = pressure;
	                  bme280_display_data.valid = true;
	                  bme280_display_data.update_count++;
	                  osMutexRelease(sensor_data_mutexHandle);
	                }
	              }
	          }
	      }
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_starti2cdmatask */
/**
* @brief Function implementing the parallel I2C DMA dispatcher thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_starti2cdmatask */
void starti2cdmatask(void *argument)
{
  /* USER CODE BEGIN starti2cdmatask */
  system_health_heartbeat(HEALTH_TASK_I2C_DISPATCH);

  i2c_dispatch_slot_t active_jobs[3] = {0};
  const uint32_t completion_flags[3] = {
    I2C1_DMA_DONE_FLAG,
    I2C2_DMA_DONE_FLAG,
    I2C3_DMA_DONE_FLAG
  };

  i2c_manager_set_dispatch_notification(I2C1, i2cdmataskHandle, I2C1_DMA_DONE_FLAG);
  i2c_manager_set_dispatch_notification(I2C2, i2cdmataskHandle, I2C2_DMA_DONE_FLAG);
  i2c_manager_set_dispatch_notification(I2C3, i2cdmataskHandle, I2C3_DMA_DONE_FLAG);
  i2c_manager_set_job_service(i2c_job_queueHandle, i2cdmataskHandle,
          I2C_DMA_NEW_JOB_FLAG);

  while(osSemaphoreAcquire(dma_semaphoreHandle, 0) == osOK) { }
  osThreadFlagsClear(I2C_DMA_ALL_FLAGS);

  bool i2c1_workers_released = false;
  bool i2c2_workers_released = false;
  uint32_t next_startup_retry_tick = 0;

  /* Infinite loop */
  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_I2C_DISPATCH);
    uint32_t pending_flags;
    pending_flags = osThreadFlagsWait(I2C_DMA_ALL_FLAGS, osFlagsWaitAny, 10);
    bool flag_wait_error = (pending_flags & osFlagsError) != 0;
    if(flag_wait_error) pending_flags = 0;
    uint32_t now = osKernelGetTickCount();

    /* Her I2C hatti kendi basina hazirlanir. Bir hattin fiziksel arizasi,
     * diger hattaki sensor task'larinin baslamasini engellemez. */
    if((int32_t)(now - next_startup_retry_tick) >= 0){
      if(!i2c1_workers_released){
        bool i2c1_ready = i2c1_startup_recovery_completed &&
                i2c1_startup_recovery_success;
        if(!i2c1_ready){
          i2c1_ready = i2c_manager_recover_bus(I2C1) ==
                  _i2c_manager_ok;
        }
        if(i2c1_ready){
          osThreadFlagsSet(bme280taskHandle, I2C_WORKER_READY_FLAG);
          osThreadFlagsSet(mpu6500taskHandle, I2C_WORKER_READY_FLAG);
          i2c1_workers_released = true;
        }
      }

      if(!i2c2_workers_released){
        bool i2c2_ready = i2c_manager_recover_bus(I2C2) ==
                _i2c_manager_ok;
        if(i2c2_ready){
          osThreadFlagsSet(adxl345taskHandle, I2C_WORKER_READY_FLAG);
          i2c2_workers_released = true;
        }
      }

      next_startup_retry_tick = now + 1000;
    }

    for(uint8_t index = 0; index < 3; index++){
      bool transfer_finished = (pending_flags & completion_flags[index]) != 0;
      if(transfer_finished && active_jobs[index].active){
        i2c_manager_return_status completed_status;
        completed_status = i2c_manager_unlock_bus(
                active_jobs[index].job.i2c_handle);
        complete_i2c_job(&active_jobs[index].job, completed_status);
        active_jobs[index].active = false;
        refresh_i2c_dma_debug(active_jobs);
      }

      if(active_jobs[index].active){
        uint32_t elapsed_time = now - active_jobs[index].start_tick;
        if(elapsed_time >= DMA_DISPATCH_TIMEOUT_MS){
          i2c_manager_abort_transfer(active_jobs[index].job.i2c_handle);
          complete_i2c_job(&active_jobs[index].job,
                  _i2c_manager_aborted);
          active_jobs[index].active = false;
          refresh_i2c_dma_debug(active_jobs);
        }
      }
    }

    uint32_t queued_job_count = osMessageQueueGetCount(i2c_job_queueHandle);
    for(uint32_t queue_index = 0; queue_index < queued_job_count; queue_index++){
      i2c_job_t job;
      osStatus_t queue_status;
      queue_status = osMessageQueueGet(i2c_job_queueHandle, &job, NULL, 0);
      if(queue_status != osOK) break;

      int8_t dispatch_index = get_i2c_dispatch_index(job.i2c_handle);
      if(dispatch_index < 0){
        complete_i2c_job(&job, _i2c_manager_uninited_struct);
        continue;
      }

      bool polling_job = job.operation != I2C_JOB_READ_DMA &&
              job.operation != I2C_JOB_WRITE_DMA;
      if(polling_job){
        if(active_jobs[dispatch_index].active){
          requeue_i2c_job(&job);
          continue;
        }

        i2c_last_polling_job_status =
                i2c_manager_execute_polling_job(&job);
        i2c_polling_job_count++;
        complete_i2c_job(&job, i2c_last_polling_job_status);
        continue;
      }

      bool dma_resource_busy = false;
      for(uint8_t active_index = 0; active_index < 3; active_index++){
        if(active_jobs[active_index].active == false) continue;
        bool same_dma = active_jobs[active_index].job.dma_handle == job.dma_handle;
        bool same_stream = active_jobs[active_index].job.dma_stream == job.dma_stream;
        if(same_dma && same_stream){
          dma_resource_busy = true;
          break;
        }
      }

      if(active_jobs[dispatch_index].active || dma_resource_busy){
        requeue_i2c_job(&job);
        continue;
      }

      active_jobs[dispatch_index].active = true;
      active_jobs[dispatch_index].start_tick = osKernelGetTickCount();
      active_jobs[dispatch_index].job = job;

      i2c_manager_return_status transfer_status;
      if(job.operation == I2C_JOB_WRITE_DMA){
        transfer_status = i2c_manager_write_dma(job.dma_handle,
                job.dma_stream, job.i2c_handle, job.dev_addr,
                job.reg_addr, job.txdata, job.size);
      }
      else{
        transfer_status = i2c_manager_read_dma(job.dma_handle,
                job.dma_stream, job.i2c_handle, job.dev_addr,
                job.reg_addr, job.rxdata, job.size);
      }
      if(transfer_status == _i2c_manager_ok){
        i2c_dma_job_count++;
        refresh_i2c_dma_debug(active_jobs);
        continue;
      }

      active_jobs[dispatch_index].active = false;
      if(transfer_status == _i2c_manager_busy){
        /* Bu durum yalnizca kisa sureli mutex/kaynak cakismasidir. Fiziksel
         * BUSY i2c_manager icinde bir kez kurtarilir; kurtarilamazsa hata ile
         * tamamlanir ve sonsuz requeue dongusune girmez. */
        if(job.i2c_handle == I2C1) i2c1_runtime_busy_requeue_count++;
        requeue_i2c_job(&job);
      }
      else{
        complete_i2c_job(&job, transfer_status);
      }
    }
  }
  /* USER CODE END starti2cdmatask */
}

/* USER CODE BEGIN Header_startrc522task */
/**
* @brief Function implementing the rc522task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startrc522task */
void startrc522task(void *argument)
{
  /* USER CODE BEGIN startrc522task */
	rc522_user_configs rc522_cfg = {0};

	rc522_cfg.spi_handle = SPI1;
	rc522_cfg.cs_port = rc522_cs_pin_GPIO_Port;
	rc522_cfg.cs_pin = rc522_cs_pin_Pin;
	rc522_cfg.rst_port = rc522_rst_pin_GPIO_Port;
	rc522_cfg.rst_pin = rc522_rst_pin_Pin;

	while(my_rc522 == NULL){
	  system_health_heartbeat(HEALTH_TASK_RC522);
	  my_rc522 = rc522_init(&rc522_cfg);
	  if(my_rc522 == NULL) osDelay(1000);
	}
	rc522_version = rc522_get_version(my_rc522);
	rc522_last_observed_version = rc522_version;
	rc522_init_completed = true;
	(void)spi_manager_register_validator(SPI1, validate_rc522_spi, my_rc522);
	rc522_last_liveness_tick = osKernelGetTickCount();
	osStatus_t initial_display_mutex;
	initial_display_mutex = osMutexAcquire(sensor_data_mutexHandle, 10);
	if(initial_display_mutex == osOK){
	  rc522_display_data.card_found = false;
	  memset(rc522_display_data.block_text, 0,
	          sizeof(rc522_display_data.block_text));
	  rc522_display_data.update_count++;
	  osMutexRelease(sensor_data_mutexHandle);
	}

	rc522_assign_interrupt_task(my_rc522, rc522taskHandle, 0x04);
	rc522_set_active_irq_device(my_rc522);

	uint8_t card_type[2];
	uint8_t card_uid[5];
	uint8_t status;
	uint8_t card_size;

	uint8_t keyA[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t block_to_read = 4;
	uint8_t block_data[16];
	uint8_t block_data2[16];
	int16_t counter = 0;
	uint8_t counter2 = 0;
	uint8_t data_to_write[16]  = "Beyaz :)";
	uint8_t data_to_write2[16] = "Mavi :)";
	const uint32_t card_absent_timeout_ms = 1500;
	uint32_t last_card_seen_tick = osKernelGetTickCount();
	uint32_t next_liveness_check_tick = osKernelGetTickCount();
	bool card_display_active = false;
	uint8_t displayed_card_uid[5] = {0};
	bool card_present = false;
	uint8_t last_detected_uid[5] = {0};
	uint32_t consecutive_protocol_error_count = 0;

  /* Infinite loop */
  for(;;)
  {
	  system_health_heartbeat(HEALTH_TASK_RC522);
	  uint32_t loop_tick = osKernelGetTickCount();
	  if((int32_t)(loop_tick - next_liveness_check_tick) >= 0){
	    next_liveness_check_tick = loop_tick +
	            RC522_LIVENESS_CHECK_PERIOD_MS;
	    uint8_t current_version = rc522_get_version(my_rc522);
	    rc522_last_observed_version = current_version;
	    if(current_version == rc522_version){
	      rc522_last_liveness_tick = loop_tick;
	    }
	    else{
	      rc522_liveness_error_count++;
	    }
	  }
	  memset(card_type, 0, 2);
	  memset(card_uid, 0, 5);
	  card_size = 0;

	  rc522_stop_crypto1(my_rc522);

	  status = rc522_request(my_rc522, PICC_REQALL, card_type);
	  rc522_last_request_status = status;
	  if(status == MI_ERROR) consecutive_protocol_error_count++;
	  else consecutive_protocol_error_count = 0;
	  if(status == MI_OK){
		  status = rc522_anticoll(my_rc522, card_uid);
		  rc522_last_anticoll_status = status;
		  if(status == MI_ERROR) consecutive_protocol_error_count++;
		  else consecutive_protocol_error_count = 0;
		  if(status == MI_OK){
			  bool new_card_event = card_present == false ||
			          memcmp(last_detected_uid, card_uid,
			                  sizeof(last_detected_uid)) != 0;
			  last_card_seen_tick = osKernelGetTickCount();
			  if(new_card_event){
			    rc522_detection_count++;
			    memcpy(last_detected_uid, card_uid,
			            sizeof(last_detected_uid));
			  }
			  card_present = true;
			  for(uint8_t uid_index = 0; uid_index < 5; uid_index++){
				  rc522_last_uid[uid_index] = card_uid[uid_index];
			  }

			  if((card_uid[0] == 193) && (card_uid[1] == 99) && (card_uid[2] == 247) && (card_uid[3] == 3) && (card_uid[4] == 86)){
				  card_size = rc522_select_tag(my_rc522, card_uid);
				  if(card_size != 0){
					  status = rc522_auth(my_rc522, PICC_AUTHENT1A, block_to_read, keyA, card_uid);
					  if(status == MI_OK){
						  if(counter == 0){
							  counter++;
							  rc522_write(my_rc522, block_to_read, data_to_write);
						  }
						  status = rc522_read(my_rc522, block_to_read, block_data);
						  if(status == MI_OK){
						    last_card_seen_tick = osKernelGetTickCount();
						    bool card_changed = card_display_active == false ||
						            memcmp(displayed_card_uid, card_uid,
						                    sizeof(displayed_card_uid)) != 0;
						    if(card_changed){
						      osStatus_t mutex_status;
						      mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
						      if(mutex_status == osOK){
						        rc522_display_data.card_found = true;
						        copy_display_text(rc522_display_data.block_text,
						                sizeof(rc522_display_data.block_text), block_data, 16);
						        rc522_display_data.update_count++;
						        osMutexRelease(sensor_data_mutexHandle);
						        memcpy(displayed_card_uid, card_uid,
						                sizeof(displayed_card_uid));
						        card_display_active = true;
						      }
						    }
						  }
						  rc522_halt(my_rc522);
					  }
				  }
			  }

			  if((card_uid[0] == 162) && (card_uid[1] == 98) && (card_uid[2] == 192) && (card_uid[3] == 1) && (card_uid[4] == 1)){
				  card_size = rc522_select_tag(my_rc522, card_uid);
				  if(card_size != 0){
					  status = rc522_auth(my_rc522, PICC_AUTHENT1A, block_to_read, keyA, card_uid);
					  if(status == MI_OK){
						  if(counter2 == 0){
							  counter2++;
							  rc522_write(my_rc522, block_to_read, data_to_write2);
						  }
						  status = rc522_read(my_rc522, block_to_read, block_data2);
						  if(status == MI_OK){
						    last_card_seen_tick = osKernelGetTickCount();
						    bool card_changed = card_display_active == false ||
						            memcmp(displayed_card_uid, card_uid,
						                    sizeof(displayed_card_uid)) != 0;
						    if(card_changed){
						      osStatus_t mutex_status;
						      mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
						      if(mutex_status == osOK){
						        rc522_display_data.card_found = true;
						        copy_display_text(rc522_display_data.block_text,
						                sizeof(rc522_display_data.block_text), block_data2, 16);
						        rc522_display_data.update_count++;
						        osMutexRelease(sensor_data_mutexHandle);
						        memcpy(displayed_card_uid, card_uid,
						                sizeof(displayed_card_uid));
						        card_display_active = true;
						      }
						    }
						  }
						  rc522_halt(my_rc522);
					  }
				  }
			  }
		  }
	  }
	  if(consecutive_protocol_error_count >= 5U){
	    (void)rc522_recover(my_rc522);
	    consecutive_protocol_error_count = 0;
	    rc522_last_observed_version = rc522_get_version(my_rc522);
	  }

	  uint32_t now = osKernelGetTickCount();
	  if((card_present || card_display_active) &&
	     (now - last_card_seen_tick) >= card_absent_timeout_ms){
	    card_present = false;
	    memset(last_detected_uid, 0, sizeof(last_detected_uid));
	    osStatus_t mutex_status;
	    mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
	    if(mutex_status == osOK){
	      rc522_display_data.card_found = false;
	      memset(rc522_display_data.block_text, 0,
	              sizeof(rc522_display_data.block_text));
	      rc522_display_data.update_count++;
	      osMutexRelease(sensor_data_mutexHandle);
	      memset(displayed_card_uid, 0, sizeof(displayed_card_uid));
	      card_display_active = false;
	    }
	  }
	  osDelay(50);
  }
  /* USER CODE END startrc522task */
}

/* USER CODE BEGIN Header_startmpu6500task */
/**
* @brief Function implementing the mpu6500task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startmpu6500task */
void startmpu6500task(void *argument)
{
  /* USER CODE BEGIN startmpu6500task */
	uint32_t startup_flags = osThreadFlagsWait(I2C_WORKER_READY_FLAG,
	        osFlagsWaitAny, osWaitForever);
	if((startup_flags & I2C_WORKER_READY_FLAG) == 0){
	  for(;;) osDelay(1000);
	}

	mpu6500_user_configs mpu_cfg = {0};

	mpu_cfg.i2c_handle = I2C1;
	mpu_cfg.i2c_addr = mpu6500_i2c_addr_0;
	mpu_cfg.dma_handle = DMA1;
	mpu_cfg.dma_stream = LL_DMA_STREAM_0;
	mpu_cfg.sample_rate = 9;

	mpu_cfg.config_t.bits.dlpf_cfg = 3;
	mpu_cfg.config_t.bits.ext_sync_set = 0;
	mpu_cfg.gyro_config_t.bits.fs_sel = 0;
	mpu_cfg.accel_config_t.bits.afs_sel = 0;

	// FIFO bu canli veri uygulamasinda iki modda da kapali.
	mpu_cfg.fifo_en_t.raw = 0;
	mpu_cfg.user_ctrl_t.raw = 0;

#if MPU6500_USE_DATA_READY_INTERRUPT
	mpu_cfg.int_pin_cfg_t.bits.int_level = 0;
	mpu_cfg.int_pin_cfg_t.bits.int_open = 0;
	mpu_cfg.int_pin_cfg_t.bits.latch_int_en = 1;
	/* Veri register'larinin okunmasi interrupt'i temizlemesin. DMA veri
	 * transferinden sonra INT_STATUS ayrica okunarak latch temizlenecek. */
	mpu_cfg.int_pin_cfg_t.bits.int_rd_clear = 0;
	mpu_cfg.int_enable_t.bits.data_rdy_en = 1;
#else
	// Kontrollu A/B testi: MPU interrupt kaynaklari tamamen kapali.
	mpu_cfg.int_pin_cfg_t.raw = 0;
	mpu_cfg.int_enable_t.raw = 0;
#endif

	mpu_cfg.pwr_mgmt_1_t.bits.sleep = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.clksel = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.temp_dis = 0;
	mpu_cfg.pwr_mgmt_1_t.bits.cycle = 0;
	mpu_cfg.pwr_mgmt_2_t.raw = 0;

	bool mpu_configured = false;
	while(!mpu_configured){
	  system_health_heartbeat(HEALTH_TASK_MPU6500);
	  if(acquire_i2c_init_lock()){
	    if(my_mpu6500 == NULL) my_mpu6500 = mpu6500_init(&mpu_cfg);
	    if(my_mpu6500 != NULL){
	      mpu_configured =
	              mpu6500_configurate(my_mpu6500) == _mpu6500_ok;
	    }
	    release_i2c_init_lock();
	  }
	  if(!mpu_configured) osDelay(1000);
	}

#if MPU6500_USE_DATA_READY_INTERRUPT
	mpu6500_assign_interrupt_task(my_mpu6500, mpu6500taskHandle,
	        MPU6500_DATA_READY_FLAG);
	mpu6500_set_active_irq_device(my_mpu6500);
#endif

#if MPU6500_USE_DATA_READY_INTERRUPT
	mpu6500_int_status_t startup_int_status = {0};
	mpu6500_startup_int_clear_success =
	        mpu6500_read_int_status(my_mpu6500, &startup_int_status) ==
	        _mpu6500_ok;
	mpu6500_startup_int_was_data_ready = startup_int_status.data_rdy;
	osThreadFlagsClear(MPU6500_DATA_READY_FLAG);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
#endif
	mpu6500_init_completed = true;
	mpu6500_last_success_tick = osKernelGetTickCount();
	register_i2c_sensor_client_ready(I2C1);

	uint32_t runtime_flags = osThreadFlagsWait(I2C_RUNTIME_READY_FLAG,
	        osFlagsWaitAny, osWaitForever);
	if((runtime_flags & I2C_RUNTIME_READY_FLAG) == 0){
	  for(;;) osDelay(1000);
	}

#if !MPU6500_USE_DATA_READY_INTERRUPT
	const uint32_t mpu6500_period_ms = 100;
	uint32_t mpu6500_next_wake = osKernelGetTickCount();
#endif

  /* Infinite loop */
  for(;;)
  {
	  system_health_heartbeat(HEALTH_TASK_MPU6500);
#if MPU6500_USE_DATA_READY_INTERRUPT
	  uint32_t irq_flags = osThreadFlagsWait(MPU6500_DATA_READY_FLAG,
	          osFlagsWaitAny, 100);
	  bool irq_received = (irq_flags & MPU6500_DATA_READY_FLAG) != 0;
	  bool interrupt_pin_high = LL_GPIO_IsInputPinSet(
	          mpu6500_irq_pin_GPIO_Port, mpu6500_irq_pin_Pin) != 0;

	  if(irq_received){
	    mpu6500_last_irq_tick = osKernelGetTickCount();
	  }
	  else if(interrupt_pin_high){
	    /* Rising edge kacmis fakat latched INT seviyesi goruluyor. */
	    mpu6500_irq_level_recovery_count++;
	  }
	  else{
	    /* Ne EXTI flag'i ne de HIGH seviyesi var. INT kablosu/kenari kaybolsa
	     * bile MPU veri akisini tamamen durdurma; 100 ms'de bir DMA okuyarak
	     * sistemi calisir tut ve olayi ayri sayaclarda gorunur yap. */
	    mpu6500_irq_timeout_count++;
	    mpu6500_poll_fallback_count++;
	  }

	  // Tek etkin MPU kaynagi DATA_READY. INT_RD_CLEAR=0: once 14 baytlik
	  // veri okunur, sonra INT_STATUS ayrica okunarak interrupt temizlenir.
	  i2c_job_t job;
	  osThreadFlagsClear(MPU6500_DMA_SUCCESS_FLAG |
	          MPU6500_DMA_ERROR_FLAG);
	  mpu6500_return_status build_status;
	  build_status = mpu6500_build_data_job(my_mpu6500,
	          mpu6500taskHandle, MPU6500_DMA_SUCCESS_FLAG,
	          MPU6500_DMA_ERROR_FLAG, &job);
	  if(build_status != _mpu6500_ok){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  if(submit_i2c_dma_job(&job) != osOK){
	    mpu6500_queue_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  uint32_t dma_flags = osThreadFlagsWait(
	          MPU6500_DMA_SUCCESS_FLAG | MPU6500_DMA_ERROR_FLAG,
	          osFlagsWaitAny, 500);
	  bool wait_error = (dma_flags & osFlagsError) != 0;
	  bool dma_finished = (dma_flags & MPU6500_DMA_SUCCESS_FLAG) != 0;
	  if(wait_error || dma_finished == false){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  mpu6500_int_status_t runtime_int_status = {0};
	  if(mpu6500_read_int_status(my_mpu6500, &runtime_int_status) !=
	          _mpu6500_ok){
	    mpu6500_int_status_clear_error_count++;
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }
	  mpu6500_last_int_status = runtime_int_status.raw;
	  mpu6500_int_status_clear_success_count++;

	  if(mpu6500_get_values(my_mpu6500) != _mpu6500_ok){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }
	  mpu6500_last_success_tick = osKernelGetTickCount();

	  float accel[3];
	  float gyro[3];
	  mpu6500_get_accel_g(my_mpu6500, accel);
	  mpu6500_get_gyro_dps(my_mpu6500, gyro);

	  osStatus_t mutex_status;
	  mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
	  if(mutex_status == osOK){
	    for(uint8_t index = 0; index < 3; index++){
	      mpu6500_display_data.accel_g[index] = accel[index];
	      mpu6500_display_data.gyro_dps[index] = gyro[index];
	    }
	    mpu6500_display_data.valid = true;
	    mpu6500_display_data.update_count++;
	    osMutexRelease(sensor_data_mutexHandle);
	    mpu6500_dma_success_count++;
	  }
#else
	  mpu6500_next_wake += mpu6500_period_ms;
	  osDelayUntil(mpu6500_next_wake);

	  i2c_job_t job;
	  osThreadFlagsClear(MPU6500_ALL_FLAGS);
	  mpu6500_return_status build_status;
	  build_status = mpu6500_build_data_job(my_mpu6500,
	          mpu6500taskHandle, MPU6500_DMA_SUCCESS_FLAG,
	          MPU6500_DMA_ERROR_FLAG, &job);
	  if(build_status != _mpu6500_ok){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  if(submit_i2c_dma_job(&job) != osOK){
	    mpu6500_queue_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  uint32_t dma_flags = osThreadFlagsWait(
	          MPU6500_DMA_SUCCESS_FLAG | MPU6500_DMA_ERROR_FLAG,
	          osFlagsWaitAny, 500);
	  bool wait_error = (dma_flags & osFlagsError) != 0;
	  bool dma_finished = (dma_flags & MPU6500_DMA_SUCCESS_FLAG) != 0;
	  if(wait_error || dma_finished == false){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }

	  if(mpu6500_get_values(my_mpu6500) != _mpu6500_ok){
	    mpu6500_dma_error_count++;
	    osDelay(I2C_SENSOR_ERROR_BACKOFF_MS);
	    continue;
	  }
	  mpu6500_last_success_tick = osKernelGetTickCount();

	  float accel[3];
	  float gyro[3];
	  mpu6500_get_accel_g(my_mpu6500, accel);
	  mpu6500_get_gyro_dps(my_mpu6500, gyro);

	  osStatus_t mutex_status;
	  mutex_status = osMutexAcquire(sensor_data_mutexHandle, 10);
	  if(mutex_status == osOK){
	    for(uint8_t index = 0; index < 3; index++){
	      mpu6500_display_data.accel_g[index] = accel[index];
	      mpu6500_display_data.gyro_dps[index] = gyro[index];
	    }
	    mpu6500_display_data.valid = true;
	    mpu6500_display_data.update_count++;
	    osMutexRelease(sensor_data_mutexHandle);
	    mpu6500_dma_success_count++;
	  }
#endif
  }
  /* USER CODE END startmpu6500task */
}

/* USER CODE BEGIN Header_startspidmatask */
/**
* @brief Function implementing the central SPI DMA worker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startspidmatask */
void startspidmatask(void *argument)
{
  /* USER CODE BEGIN startspidmatask */
  spi_dispatch_slot_t active_jobs[3] = {0};
  const uint32_t completion_flags[3] = {
    SPI1_DMA_DONE_FLAG,
    SPI2_DMA_DONE_FLAG,
    SPI3_DMA_DONE_FLAG
  };

  spi_manager_set_dispatch_notification(SPI1, spidmataskHandle, SPI1_DMA_DONE_FLAG);
  spi_manager_set_dispatch_notification(SPI3, spidmataskHandle, SPI3_DMA_DONE_FLAG);

  while(osSemaphoreAcquire(spi_semaphoreHandle, 0) == osOK) { }
  while(osSemaphoreAcquire(spi3_semaphoreHandle, 0) == osOK) { }
  osThreadFlagsClear(SPI_DMA_ALL_FLAGS);
  spi_dispatch_ready = true;

  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_SPI_DISPATCH);
    uint32_t pending_flags;
    pending_flags = osThreadFlagsWait(SPI_DMA_ALL_FLAGS, osFlagsWaitAny, 10);
    bool flag_wait_error = (pending_flags & osFlagsError) != 0;
    if(flag_wait_error) pending_flags = 0;
    uint32_t now = osKernelGetTickCount();

    for(uint8_t index = 0; index < 3; index++){
      bool transfer_finished = (pending_flags & completion_flags[index]) != 0;
      if(transfer_finished && active_jobs[index].active){
        bool transfer_succeeded;
        transfer_succeeded = spi_manager_last_transfer_succeeded(
                active_jobs[index].job.spi_handle);
        spi_manager_unlock_bus(active_jobs[index].job.spi_handle);

        uint32_t result_flag = active_jobs[index].job.error_flag;
        if(transfer_succeeded){
          result_flag = active_jobs[index].job.notify_flag;
        }

        if(active_jobs[index].job.notify_task != NULL && result_flag != 0){
          osThreadFlagsSet(active_jobs[index].job.notify_task, result_flag);
        }
        active_jobs[index].active = false;
        refresh_spi_dma_debug(active_jobs);
      }

      if(active_jobs[index].active){
        uint32_t elapsed_time = now - active_jobs[index].start_tick;
        if(elapsed_time >= DMA_DISPATCH_TIMEOUT_MS){
          spi_manager_abort_transfer(active_jobs[index].job.spi_handle);
          if(active_jobs[index].job.notify_task != NULL &&
             active_jobs[index].job.error_flag != 0){
            osThreadFlagsSet(active_jobs[index].job.notify_task,
                             active_jobs[index].job.error_flag);
          }
          active_jobs[index].active = false;
          refresh_spi_dma_debug(active_jobs);
        }
      }
    }

    uint32_t queued_job_count = osMessageQueueGetCount(spi_job_queueHandle);
    for(uint32_t queue_index = 0; queue_index < queued_job_count; queue_index++){
      spi_job_t job;
      osStatus_t queue_status;
      queue_status = osMessageQueueGet(spi_job_queueHandle, &job, NULL, 0);
      if(queue_status != osOK) break;

      int8_t dispatch_index = get_spi_dispatch_index(job.spi_handle);
      if(dispatch_index < 0){
        if(job.notify_task != NULL && job.error_flag != 0){
          osThreadFlagsSet(job.notify_task, job.error_flag);
        }
        continue;
      }

      bool dma_resource_busy = false;
      for(uint8_t active_index = 0; active_index < 3; active_index++){
        if(active_jobs[active_index].active == false) continue;
        bool same_dma = active_jobs[active_index].job.dma_handle == job.dma_handle;
        bool rx_stream_conflict =
                active_jobs[active_index].job.rx_stream == job.rx_stream ||
                active_jobs[active_index].job.tx_stream == job.rx_stream;
        bool tx_stream_conflict =
                active_jobs[active_index].job.rx_stream == job.tx_stream ||
                active_jobs[active_index].job.tx_stream == job.tx_stream;
        if(same_dma && (rx_stream_conflict || tx_stream_conflict)){
          dma_resource_busy = true;
          break;
        }
      }

      if(active_jobs[dispatch_index].active || dma_resource_busy){
        osMessageQueuePut(spi_job_queueHandle, &job, 0, 0);
        continue;
      }

      active_jobs[dispatch_index].active = true;
      active_jobs[dispatch_index].start_tick = osKernelGetTickCount();
      active_jobs[dispatch_index].job = job;

      spi_manager_return_status transfer_status;
      transfer_status = spi_manager_transfer_dma(job.spi_handle, job.dma_handle,
                                                 job.rx_stream, job.tx_stream,
                                                 job.cs_port, job.cs_pin,
                                                 job.txdata, job.rxdata, job.size);
      if(transfer_status == _spi_manager_ok){
        refresh_spi_dma_debug(active_jobs);
        continue;
      }

      active_jobs[dispatch_index].active = false;
      if(transfer_status == _spi_manager_busy){
        osMessageQueuePut(spi_job_queueHandle, &job, 0, 0);
      }
      else if(job.notify_task != NULL && job.error_flag != 0){
        osThreadFlagsSet(job.notify_task, job.error_flag);
      }
    }
  }
  /* USER CODE END startspidmatask */
}

/* USER CODE BEGIN Header_startnrf24l01TXtask */
/**
* @brief Function implementing the nrf24l01TXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startnrf24l01TXtask */
void startnrf24l01TXtask(void *argument)
{
  /* USER CODE BEGIN startnrf24l01TXtask */
  (void)argument;

  /* F407'de tek NRF vardir. Fiziksel radyoyu RX taski yonetir; bu task
     yalnizca gonderilecek payload'i radyo-sahibi taska teslim eder. */
  while(!nrf_rx_init_completed || my_nrf_rx == NULL){
    system_health_heartbeat(HEALTH_TASK_NRF_TX);
    osDelay(100);
  }
  my_nrf_tx = my_nrf_rx;
  nrf_tx_init_completed = true;
  nrf_tx_last_success_tick = osKernelGetTickCount();

  static const uint8_t tx_high[] = "HIGH";
  static const uint8_t tx_low[] = "LOW";
  uint32_t next_transmit_tick =
          ((osKernelGetTickCount() / COMMUNICATION_PERIOD_MS) + 1) *
          COMMUNICATION_PERIOD_MS;

  /* Infinite loop */
  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_NRF_TX);
    osDelayUntil(next_transmit_tick);
    next_transmit_tick += COMMUNICATION_PERIOD_MS;
    uint32_t phase = (osKernelGetTickCount() / COMMUNICATION_PERIOD_MS) & 1;
    const uint8_t* tx_message;
    uint8_t tx_message_length;
    /* F407 faz 0: CAN HIGH, NRF LOW. Faz 1: CAN LOW, NRF HIGH. */
    if(phase == 0){
      tx_message = tx_low;
      tx_message_length = sizeof(tx_low) - 1;
    }
    else{
      tx_message = tx_high;
      tx_message_length = sizeof(tx_high) - 1;
    }

    if(osMutexAcquire(nrf_mode_mutexHandle, 20) == osOK){
      if(!nrf_tx_request_pending){
        memcpy(nrf_pending_tx, tx_message, tx_message_length);
        nrf_pending_tx_length = tx_message_length;
        nrf_tx_request_pending = true;
        osThreadFlagsSet(nrf24l01RXtaskHandle, NRF_RADIO_TX_REQUEST_FLAG);

      }
      osMutexRelease(nrf_mode_mutexHandle);
    }

  }
  /* USER CODE END startnrf24l01TXtask */
}

/* USER CODE BEGIN Header_startnrf24l01RXtask */
/**
* @brief Function implementing the nrf24l01RXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startnrf24l01RXtask */
void startnrf24l01RXtask(void *argument)
{
  /* USER CODE BEGIN startnrf24l01RXtask */
  (void)argument;
  nrf24l01_user_configs rx_cfg = {0};
  rx_cfg.spi_handle = SPI1;
  rx_cfg.dma_handle = DMA2;
  rx_cfg.rx_stream = LL_DMA_STREAM_0;
  rx_cfg.tx_stream = LL_DMA_STREAM_3;
  rx_cfg.csn_port = nrf24_csn_pin_spi1_GPIO_Port;
  rx_cfg.csn_pin = nrf24_csn_pin_spi1_Pin;
  rx_cfg.ce_port = nrf24_ce_pin_spi1_GPIO_Port;
  rx_cfg.ce_pin = nrf24_ce_pin_spi1_Pin;
  rx_cfg.spi_semaphore = spi_semaphoreHandle;

  while(my_nrf_rx == NULL){
    system_health_heartbeat(HEALTH_TASK_NRF_RX);
    my_nrf_rx = nrf24l01_init(&rx_cfg);
    if(my_nrf_rx == NULL) osDelay(1000);
  }

  (void)spi_manager_register_validator(SPI1, validate_nrf24_spi, my_nrf_rx);

  my_nrf_tx = my_nrf_rx;
  nrf24l01_assign_interrupt_task(my_nrf_rx, nrf24l01RXtaskHandle, NRF_RADIO_IRQ_FLAG);
  osThreadFlagsClear(NRF_RADIO_IRQ_FLAG | NRF_RADIO_DMA_SUCCESS_FLAG |
                     NRF_RADIO_DMA_ERROR_FLAG | NRF_RADIO_TX_REQUEST_FLAG);
  nrf24l01_clear_interrupts(my_nrf_rx);
  nrf24l01_start_listening(my_nrf_rx);
  nrf_rx_init_completed = true;
  nrf_rx_last_success_tick = osKernelGetTickCount();
  uint32_t nrf_link_start_tick = osKernelGetTickCount();
  uint32_t nrf_reinit_last_attempt_tick = 0;

  /* Infinite loop */
  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_NRF_RX);
    uint32_t now = osKernelGetTickCount();
    uint32_t rx_reference_tick = g_communication_link_debug.nrf_rx_last_tick;
    if(rx_reference_tick == 0) rx_reference_tick = nrf_link_start_tick;
    g_communication_link_debug.nrf_rx_timeout =
            (now - rx_reference_tick) >= COMMUNICATION_RX_TIMEOUT_MS;
    if(g_communication_link_debug.nrf_rx_timeout){
      /* Son alinan HIGH/LOW artik guncel degildir; kopuk hatta LED eski
         durumunu koruyarak yaniltici bir baglanti goruntusu vermesin. */
      LL_GPIO_ResetOutputPin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin);
    }

    if(g_communication_link_debug.nrf_rx_timeout &&
       (now - nrf_reinit_last_attempt_tick) >= NRF_REINIT_RETRY_MS){
      nrf_reinit_last_attempt_tick = now;
      g_communication_link_debug.nrf_reinit_attempt_count++;
      nrf24l01_return_status reinit_status =
              nrf24l01_reinitialize(my_nrf_rx);
      if(reinit_status == _nrf24l01_ok){
        g_communication_link_debug.nrf_reinit_success_count++;
      }
      else{
        g_communication_link_debug.nrf_reinit_failure_count++;
      }
    }

    uint32_t flags = osThreadFlagsWait(NRF_RADIO_IRQ_FLAG | NRF_RADIO_TX_REQUEST_FLAG,
                                       osFlagsWaitAny, 100);
    bool flag_wait_error = (flags & osFlagsError) != 0;
    if(flag_wait_error) continue;

    if((flags & NRF_RADIO_IRQ_FLAG) != 0){
      nrf24l01_irq_status_t irq_status = {0};
      nrf24l01_return_status nrf_status;
      nrf_status = nrf24l01_get_irq_status(my_nrf_rx, &irq_status);

      if(nrf_status == _nrf24l01_ok && irq_status.rx_dr){
        uint8_t packets_read_this_interrupt = 0;
        nrf24l01_fifo_status_t fifo_status;
        nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);

        if(nrf_status == _nrf24l01_ok){
          nrf_rx_fifo_was_full = fifo_status.rx_full;

          while(!fifo_status.rx_empty &&
                packets_read_this_interrupt < NRF24L01_FIFO_DEPTH)
          {
            spi_job_t job;
            uint8_t payload_length = 0;
            nrf_status = nrf24l01_build_rx_fifo_job(my_nrf_rx,
                        nrf24l01RXtaskHandle, NRF_RADIO_DMA_SUCCESS_FLAG,
                        NRF_RADIO_DMA_ERROR_FLAG, &payload_length, &job);
            if(nrf_status != _nrf24l01_ok) break;
            if(submit_spi_dma_job(&job) != osOK) break;

            uint32_t dma_flags = osThreadFlagsWait(
                        NRF_RADIO_DMA_SUCCESS_FLAG | NRF_RADIO_DMA_ERROR_FLAG,
                        osFlagsWaitAny, 500);
            if((dma_flags & osFlagsError) != 0 ||
               (dma_flags & NRF_RADIO_DMA_SUCCESS_FLAG) == 0) break;

            uint8_t storage_index = nrf_rx_debug_write_index;
            memset(nrf_rx_fifo[storage_index], 0, NRF24L01_PAYLOAD_SIZE);
            nrf_status = nrf24l01_finish_rx_fifo_job(my_nrf_rx,
                        nrf_rx_fifo[storage_index], payload_length);
            if(nrf_status != _nrf24l01_ok) break;

            nrf_rx_payload_lengths[storage_index] = payload_length;
            if(osMutexAcquire(sensor_data_mutexHandle, 10) == osOK){
              copy_display_text(nrf24l01_display_data.rx_text,
                      sizeof(nrf24l01_display_data.rx_text),
                      nrf_rx_fifo[storage_index], payload_length);
              nrf24l01_display_data.rx_length = payload_length;
              nrf24l01_display_data.rx_valid = true;
              nrf24l01_display_data.rx_update_count++;
              osMutexRelease(sensor_data_mutexHandle);
            }

            bool valid_level_message = false;
            if(payload_length == 4 &&
               memcmp(nrf_rx_fifo[storage_index], "HIGH", 4) == 0){
              LL_GPIO_SetOutputPin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin);
              valid_level_message = true;
            }
            else if(payload_length == 3 &&
                    memcmp(nrf_rx_fifo[storage_index], "LOW", 3) == 0){
              LL_GPIO_ResetOutputPin(NRF_LINK_LED_GPIO_Port, NRF_LINK_LED_Pin);
              valid_level_message = true;
            }
            if(valid_level_message){
              uint32_t receive_tick = osKernelGetTickCount();
              g_communication_link_debug.nrf_rx_last_tick = receive_tick;
              g_communication_link_debug.nrf_rx_timeout = false;
            }

            nrf_rx_debug_write_index++;
            if(nrf_rx_debug_write_index >= NRF24L01_FIFO_DEPTH){
              nrf_rx_debug_write_index = 0;
            }
            if(nrf_rx_fifo_count < NRF24L01_FIFO_DEPTH) nrf_rx_fifo_count++;

            packets_read_this_interrupt++;
            nrf_rx_packet_count++;
            nrf_rx_last_success_tick = osKernelGetTickCount();
            sayac++;

            nrf24l01_clear_irq_sources(my_nrf_rx, NRF24L01_IRQ_RX_DR);
            nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);
            if(nrf_status != _nrf24l01_ok) break;
          }

          nrf_status = nrf24l01_get_fifo_status(my_nrf_rx, &fifo_status);
          if(nrf_status == _nrf24l01_ok && !fifo_status.rx_empty){
            osThreadFlagsSet(nrf24l01RXtaskHandle, NRF_RADIO_IRQ_FLAG);
          }
        }
      }
      else if(nrf_status == _nrf24l01_ok){
        nrf24l01_clear_irq_sources(my_nrf_rx, irq_status.raw);
      }
    }

    if((flags & NRF_RADIO_TX_REQUEST_FLAG) != 0){
      uint8_t tx_payload[NRF24L01_PAYLOAD_SIZE] = {0};
      uint8_t tx_length = 0;

      if(osMutexAcquire(nrf_mode_mutexHandle, 20) == osOK){
        if(nrf_tx_request_pending){
          tx_length = nrf_pending_tx_length;
          memcpy(tx_payload, nrf_pending_tx, tx_length);
          nrf_tx_request_pending = false;
        }
        osMutexRelease(nrf_mode_mutexHandle);
      }

      if(tx_length > 0){
        bool tx_success = false;
        spi_job_t job;
        nrf24l01_return_status nrf_status;

        nrf24l01_stop_listening(my_nrf_rx);
        nrf_status = nrf24l01_build_tx_job(my_nrf_rx, tx_payload, tx_length,
                    nrf24l01RXtaskHandle, NRF_RADIO_DMA_SUCCESS_FLAG,
                    NRF_RADIO_DMA_ERROR_FLAG, &job);

        if(nrf_status == _nrf24l01_ok && submit_spi_dma_job(&job) == osOK){
          uint32_t dma_flags = osThreadFlagsWait(
                      NRF_RADIO_DMA_SUCCESS_FLAG | NRF_RADIO_DMA_ERROR_FLAG,
                      osFlagsWaitAny, 500);

          if((dma_flags & osFlagsError) == 0 &&
             (dma_flags & NRF_RADIO_DMA_SUCCESS_FLAG) != 0 &&
             nrf24l01_trigger_transmission(my_nrf_rx) == _nrf24l01_ok){
            uint32_t irq_flags = osThreadFlagsWait(NRF_RADIO_IRQ_FLAG,
                                                   osFlagsWaitAny, 200);
            if((irq_flags & osFlagsError) == 0 &&
               (irq_flags & NRF_RADIO_IRQ_FLAG) != 0){
              nrf24l01_irq_status_t irq_status = {0};
              if(nrf24l01_get_irq_status(my_nrf_rx, &irq_status) ==
                 _nrf24l01_ok){
                nrf24l01_clear_irq_sources(my_nrf_rx, irq_status.raw);
                tx_success = irq_status.tx_ds;
                if(irq_status.max_rt) flush_tx(my_nrf_rx);
              }
            }
          }
        }

        if(tx_success){
          nrf_tx_success_count++;
          nrf_tx_last_success_tick = osKernelGetTickCount();
          if(osMutexAcquire(sensor_data_mutexHandle, 10) == osOK){
            copy_display_text(nrf24l01_display_data.tx_text,
                    sizeof(nrf24l01_display_data.tx_text),
                    tx_payload, tx_length);
            nrf24l01_display_data.tx_length = tx_length;
            nrf24l01_display_data.tx_valid = true;
            nrf24l01_display_data.tx_update_count++;
            osMutexRelease(sensor_data_mutexHandle);
          }
        }
        else{
          flush_tx(my_nrf_rx);
          nrf24l01_clear_interrupts(my_nrf_rx);
          nrf_tx_error_count++;
        }

        nrf24l01_start_listening(my_nrf_rx);
      }
    }
  }
  /* USER CODE END startnrf24l01RXtask */
}

/* USER CODE BEGIN Header_startili9341task */
/**
* @brief Function implementing the ili9341task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startili9341task */
void startili9341task(void *argument)
{
  /* USER CODE BEGIN startili9341task */
  ili9341_user_configs tft1_cfg = {0};
  tft1_cfg.spi_handle = SPI3;
  tft1_cfg.dma_handle = DMA1;
  tft1_cfg.dma_tx_stream = LL_DMA_STREAM_5;
  tft1_cfg.dma_rx_stream = LL_DMA_STREAM_2;
  tft1_cfg.cs_port = TFT1_CS_GPIO_Port;
  tft1_cfg.cs_pin = TFT1_CS_Pin;
  tft1_cfg.dc_port = TFT1_DC_GPIO_Port;
  tft1_cfg.dc_pin = TFT1_DC_Pin;
  tft1_cfg.rst_port = TFT1_RST_GPIO_Port;
  tft1_cfg.rst_pin = TFT1_RST_Pin;
  tft1_cfg.rotation = ILI9341_ROTATION_270;

  ili9341_user_configs tft2_cfg = {0};
  tft2_cfg.spi_handle = SPI3;
  tft2_cfg.dma_handle = DMA1;
  tft2_cfg.dma_tx_stream = LL_DMA_STREAM_5;
  tft2_cfg.dma_rx_stream = LL_DMA_STREAM_2;
  tft2_cfg.cs_port = TFT2_CS_GPIO_Port;
  tft2_cfg.cs_pin = TFT2_CS_Pin;
  tft2_cfg.dc_port = TFT2_DC_GPIO_Port;
  tft2_cfg.dc_pin = TFT2_DC_Pin;
  tft2_cfg.rst_port = TFT2_RST_GPIO_Port;
  tft2_cfg.rst_pin = TFT2_RST_Pin;
  tft2_cfg.rotation = ILI9341_ROTATION_270;

  while(my_tft1 == NULL || my_tft2 == NULL){
    system_health_heartbeat(HEALTH_TASK_DISPLAY);
    if(my_tft1 == NULL) my_tft1 = ili9341_init(&tft1_cfg);
    if(my_tft2 == NULL) my_tft2 = ili9341_init(&tft2_cfg);
    if(my_tft1 == NULL || my_tft2 == NULL){
      ili9341_last_status = _ili9341_fail;
      ili9341_test_passed = false;
      osDelay(1000);
    }
  }
  ili9341_init_completed = true;
  ili9341_last_success_tick = osKernelGetTickCount();

  ili9341_last_status = ili9341_begin_fill_rect(my_tft1, 0, 0,
          ili9341_get_width(my_tft1), ili9341_get_height(my_tft1),
          ILI9341_COLOR_BLACK);
  if(ili9341_last_status == _ili9341_ok){
    ili9341_last_status = run_ili9341_dma_operation(my_tft1);
  }

  if(ili9341_last_status == _ili9341_ok){
    ili9341_last_status = ili9341_begin_fill_rect(my_tft2, 0, 0,
            ili9341_get_width(my_tft2), ili9341_get_height(my_tft2),
            ILI9341_COLOR_BLACK);
  }
  if(ili9341_last_status == _ili9341_ok){
    ili9341_last_status = run_ili9341_dma_operation(my_tft2);
  }

  if(ili9341_last_status == _ili9341_ok){
    ili9341_last_status = draw_sensor_display_layout();
  }

  ili9341_test_passed = ili9341_last_status == _ili9341_ok;
  uint32_t last_bme280_update_count = 0;
  uint32_t last_mpu6500_update_count = 0;
  uint32_t last_adxl345_update_count = 0;
  uint32_t last_rc522_update_count = 0;
  uint32_t next_sensor_display_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_DISPLAY);
    // DMA aktarimlari zaten taski uyutarak bekletiyor. Bu kisa gecikme,
    // yeni snapshot yokken ekran taskinin bosuna donmesini engelliyor.
    osDelay(10);

    bme280_display_data_t bme_snapshot = {0};
    mpu6500_display_data_t mpu_snapshot = {0};
    adxl345_display_data_t adxl_snapshot = {0};
    rc522_display_data_t rc522_snapshot = {0};
    osStatus_t mutex_status;
    mutex_status = osMutexAcquire(sensor_data_mutexHandle, 20);
    if(mutex_status == osOK){
      bme_snapshot = bme280_display_data;
      mpu_snapshot = mpu6500_display_data;
      adxl_snapshot = adxl345_display_data;
      rc522_snapshot = rc522_display_data;
      osMutexRelease(sensor_data_mutexHandle);
    }

    uint32_t now = osKernelGetTickCount();
    bool sensor_refresh_due = (int32_t)(now - next_sensor_display_tick) >= 0;
    if(sensor_refresh_due){
      next_sensor_display_tick = now + 100;
    }

    if(sensor_refresh_due && bme_snapshot.valid &&
       bme_snapshot.update_count != last_bme280_update_count){
      ili9341_return_status display_status;
      display_status = refresh_bme280_display(&bme_snapshot);
      if(display_status == _ili9341_ok){
        last_bme280_update_count = bme_snapshot.update_count;
      }
      else{
        ili9341_last_status = display_status;
      }
    }

    if(sensor_refresh_due && mpu_snapshot.valid &&
       mpu_snapshot.update_count != last_mpu6500_update_count){
      ili9341_return_status display_status;
      display_status = refresh_mpu6500_display(&mpu_snapshot);
      if(display_status == _ili9341_ok){
        last_mpu6500_update_count = mpu_snapshot.update_count;
      }
      else{
        ili9341_last_status = display_status;
      }
    }

    if(sensor_refresh_due && adxl_snapshot.valid &&
       adxl_snapshot.update_count != last_adxl345_update_count){
      ili9341_return_status display_status;
      display_status = refresh_adxl345_display(&adxl_snapshot);
      if(display_status == _ili9341_ok){
        last_adxl345_update_count = adxl_snapshot.update_count;
      }
      else{
        ili9341_last_status = display_status;
      }
    }

    if(rc522_snapshot.update_count != last_rc522_update_count){
      ili9341_return_status display_status;
      display_status = refresh_rc522_display(&rc522_snapshot);
      if(display_status == _ili9341_ok){
        last_rc522_update_count = rc522_snapshot.update_count;
      }
      else{
        ili9341_last_status = display_status;
      }
    }

    ili9341_test_passed = ili9341_last_status == _ili9341_ok;
  }
  /* USER CODE END startili9341task */
}

/* USER CODE BEGIN Header_startadxl345task */
/* Periodic ADXL345 I2C2 RX-DMA task. */
/* USER CODE END Header_startadxl345task */
void startadxl345task(void *argument)
{
  /* USER CODE BEGIN startadxl345task */
  (void)argument;
  uint32_t ready = osThreadFlagsWait(I2C_WORKER_READY_FLAG,
          osFlagsWaitAny, osWaitForever);
  if((ready & I2C_WORKER_READY_FLAG) == 0) for(;;) osDelay(1000);

  adxl345_config_t config = {
    .i2c_handle = I2C2,
    .dma_handle = DMA1,
    .dma_stream = LL_DMA_STREAM_3,
    .i2c_address = ADXL345_I2C_ADDRESS_ALT_LOW
  };
  while(my_adxl345 == NULL){
    system_health_heartbeat(HEALTH_TASK_ADXL345);
    if(acquire_i2c_init_lock()){
      config.i2c_address = ADXL345_I2C_ADDRESS_ALT_LOW;
      my_adxl345 = adxl345_init(&config);
      if(my_adxl345 == NULL){
        config.i2c_address = ADXL345_I2C_ADDRESS_ALT_HIGH;
        my_adxl345 = adxl345_init(&config);
      }
      release_i2c_init_lock();
    }
    if(my_adxl345 == NULL) osDelay(1000);
  }
  adxl345_init_completed = true;
  adxl345_last_success_tick = osKernelGetTickCount();
  register_i2c_sensor_client_ready(I2C2);

  /* I2C2 dispatcher zaten ayni hattaki islemleri siralastirir. Bir sensorun
   * init hatasi, basariyla init edilen diger sensoru ortak bariyerde
   * sonsuza kadar bekletmemelidir. */

  uint32_t next_wake = osKernelGetTickCount();
  for(;;){
    system_health_heartbeat(HEALTH_TASK_ADXL345);
    next_wake += 100;
    osThreadFlagsClear(ADXL345_DMA_SUCCESS_FLAG | ADXL345_DMA_ERROR_FLAG);
    i2c_job_t job;
    if(adxl345_build_read_job(my_adxl345, adxl345taskHandle,
            ADXL345_DMA_SUCCESS_FLAG, ADXL345_DMA_ERROR_FLAG,
            &job) != ADXL345_OK || submit_i2c_dma_job(&job) != osOK){
      adxl345_dma_error_count++;
      osDelayUntil(next_wake);
      continue;
    }
    uint32_t flags = osThreadFlagsWait(ADXL345_DMA_SUCCESS_FLAG |
            ADXL345_DMA_ERROR_FLAG, osFlagsWaitAny, 1000);
    if((flags & osFlagsError) != 0 ||
            (flags & ADXL345_DMA_SUCCESS_FLAG) == 0){
      adxl345_dma_error_count++;
      system_health_bus_record_failure(SYSTEM_BUS_I2C2,
              _i2c_manager_dma_error);
    }
    else{
      adxl345_data_t sample;
      if(adxl345_process_data(my_adxl345, &sample) == ADXL345_OK){
        if(osMutexAcquire(sensor_data_mutexHandle, 20) == osOK){
          adxl345_display_data.x_g = sample.x_g;
          adxl345_display_data.y_g = sample.y_g;
          adxl345_display_data.z_g = sample.z_g;
          adxl345_display_data.valid = true;
          adxl345_display_data.update_count++;
          osMutexRelease(sensor_data_mutexHandle);
        }
        adxl345_dma_success_count++;
        adxl345_last_success_tick = osKernelGetTickCount();
        system_health_bus_record_success(SYSTEM_BUS_I2C2,
                SYSTEM_SOURCE_ADXL345, SYSTEM_OPERATION_READ,
                config.i2c_address, 0x32);
      }
    }
    osDelayUntil(next_wake);
  }
  /* USER CODE END startadxl345task */
}

/* USER CODE BEGIN Header_startssd1306spitask */
/* SPI3 SSD1306 framebuffer DMA task. */
/* USER CODE END Header_startssd1306spitask */
void startssd1306spitask(void *argument)
{
  /* USER CODE BEGIN startssd1306spitask */
  (void)argument;
  while(!spi_dispatch_ready) osDelay(10);
  ssd1306_spi_config_t config = {
    .spi_handle = SPI3, .dma_handle = DMA1,
    .rx_stream = LL_DMA_STREAM_2, .tx_stream = LL_DMA_STREAM_5,
    .cs_port = SSD1306_SPI_CS_GPIO_Port, .cs_pin = SSD1306_SPI_CS_Pin,
    .dc_port = SSD1306_SPI_DC_GPIO_Port, .dc_pin = SSD1306_SPI_DC_Pin,
    .reset_port = SSD1306_SPI_RST_GPIO_Port, .reset_pin = SSD1306_SPI_RST_Pin
  };
  while(my_ssd1306_spi == NULL){
    system_health_heartbeat(HEALTH_TASK_SSD1306_SPI);
    my_ssd1306_spi = ssd1306_spi_init(&config);
    if(my_ssd1306_spi == NULL) osDelay(1000);
  }
  ssd1306_spi_init_completed = true;
  ssd1306_spi_last_success_tick = osKernelGetTickCount();

  uint32_t last_nrf_tx_count = UINT32_MAX;
  uint32_t last_nrf_rx_count = UINT32_MAX;
  uint32_t last_can_tx_count = UINT32_MAX;
  uint32_t last_can_rx_count = UINT32_MAX;
  bool last_can_timeout = false;
  bool last_nrf_timeout = false;
  for(;;){
    system_health_heartbeat(HEALTH_TASK_SSD1306_SPI);
    nrf24l01_display_data_t nrf = {0};
    can_display_data_t can = {0};
    if(osMutexAcquire(sensor_data_mutexHandle, 20) == osOK){
      nrf = nrf24l01_display_data;
      can = can_display_data;
      osMutexRelease(sensor_data_mutexHandle);
    }
    uint32_t can_tx_count = can.tx_update_count;
    uint32_t can_rx_count = can.rx_update_count;
    bool can_timeout = g_communication_link_debug.can_rx_timeout;
    bool nrf_timeout = g_communication_link_debug.nrf_rx_timeout;
    if(nrf.tx_update_count != last_nrf_tx_count ||
       nrf.rx_update_count != last_nrf_rx_count ||
       can_tx_count != last_can_tx_count || can_rx_count != last_can_rx_count ||
       can_timeout != last_can_timeout || nrf_timeout != last_nrf_timeout){
      char line[22];
      const char* can_tx_text = "WAIT";
      const char* can_rx_text = "WAIT";
      const char* nrf_tx_text = "WAIT";
      const char* nrf_rx_text = "WAIT";
      if(can_timeout){
        /* TX isteginin mailbox'a alinmasi, fiziksel hattan ACK geldigi
           anlamina gelmez. RX akisi kesildiyse iki CAN satiri da offline. */
        can_tx_text = "VERI GELMEDI";
        can_rx_text = "VERI GELMEDI";
      }
      else{
        if(can.tx_update_count != 0) can_tx_text = can.tx_text;
        if(can.rx_update_count != 0) can_rx_text = can.rx_text;
      }
      if(nrf_timeout){
        /* Kopukken son basarili TX metnini gostermek de eski veri olur. */
        nrf_tx_text = "VERI GELMEDI";
        nrf_rx_text = "VERI GELMEDI";
      }
      else{
        if(nrf.tx_valid) nrf_tx_text = nrf.tx_text;
        if(nrf.rx_valid) nrf_rx_text = nrf.rx_text;
      }
      ssd1306_spi_clear(my_ssd1306_spi);
      snprintf(line, sizeof(line), "CAN>%.12s",
              can_tx_text);
      ssd1306_spi_draw_text(my_ssd1306_spi, 0, 0, line);
      snprintf(line, sizeof(line), "CAN<%.12s",
              can_rx_text);
      ssd1306_spi_draw_text(my_ssd1306_spi, 0, 4, line);

      snprintf(line, sizeof(line), "NRF>%.12s",
              nrf_tx_text);
      ssd1306_spi_draw_text(my_ssd1306_spi, 0, 8, line);
      snprintf(line, sizeof(line), "NRF<%.12s",
              nrf_rx_text);
      ssd1306_spi_draw_text(my_ssd1306_spi, 0, 12, line);

      osThreadFlagsClear(SSD1306_SPI_SUCCESS_FLAG | SSD1306_SPI_ERROR_FLAG);
      spi_job_t job;
      if(ssd1306_spi_build_refresh_job(my_ssd1306_spi,
              ssd1306spitaskHandle, SSD1306_SPI_SUCCESS_FLAG,
              SSD1306_SPI_ERROR_FLAG, &job) &&
              submit_spi_dma_job(&job) == osOK){
        uint32_t flags = osThreadFlagsWait(SSD1306_SPI_SUCCESS_FLAG |
                SSD1306_SPI_ERROR_FLAG, osFlagsWaitAny, 1500);
        if((flags & osFlagsError) == 0 &&
                (flags & SSD1306_SPI_SUCCESS_FLAG) != 0){
          ssd1306_spi_refresh_count++;
          ssd1306_spi_last_success_tick = osKernelGetTickCount();
          last_nrf_tx_count = nrf.tx_update_count;
          last_nrf_rx_count = nrf.rx_update_count;
          last_can_tx_count = can_tx_count;
          last_can_rx_count = can_rx_count;
          last_can_timeout = can_timeout;
          last_nrf_timeout = nrf_timeout;
          system_health_bus_record_success(SYSTEM_BUS_SPI3,
                  SYSTEM_SOURCE_SSD1306_SPI, SYSTEM_OPERATION_WRITE,
                  0, SSD1306_BUFFER_SIZE);
        }
      }
    }
    osDelay(100);
  }
  /* USER CODE END startssd1306spitask */
}

static bool recover_supervised_bus(system_health_bus_id_t bus){
  if(bus == SYSTEM_BUS_I2C1){
    /* Aktif DMA/polling transferini health tasktan zorla bozma. Manager mutexi
     * recovery ile dispatcher'i siraya sokar; mesgulse sonraki turda denenir. */
    return i2c_manager_recover_bus(I2C1) == _i2c_manager_ok;
  }
  if(bus == SYSTEM_BUS_I2C2){
    return i2c_manager_recover_bus(I2C2) == _i2c_manager_ok;
  }
  if(bus == SYSTEM_BUS_SPI1){
    return spi_manager_recover_bus(SPI1) == _spi_manager_ok;
  }
  if(bus == SYSTEM_BUS_SPI2){
    return spi_manager_recover_bus(SPI2) == _spi_manager_ok;
  }
  if(bus == SYSTEM_BUS_SPI3){
    return spi_manager_recover_bus(SPI3) == _spi_manager_ok;
  }
  if(bus == SYSTEM_BUS_CAN2){
    return can_manager_recover(CAN2) == can_manager_ok;
  }
  return false;
}

static void supervise_faulted_buses(uint32_t now){
  for(uint32_t index = 0; index < SYSTEM_BUS_COUNT; index++){
    volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[index];
    if(health->state == SYSTEM_BUS_STATE_OK) continue;

    if((now - health->last_recovery_tick) >= BUS_RECOVERY_RETRY_PERIOD_MS){
      system_health_bus_mark_recovering((system_health_bus_id_t)index);
      bool recovered = recover_supervised_bus((system_health_bus_id_t)index);
      int32_t recovery_status = -1;
      if(recovered) recovery_status = 0;
      system_health_bus_record_recovery((system_health_bus_id_t)index,
              recovered, recovery_status);
    }

    if((health->fault_since_tick != 0) &&
       ((now - health->fault_since_tick) >= BUS_RECOVERY_RESET_TIMEOUT_MS)){
      health->reset_deadline_expired = true;
      if(is_mcu_reset_allowed){
        __DSB();
        NVIC_SystemReset();
      }else{
        health->reset_suppressed = true;
      }
    }
  }
}

/* USER CODE BEGIN Header_starthealthtask */
/**
* @brief Function implementing the healthtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_starthealthtask */
void starthealthtask(void *argument)
{
  /* USER CODE BEGIN starthealthtask */
  (void)argument;

  /* Donanimsal watchdog hemen baslatilir. Baslangic izni watchdog'u kapali
   * tutmaz; yalnizca cevre birimleri kurulurken sinirli sure beslenmesini
   * saglar. Boylece her resetten sonra korunmasiz 15 saniyelik pencere kalmaz. */
  const uint32_t monitor_period_ms = 1000;
  uint32_t startup_tick = osKernelGetTickCount();
  uint32_t next_check = osKernelGetTickCount();
  bool watchdog_ready = system_health_watchdog_start();
  uint32_t observed_bme_tick = 0;
  uint32_t observed_mpu_tick = 0;
  uint32_t observed_rc522_tick = 0;
#if WATCHDOG_MONITOR_NRF
  uint32_t observed_nrf_tx_tick = 0;
  uint32_t observed_nrf_rx_tick = 0;
#endif
  uint32_t observed_display_tick = 0;
#if CAN2_RUNTIME_ENABLED && WATCHDOG_MONITOR_CAN
  uint32_t observed_can_tx_count = 0;
  uint32_t observed_can_rx_count = 0;
  uint32_t observed_can_error_count = 0;
#endif

  for(;;)
  {
    next_check += monitor_period_ms;
    osDelayUntil(next_check);

    uint32_t now = osKernelGetTickCount();
    bool all_tasks_healthy = system_health_evaluate_window(
            ACTIVE_HEALTH_MASK);

    if(bme280_last_success_tick != observed_bme_tick){
      observed_bme_tick = bme280_last_success_tick;
      system_health_bus_record_success(SYSTEM_BUS_I2C1,
              SYSTEM_SOURCE_BME280, SYSTEM_OPERATION_READ, 0x76, 0);
    }
    if(mpu6500_last_success_tick != observed_mpu_tick){
      observed_mpu_tick = mpu6500_last_success_tick;
      system_health_bus_record_success(SYSTEM_BUS_I2C1,
              SYSTEM_SOURCE_MPU6500, SYSTEM_OPERATION_READ, 0x68, 0);
    }
    if(!I2C_ONLY_TEST &&
       rc522_last_liveness_tick != observed_rc522_tick){
      observed_rc522_tick = rc522_last_liveness_tick;
      system_health_bus_record_success(SYSTEM_BUS_SPI1,
              SYSTEM_SOURCE_RC522, SYSTEM_OPERATION_TRANSFER, 0, 0);
    }
#if WATCHDOG_MONITOR_NRF
    if(!I2C_ONLY_TEST &&
       nrf_tx_last_success_tick != observed_nrf_tx_tick){
      observed_nrf_tx_tick = nrf_tx_last_success_tick;
      system_health_bus_record_success(SYSTEM_BUS_SPI1,
              SYSTEM_SOURCE_NRF24_TX, SYSTEM_OPERATION_WRITE, 0, 0);
    }
    if(!I2C_ONLY_TEST &&
       nrf_rx_last_success_tick != observed_nrf_rx_tick){
      observed_nrf_rx_tick = nrf_rx_last_success_tick;
      system_health_bus_record_success(SYSTEM_BUS_SPI1,
              SYSTEM_SOURCE_NRF24_RX, SYSTEM_OPERATION_READ, 0, 0);
    }
#endif
    if(!I2C_ONLY_TEST &&
       ili9341_last_success_tick != observed_display_tick){
      observed_display_tick = ili9341_last_success_tick;
      system_health_bus_record_success(SYSTEM_BUS_SPI3,
              SYSTEM_SOURCE_ILI9341, SYSTEM_OPERATION_WRITE, 0, 0);
    }
#if CAN2_RUNTIME_ENABLED && WATCHDOG_MONITOR_CAN
    if(g_can2_debug.tx_complete_count != observed_can_tx_count){
      observed_can_tx_count = g_can2_debug.tx_complete_count;
      system_health_bus_record_success(SYSTEM_BUS_CAN2,
              SYSTEM_SOURCE_CAN_TX, SYSTEM_OPERATION_WRITE,
              g_can2_last_tx_frame.id, g_can2_last_tx_frame.dlc);
    }
    if(g_can2_debug.rx_received_count != observed_can_rx_count){
      observed_can_rx_count = g_can2_debug.rx_received_count;
      uint32_t received_id = 0;
      if(g_can2_last_rx_frame_ptr != NULL){
        received_id = g_can2_last_rx_frame_ptr->id;
      }
      system_health_bus_record_success(SYSTEM_BUS_CAN2,
              SYSTEM_SOURCE_CAN_RX, SYSTEM_OPERATION_READ, received_id, 0);
    }
    (void)can_manager_check_ack_timeout(CAN2, CAN_ACK_TIMEOUT_MS);
    uint32_t can_error_count = g_can2_debug.tx_error_count +
            g_can2_debug.error_irq_count + g_can2_debug.bus_off_count +
            g_can2_debug.ack_timeout_count;
    if(can_error_count != observed_can_error_count){
      observed_can_error_count = can_error_count;
      system_health_bus_record_failure(SYSTEM_BUS_CAN2,
              (int32_t)g_can2_debug.last_status);
    }
#endif

    uint32_t stale_mask = 0;
    if(bme280_init_completed &&
       (now - bme280_last_success_tick) >= SENSOR_DATA_STALE_TIMEOUT_MS){
      stale_mask |= SENSOR_STALE_BME280_MASK;
    }
    if(mpu6500_init_completed &&
       (now - mpu6500_last_success_tick) >= SENSOR_DATA_STALE_TIMEOUT_MS){
      stale_mask |= SENSOR_STALE_MPU6500_MASK;
    }
    if(!I2C_ONLY_TEST && rc522_init_completed &&
       (now - rc522_last_liveness_tick) >=
               RC522_LIVENESS_STALE_TIMEOUT_MS){
      stale_mask |= SPI_STALE_RC522_MASK;
    }
#if WATCHDOG_MONITOR_NRF
    if(!I2C_ONLY_TEST && nrf_tx_init_completed &&
       (now - nrf_tx_last_success_tick) >= NRF_PROGRESS_STALE_TIMEOUT_MS){
      stale_mask |= SPI_STALE_NRF_TX_MASK;
    }
    if(!I2C_ONLY_TEST && nrf_rx_init_completed &&
       (now - nrf_rx_last_success_tick) >= NRF_PROGRESS_STALE_TIMEOUT_MS){
      stale_mask |= SPI_STALE_NRF_RX_MASK;
    }
#endif
    if(!I2C_ONLY_TEST && ili9341_init_completed &&
       (now - ili9341_last_success_tick) >=
               DISPLAY_PROGRESS_STALE_TIMEOUT_MS){
      stale_mask |= SPI_STALE_DISPLAY_MASK;
    }
    if(adxl345_init_completed &&
       (now - adxl345_last_success_tick) >= SENSOR_DATA_STALE_TIMEOUT_MS){
      stale_mask |= SENSOR_STALE_ADXL345_MASK;
    }
    if(!I2C_ONLY_TEST && ssd1306_spi_init_completed &&
       (now - ssd1306_spi_last_success_tick) >=
               DISPLAY_PROGRESS_STALE_TIMEOUT_MS){
      stale_mask |= SPI_STALE_SSD1306_MASK;
    }
    sensor_data_stale_mask = stale_mask;
    subsystem_stale_mask = stale_mask;

    bool startup_grace_elapsed =
            (now - startup_tick) >= WATCHDOG_STARTUP_GRACE_MS;
    if(startup_grace_elapsed){
      if((!bme280_init_completed || !mpu6500_init_completed) &&
         (g_system_bus_health[SYSTEM_BUS_I2C1].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_I2C1,
                (int32_t)i2c_manager_last_result(I2C1));
      }

      if(!I2C_ONLY_TEST && !rc522_init_completed &&
         (g_system_bus_health[SYSTEM_BUS_SPI1].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_SPI1,
                _spi_manager_uninited_struct);
      }

      if(!I2C_ONLY_TEST && !ili9341_init_completed &&
         (g_system_bus_health[SYSTEM_BUS_SPI3].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_SPI3,
                _spi_manager_uninited_struct);
      }

#if WATCHDOG_MONITOR_NRF
      if(!I2C_ONLY_TEST &&
         (!nrf_rx_init_completed || !nrf_tx_init_completed) &&
         (g_system_bus_health[SYSTEM_BUS_SPI1].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_SPI1,
                _spi_manager_uninited_struct);
      }
#endif

      if(!adxl345_init_completed &&
         (g_system_bus_health[SYSTEM_BUS_I2C2].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_I2C2,
                (int32_t)i2c_manager_last_result(I2C2));
      }

      if(!I2C_ONLY_TEST && !ssd1306_spi_init_completed &&
         (g_system_bus_health[SYSTEM_BUS_SPI3].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_SPI3,
                _spi_manager_uninited_struct);
      }

#if CAN2_RUNTIME_ENABLED && WATCHDOG_MONITOR_CAN
      if((g_can2_debug.initialized == 0) &&
         (g_system_bus_health[SYSTEM_BUS_CAN2].state == SYSTEM_BUS_STATE_OK)){
        system_health_bus_record_failure(SYSTEM_BUS_CAN2,
                can_manager_not_initialized);
      }
#endif
    }

    if((stale_mask & (SENSOR_STALE_BME280_MASK |
            SENSOR_STALE_MPU6500_MASK)) != 0){
      if(g_system_bus_health[SYSTEM_BUS_I2C1].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_I2C1,
                (int32_t)i2c_manager_last_result(I2C1));
      }
    }
    if((stale_mask & (SPI_STALE_RC522_MASK | SPI_STALE_NRF_RX_MASK)) != 0){
      if(g_system_bus_health[SYSTEM_BUS_SPI1].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_SPI1, _spi_manager_timeout);
      }
    }
    if((stale_mask & SENSOR_STALE_ADXL345_MASK) != 0){
      if(g_system_bus_health[SYSTEM_BUS_I2C2].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_I2C2,
                (int32_t)i2c_manager_last_result(I2C2));
      }
    }
    if((stale_mask & SPI_STALE_DISPLAY_MASK) != 0){
      if(g_system_bus_health[SYSTEM_BUS_SPI3].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_SPI3, _spi_manager_timeout);
      }
    }
    if((stale_mask & SPI_STALE_NRF_TX_MASK) != 0){
      if(g_system_bus_health[SYSTEM_BUS_SPI1].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_SPI1, _spi_manager_timeout);
      }
    }
    if((stale_mask & SPI_STALE_SSD1306_MASK) != 0){
      if(g_system_bus_health[SYSTEM_BUS_SPI3].state == SYSTEM_BUS_STATE_OK){
        system_health_bus_record_failure(SYSTEM_BUS_SPI3, _spi_manager_timeout);
      }
    }

    system_health_bus_refresh_ages(now);
    supervise_faulted_buses(now);
    bool subsystems_initialized = bme280_init_completed &&
            mpu6500_init_completed && adxl345_init_completed;
#if CAN2_RUNTIME_ENABLED && WATCHDOG_MONITOR_CAN
    subsystems_initialized = subsystems_initialized &&
            (g_can2_debug.initialized != 0);
#endif
#if !I2C_ONLY_TEST
    subsystems_initialized = subsystems_initialized &&
            rc522_init_completed && ili9341_init_completed &&
            ssd1306_spi_init_completed;
#endif
#if !I2C_ONLY_TEST && WATCHDOG_MONITOR_NRF
    subsystems_initialized = subsystems_initialized &&
            nrf_tx_init_completed && nrf_rx_init_completed;
#endif
    if(!health_normal_monitoring_started && subsystems_initialized &&
       all_tasks_healthy && stale_mask == 0){
      health_normal_monitoring_started = true;
    }

    health_startup_grace_active = !health_normal_monitoring_started &&
            (now - startup_tick) < WATCHDOG_STARTUP_GRACE_MS;
    bool bus_fault_active = false;
    uint32_t tolerated_missing_heartbeat_mask = 0;
    for(uint32_t index = 0; index < SYSTEM_BUS_COUNT; index++){
#if !WATCHDOG_MONITOR_CAN
      if(index == SYSTEM_BUS_CAN2) continue;
#endif
      if(g_system_bus_health[index].state != SYSTEM_BUS_STATE_OK){
        bus_fault_active = true;

        if(index == SYSTEM_BUS_I2C1){
          tolerated_missing_heartbeat_mask |=
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_BME280) |
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_MPU6500);
        }
        if(index == SYSTEM_BUS_I2C2){
          tolerated_missing_heartbeat_mask |=
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_ADXL345);
        }
        if(index == SYSTEM_BUS_SPI1){
          tolerated_missing_heartbeat_mask |=
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_RC522) |
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_NRF_RX) |
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_NRF_TX);
        }
        if(index == SYSTEM_BUS_SPI3){
          tolerated_missing_heartbeat_mask |=
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_DISPLAY) |
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_SSD1306_SPI);
        }
#if CAN2_RUNTIME_ENABLED
        if(index == SYSTEM_BUS_CAN2){
          tolerated_missing_heartbeat_mask |=
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_CAN_RX) |
                  SYSTEM_HEALTH_TASK_MASK(HEALTH_TASK_CAN_TX);
        }
#endif
      }
    }

    uint32_t unexpected_missing_heartbeat_mask =
            g_system_health.missing_heartbeat_mask &
            ~tolerated_missing_heartbeat_mask;
    bool recovery_window_healthy = bus_fault_active &&
            (unexpected_missing_heartbeat_mask == 0);

    if(health_normal_monitoring_started){
      health_watchdog_feed_allowed = all_tasks_healthy ||
              recovery_window_healthy;
    }else{
      health_watchdog_feed_allowed = health_startup_grace_active ||
              recovery_window_healthy;
    }

    if(!watchdog_ready){
      watchdog_ready = system_health_watchdog_start();
    }
    if(watchdog_ready && health_watchdog_feed_allowed){
      system_health_watchdog_feed();
    }
    refresh_runtime_debug(now, stale_mask);
    /* When a heartbeat is missing, this task deliberately stops feeding the
     * watchdog. The IWDG then resets a system that can no longer self-recover. */
  }
  /* USER CODE END starthealthtask */
}

/* USER CODE BEGIN Header_StartCAN2ReadTask */
/**
* @brief Function implementing the CAN2ReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN2ReadTask */
void StartCAN2ReadTask(void *argument)
{
  /* USER CODE BEGIN StartCAN2ReadTask */
  (void)argument;

  can_manager_status_t init_status;

  do
  {
    init_status = can_manager_init(CAN2, CAN2ReadTaskHandle, CAN2WriteTaskHandle);
    if(init_status != can_manager_ok) osDelay(100);
  }
  while(init_status != can_manager_ok);
  uint32_t can_link_start_tick = osKernelGetTickCount();

  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_CAN_RX);
    uint32_t now = osKernelGetTickCount();
    uint32_t rx_reference_tick = g_communication_link_debug.can_rx_last_tick;
    if(rx_reference_tick == 0) rx_reference_tick = can_link_start_tick;
    g_communication_link_debug.can_rx_timeout =
            (now - rx_reference_tick) >= COMMUNICATION_RX_TIMEOUT_MS;
    if(g_communication_link_debug.can_rx_timeout){
      /* Kopuk hatta son HIGH/LOW seviyesini korumak yerine guvenli ve
         gozle ayirt edilebilir duruma gec. */
      LL_GPIO_ResetOutputPin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin);
    }

    uint32_t flags = osThreadFlagsWait(CAN_MANAGER_RX_EVENT_ANY, osFlagsWaitAny, 100);

    if((flags & osFlagsError) != 0) continue;

    if((flags & CAN_MANAGER_RX_EVENT_ERROR) != 0) can_manager_refresh_debug(CAN2);

    const can_frame_t* received_frame;
    while((received_frame = can_manager_rx_peek(CAN2)) != NULL)
    {
      /* received_frame gercek ring-buffer hucresidir; burada ikinci bir frame kopyasi olusturulmaz. */
        if(received_frame->id == 0x103 &&
           ((received_frame->dlc == 4 &&
             memcmp(received_frame->data, "HIGH", 4) == 0) ||
            (received_frame->dlc == 3 &&
             memcmp(received_frame->data, "LOW", 3) == 0))){
          if(received_frame->dlc == 4){
            LL_GPIO_SetOutputPin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin);
          }
          else{
            LL_GPIO_ResetOutputPin(CAN_LINK_LED_GPIO_Port, CAN_LINK_LED_Pin);
          }
          if(osMutexAcquire(sensor_data_mutexHandle, 10) == osOK){
            copy_display_text(can_display_data.rx_text,
                    sizeof(can_display_data.rx_text), received_frame->data,
                    received_frame->dlc);
            can_display_data.rx_update_count++;
            osMutexRelease(sensor_data_mutexHandle);
          }
          g_communication_link_debug.can_rx_last_tick = osKernelGetTickCount();
          g_communication_link_debug.can_rx_timeout = false;
        }
      (void)can_manager_rx_release(CAN2);
    }
  }
  /* USER CODE END StartCAN2ReadTask */
}

/* USER CODE BEGIN Header_StartCAN2WriteTask */
/**
* @brief Function implementing the CAN2WriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCAN2WriteTask */
void StartCAN2WriteTask(void *argument)
{
  /* USER CODE BEGIN StartCAN2WriteTask */
  (void)argument;

  can_frame_t test_frame = {
    .id = 0x407,
    .dlc = 4,
    .extended_id = false,
    .remote_frame = false,
    .data = {'H', 'I', 'G', 'H', 0, 0, 0, 0},
    .timestamp = 0
  };
  static const uint8_t can_high[] = "HIGH";
  static const uint8_t can_low[] = "LOW";

  (void)osThreadFlagsWait(CAN_MANAGER_TX_EVENT_READY, osFlagsWaitAny, osWaitForever);
  uint32_t next_transmit_tick =
          ((osKernelGetTickCount() / COMMUNICATION_PERIOD_MS) + 1) *
          COMMUNICATION_PERIOD_MS;

  for(;;)
  {
    system_health_heartbeat(HEALTH_TASK_CAN_TX);
    osDelayUntil(next_transmit_tick);
    next_transmit_tick += COMMUNICATION_PERIOD_MS;

    uint32_t phase = (osKernelGetTickCount() / COMMUNICATION_PERIOD_MS) & 1;
    memset(test_frame.data, 0, sizeof(test_frame.data));
    if(phase == 0){
      memcpy(test_frame.data, can_high, sizeof(can_high) - 1);
      test_frame.dlc = sizeof(can_high) - 1;
    }
    else{
      memcpy(test_frame.data, can_low, sizeof(can_low) - 1);
      test_frame.dlc = sizeof(can_low) - 1;
    }
    can_manager_status_t send_status = can_manager_send(CAN2, &test_frame);
    if(send_status == can_manager_no_tx_mailbox)
    {
      (void)osThreadFlagsWait(CAN_MANAGER_TX_EVENT_ANY, osFlagsWaitAny, 100);
      send_status = can_manager_send(CAN2, &test_frame);
    }
    if(send_status == can_manager_ok){
      if(osMutexAcquire(sensor_data_mutexHandle, 10) == osOK){
        copy_display_text(can_display_data.tx_text,
                sizeof(can_display_data.tx_text), test_frame.data,
                test_frame.dlc);
        can_display_data.tx_update_count++;
        osMutexRelease(sensor_data_mutexHandle);
      }
    }
  }
  /* USER CODE END StartCAN2WriteTask */
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
