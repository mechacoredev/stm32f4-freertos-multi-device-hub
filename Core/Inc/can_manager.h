#ifndef INC_CAN_MANAGER_H_
#define INC_CAN_MANAGER_H_

#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

#define CAN_MANAGER_RX_EVENT_FIFO0 0x00000001
#define CAN_MANAGER_RX_EVENT_FIFO1 0x00000002
#define CAN_MANAGER_RX_EVENT_ERROR 0x00000004
#define CAN_MANAGER_RX_EVENT_ANY   (CAN_MANAGER_RX_EVENT_FIFO0 | CAN_MANAGER_RX_EVENT_FIFO1 | CAN_MANAGER_RX_EVENT_ERROR)

#define CAN_MANAGER_TX_EVENT_COMPLETE 0x00000001
#define CAN_MANAGER_TX_EVENT_ERROR    0x00000002
#define CAN_MANAGER_TX_EVENT_READY    0x00000004
#define CAN_MANAGER_TX_EVENT_ANY      (CAN_MANAGER_TX_EVENT_COMPLETE | CAN_MANAGER_TX_EVENT_ERROR)

typedef enum{
  can_manager_ok = 0,
  can_manager_invalid_argument,
  can_manager_not_initialized,
  can_manager_start_timeout,
  can_manager_no_tx_mailbox,
  can_manager_rx_empty,
  can_manager_tx_error,
  can_manager_error_warning,
  can_manager_error_passive,
  can_manager_bus_off,
  can_manager_ack_timeout,
  can_manager_recovery_failed
}can_manager_status_t;

typedef struct{
  uint32_t id;
  uint8_t dlc;
  bool extended_id;
  bool remote_frame;
  uint8_t data[8];
  uint16_t timestamp;
}can_frame_t;

typedef struct{
  volatile uint32_t initialized;
  volatile uint32_t init_count;
  volatile uint32_t send_call_count;
  volatile uint32_t no_tx_mailbox_count;
  volatile uint32_t tx_requested_count;
  volatile uint32_t tx_complete_count;
  volatile uint32_t tx_error_count;
  volatile uint32_t arbitration_lost_count;
  volatile uint32_t last_tx_request_tick;
  volatile uint32_t last_tx_ack_tick;
  volatile uint32_t ack_wait_start_tick;
  volatile uint32_t ack_timeout_count;
  volatile uint32_t ack_waiting;
  volatile uint32_t ack_timeout_reported;
  volatile uint32_t rx_irq_count;
  volatile uint32_t rx_received_count;
  volatile uint32_t rx_processed_count;
  volatile uint32_t rx_software_overflow_count;
  volatile uint32_t rx_hardware_overflow_count;
  volatile uint32_t error_irq_count;
  volatile uint32_t warning_count;
  volatile uint32_t error_passive_count;
  volatile uint32_t bus_off_count;
  volatile uint32_t recovery_attempt_count;
  volatile uint32_t recovery_success_count;
  volatile uint32_t recovery_failure_count;
  volatile uint32_t last_esr;
  volatile uint32_t last_msr;
  volatile uint32_t last_tsr;
  volatile uint32_t last_mcr;
  volatile uint32_t last_msr_snapshot;
  volatile uint32_t last_rf0r;
  volatile uint32_t last_ier;
  volatile uint32_t last_btr;
  volatile can_manager_status_t last_status;
}can_manager_debug_t;

extern volatile can_manager_debug_t g_can2_debug;
extern volatile can_frame_t g_can2_last_tx_frame;
extern const can_frame_t* volatile g_can2_last_rx_frame_ptr;

can_manager_status_t can_manager_init(CAN_TypeDef* can, osThreadId_t rx_notification_task, osThreadId_t tx_notification_task);
can_manager_status_t can_manager_send(CAN_TypeDef* can, const can_frame_t* frame);
can_manager_status_t can_manager_check_ack_timeout(CAN_TypeDef* can, uint32_t timeout_ms);
can_manager_status_t can_manager_recover(CAN_TypeDef* can);
const can_frame_t* can_manager_rx_peek(CAN_TypeDef* can);
can_manager_status_t can_manager_rx_release(CAN_TypeDef* can);
void can_manager_refresh_debug(CAN_TypeDef* can);
void can_manager_tx_irq_handler(CAN_TypeDef* can);
void can_manager_rx0_irq_handler(CAN_TypeDef* can);
void can_manager_rx1_irq_handler(CAN_TypeDef* can);
void can_manager_sce_irq_handler(CAN_TypeDef* can);

#endif
