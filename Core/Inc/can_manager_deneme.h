#ifndef INC_CAN_MANAGER_DENEME_H_
#define INC_CAN_MANAGER_DENEME_H_

#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

#define CAN_DENEME_EVENT_RX       0x00000001
#define CAN_DENEME_EVENT_TX       0x00000002
#define CAN_DENEME_EVENT_ERROR    0x00000004
#define CAN_DENEME_EVENT_ANY      (CAN_DENEME_EVENT_RX | CAN_DENEME_EVENT_TX | CAN_DENEME_EVENT_ERROR)

typedef enum{
  can_deneme_ok = 0,
  can_deneme_invalid_argument,
  can_deneme_not_initialized,
  can_deneme_start_timeout,
  can_deneme_no_tx_mailbox,
  can_deneme_rx_empty
}can_deneme_status_t;

typedef struct{
  uint32_t id;
  uint8_t dlc;
  bool extended_id;
  bool remote_frame;
  uint8_t data[8];
  uint16_t timestamp;
}can_deneme_frame_t;

typedef struct{
  volatile uint32_t initialized;
  volatile uint32_t init_count;
  volatile uint32_t send_call_count;
  volatile uint32_t no_tx_mailbox_count;
  volatile uint32_t tx_requested_count;
  volatile uint32_t tx_complete_count;
  volatile uint32_t tx_error_count;
  volatile uint32_t arbitration_lost_count;
  volatile uint32_t rx_irq_count;
  volatile uint32_t rx_received_count;
  volatile uint32_t rx_processed_count;
  volatile uint32_t rx_software_overflow_count;
  volatile uint32_t rx_hardware_overflow_count;
  volatile uint32_t error_irq_count;
  volatile uint32_t warning_count;
  volatile uint32_t error_passive_count;
  volatile uint32_t bus_off_count;
  volatile uint32_t last_esr;
  volatile uint32_t last_msr;
  volatile uint32_t last_tsr;
  volatile uint32_t last_mcr;
  volatile uint32_t last_rf0r;
  volatile uint32_t last_ier;
  volatile uint32_t last_btr;
  volatile can_deneme_status_t last_status;
}can_deneme_debug_t;

extern volatile can_deneme_debug_t g_can2_deneme_debug;
extern volatile can_deneme_frame_t g_can2_deneme_last_rx_frame;
extern volatile can_deneme_frame_t g_can2_deneme_last_tx_frame;

can_deneme_status_t can_deneme_init(CAN_TypeDef* can, osThreadId_t notification_task);
can_deneme_status_t can_deneme_send(CAN_TypeDef* can, const can_deneme_frame_t* frame);
can_deneme_status_t can_deneme_receive(CAN_TypeDef* can, can_deneme_frame_t* frame);
void can_deneme_refresh_debug(CAN_TypeDef* can);
void can_deneme_tx_irq_handler(CAN_TypeDef* can);
void can_deneme_rx0_irq_handler(CAN_TypeDef* can);
void can_deneme_rx1_irq_handler(CAN_TypeDef* can);
void can_deneme_sce_irq_handler(CAN_TypeDef* can);

#endif
