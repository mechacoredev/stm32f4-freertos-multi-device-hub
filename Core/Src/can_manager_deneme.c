#include "can_manager_deneme.h"

#define CAN_DENEME_RX_QUEUE_SIZE       16
#define CAN_DENEME_FILTER_BANK         14
#define CAN_DENEME_START_TIMEOUT_MS    100
#define CAN_DENEME_TX_MAILBOX_COUNT    3

typedef struct{
  bool sleep_mode;
  bool automatic_retransmission;
  bool receive_fifo_locked_mode;
  bool transmit_fifo_request_order;
  bool automatic_bus_off_management;
}can_master_control_config_t;

typedef struct{
  CAN_TypeDef* can;
  osThreadId_t notification_task;
  volatile uint8_t rx_write_index;
  volatile uint8_t rx_read_index;
  volatile uint8_t rx_count;
  volatile bool initialized;
  can_deneme_frame_t rx_queue[CAN_DENEME_RX_QUEUE_SIZE];
}can_deneme_context_t;

static can_deneme_context_t can2_context = {0};

volatile can_deneme_debug_t g_can2_deneme_debug = {0};
volatile can_deneme_frame_t g_can2_deneme_last_rx_frame = {0};
volatile can_deneme_frame_t g_can2_deneme_last_tx_frame = {0};

/* -------------------------------------------------------------------------- */
/* Register katmani: isimler register kisaltmasini degil, fiziksel isi anlatir. */
/* -------------------------------------------------------------------------- */

static void can_master_control_apply_config(CAN_TypeDef* can, const can_master_control_config_t* config){
  uint32_t master_control = can->MCR;

  if(config->sleep_mode) master_control |= CAN_MCR_SLEEP;
  else master_control &= ~CAN_MCR_SLEEP;

  if(config->automatic_retransmission) master_control &= ~CAN_MCR_NART;
  else master_control |= CAN_MCR_NART;

  if(config->receive_fifo_locked_mode) master_control |= CAN_MCR_RFLM;
  else master_control &= ~CAN_MCR_RFLM;

  if(config->transmit_fifo_request_order) master_control |= CAN_MCR_TXFP;
  else master_control &= ~CAN_MCR_TXFP;

  if(config->automatic_bus_off_management) master_control |= CAN_MCR_ABOM;
  else master_control &= ~CAN_MCR_ABOM;

  can->MCR = master_control;
}

static void can_all_interrupts_disable(CAN_TypeDef* can){
  can->IER = 0;
}

static void can_runtime_interrupts_enable(CAN_TypeDef* can){
  can->IER = CAN_IER_TMEIE |
          CAN_IER_FMPIE0 | CAN_IER_FOVIE0 |
          CAN_IER_FMPIE1 | CAN_IER_FOVIE1 |
          CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE |
          CAN_IER_LECIE | CAN_IER_ERRIE;
}

static void can_request_normal_mode(CAN_TypeDef* can){
  can->MCR &= ~CAN_MCR_INRQ;
}

static bool can_is_in_initialization_mode(CAN_TypeDef* can){
  return (can->MSR & CAN_MSR_INAK) != 0;
}

static uint8_t can_receive_fifo_pending_count(CAN_TypeDef* can, uint8_t fifo){
  uint32_t fifo_status = fifo == 0 ? can->RF0R : can->RF1R;
  uint32_t pending_mask = fifo == 0 ? CAN_RF0R_FMP0 : CAN_RF1R_FMP1;
  return (uint8_t)(fifo_status & pending_mask);
}

static bool can_receive_fifo_overflowed(CAN_TypeDef* can, uint8_t fifo){
  return fifo == 0 ? ((can->RF0R & CAN_RF0R_FOVR0) != 0) : ((can->RF1R & CAN_RF1R_FOVR1) != 0);
}

static void can_receive_fifo_clear_overflow(CAN_TypeDef* can, uint8_t fifo){
  if(fifo == 0) can->RF0R = CAN_RF0R_FOVR0;
  else can->RF1R = CAN_RF1R_FOVR1;
}

static void can_receive_fifo_release_oldest_frame(CAN_TypeDef* can, uint8_t fifo){
  if(fifo == 0) can->RF0R = CAN_RF0R_RFOM0;
  else can->RF1R = CAN_RF1R_RFOM1;
}

static void can_receive_fifo_discard_all(CAN_TypeDef* can, uint8_t fifo){
  while(can_receive_fifo_pending_count(can, fifo) != 0){
    can_receive_fifo_release_oldest_frame(can, fifo);
  }
}

static bool can_transmit_mailbox_is_empty(CAN_TypeDef* can, uint8_t mailbox_index){
  const uint32_t empty_flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2};
  return mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT && (can->TSR & empty_flags[mailbox_index]) != 0;
}

static int8_t can_find_empty_transmit_mailbox(CAN_TypeDef* can){
  for(uint8_t index = 0; index < CAN_DENEME_TX_MAILBOX_COUNT; index++){
    if(can_transmit_mailbox_is_empty(can, index)) return (int8_t)index;
  }

  return -1;
}

static bool can_transmit_mailbox_request_completed(uint32_t transmit_status, uint8_t mailbox_index){
  const uint32_t flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2};
  return mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT && (transmit_status & flags[mailbox_index]) != 0;
}

static bool can_transmit_mailbox_succeeded(uint32_t transmit_status, uint8_t mailbox_index){
  const uint32_t flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2};
  return mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT && (transmit_status & flags[mailbox_index]) != 0;
}

static bool can_transmit_mailbox_lost_arbitration(uint32_t transmit_status, uint8_t mailbox_index){
  const uint32_t flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_ALST0, CAN_TSR_ALST1, CAN_TSR_ALST2};
  return mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT && (transmit_status & flags[mailbox_index]) != 0;
}

static bool can_transmit_mailbox_had_error(uint32_t transmit_status, uint8_t mailbox_index){
  const uint32_t flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2};
  return mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT && (transmit_status & flags[mailbox_index]) != 0;
}

static void can_transmit_clear_mailbox_completion(CAN_TypeDef* can, uint8_t mailbox_index){
  const uint32_t flags[CAN_DENEME_TX_MAILBOX_COUNT] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2};
  if(mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT) can->TSR = flags[mailbox_index];
}

static void can_transmit_clear_all_completions(CAN_TypeDef* can){
  can->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
}

static void can_clear_status_interrupt_reasons(CAN_TypeDef* can){
  can->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;
}

static void can_clear_last_error_code(CAN_TypeDef* can){
  can->ESR &= ~CAN_ESR_LEC;
}

static bool can_error_warning_is_active(uint32_t error_status){
  return (error_status & CAN_ESR_EWGF) != 0;
}

static bool can_error_passive_is_active(uint32_t error_status){
  return (error_status & CAN_ESR_EPVF) != 0;
}

static bool can_bus_off_is_active(uint32_t error_status){
  return (error_status & CAN_ESR_BOFF) != 0;
}

static void can_clear_error_interrupt_reason(CAN_TypeDef* can){
  can->MSR = CAN_MSR_ERRI;
}

static void can_filter_configure_accept_all_for_can2(void){
  uint32_t filter_bit = 1 << CAN_DENEME_FILTER_BANK;

  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FMR = (CAN1->FMR & ~CAN_FMR_CAN2SB) |
          (CAN_DENEME_FILTER_BANK << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT;

  CAN1->FA1R &= ~filter_bit;
  CAN1->FM1R &= ~filter_bit;
  CAN1->FS1R |= filter_bit;
  CAN1->FFA1R &= ~filter_bit;
  CAN1->sFilterRegister[CAN_DENEME_FILTER_BANK].FR1 = 0;
  CAN1->sFilterRegister[CAN_DENEME_FILTER_BANK].FR2 = 0;
  CAN1->FA1R |= filter_bit;

  CAN1->FMR &= ~CAN_FMR_FINIT;
}

static void can_transmit_mailbox_load_frame(CAN_TypeDef* can, uint8_t mailbox_index, const can_deneme_frame_t* frame){
  CAN_TxMailBox_TypeDef* mailbox = &can->sTxMailBox[mailbox_index];
  uint32_t identifier = frame->extended_id ? ((frame->id & 0x1FFFFFFF) << 3) | CAN_TI0R_IDE : ((frame->id & 0x7FF) << 21);

  if(frame->remote_frame) identifier |= CAN_TI0R_RTR;

  mailbox->TIR = identifier;
  mailbox->TDTR = frame->dlc & CAN_TDT0R_DLC;
  mailbox->TDLR = ((uint32_t)frame->data[0] << 0) | ((uint32_t)frame->data[1] << 8) |
          ((uint32_t)frame->data[2] << 16) | ((uint32_t)frame->data[3] << 24);
  mailbox->TDHR = ((uint32_t)frame->data[4] << 0) | ((uint32_t)frame->data[5] << 8) |
          ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
}

static void can_transmit_mailbox_request_transmission(CAN_TypeDef* can, uint8_t mailbox_index){
  can->sTxMailBox[mailbox_index].TIR |= CAN_TI0R_TXRQ;
}

static void can_receive_fifo_read_oldest_frame(CAN_TypeDef* can, uint8_t fifo, can_deneme_frame_t* frame){
  CAN_FIFOMailBox_TypeDef* mailbox = &can->sFIFOMailBox[fifo];
  uint32_t identifier = mailbox->RIR;
  uint32_t length_and_time = mailbox->RDTR;
  uint32_t data_low = mailbox->RDLR;
  uint32_t data_high = mailbox->RDHR;

  frame->extended_id = (identifier & CAN_RI0R_IDE) != 0;
  frame->remote_frame = (identifier & CAN_RI0R_RTR) != 0;
  frame->id = frame->extended_id ? ((identifier >> 3) & 0x1FFFFFFF) : ((identifier >> 21) & 0x7FF);
  frame->dlc = (uint8_t)(length_and_time & CAN_RDT0R_DLC);
  if(frame->dlc > 8) frame->dlc = 8;
  frame->timestamp = (uint16_t)(length_and_time >> 16);
  frame->data[0] = (uint8_t)(data_low >> 0);
  frame->data[1] = (uint8_t)(data_low >> 8);
  frame->data[2] = (uint8_t)(data_low >> 16);
  frame->data[3] = (uint8_t)(data_low >> 24);
  frame->data[4] = (uint8_t)(data_high >> 0);
  frame->data[5] = (uint8_t)(data_high >> 8);
  frame->data[6] = (uint8_t)(data_high >> 16);
  frame->data[7] = (uint8_t)(data_high >> 24);
}

/* -------------------------------------------------------------------------- */
/* Manager katmani: register ayrintilari yerine yukaridaki anlamli islemleri kullanir. */
/* -------------------------------------------------------------------------- */

static void copy_frame(can_deneme_frame_t* destination, const can_deneme_frame_t* source){
  destination->id = source->id;
  destination->dlc = source->dlc;
  destination->extended_id = source->extended_id;
  destination->remote_frame = source->remote_frame;
  destination->timestamp = source->timestamp;

  for(uint8_t index = 0; index < 8; index++) destination->data[index] = source->data[index];
}

static void copy_frame_to_volatile(volatile can_deneme_frame_t* destination, const can_deneme_frame_t* source){
  destination->id = source->id;
  destination->dlc = source->dlc;
  destination->extended_id = source->extended_id;
  destination->remote_frame = source->remote_frame;
  destination->timestamp = source->timestamp;

  for(uint8_t index = 0; index < 8; index++) destination->data[index] = source->data[index];
}

static void notify_can_task(uint32_t flag){
  if(can2_context.notification_task != NULL) (void)osThreadFlagsSet(can2_context.notification_task, flag);
}

static bool frame_is_valid(const can_deneme_frame_t* frame){
  if(frame == NULL || frame->dlc > 8) return false;
  if(!frame->extended_id && frame->id > 0x7FF) return false;
  if(frame->extended_id && frame->id > 0x1FFFFFFF) return false;
  return true;
}

static void store_received_frame(const can_deneme_frame_t* frame){
  if(can2_context.rx_count >= CAN_DENEME_RX_QUEUE_SIZE){
    g_can2_deneme_debug.rx_software_overflow_count++;
    return;
  }

  copy_frame(&can2_context.rx_queue[can2_context.rx_write_index], frame);
  can2_context.rx_write_index = (uint8_t)((can2_context.rx_write_index + 1) % CAN_DENEME_RX_QUEUE_SIZE);
  can2_context.rx_count++;
  copy_frame_to_volatile(&g_can2_deneme_last_rx_frame, frame);
  g_can2_deneme_debug.rx_received_count++;
}

static void drain_receive_fifo(CAN_TypeDef* can, uint8_t fifo){
  while(can_receive_fifo_pending_count(can, fifo) != 0){
    can_deneme_frame_t frame;

    can_receive_fifo_read_oldest_frame(can, fifo, &frame);
    store_received_frame(&frame);
    can_receive_fifo_release_oldest_frame(can, fifo);
  }
}

void can_deneme_refresh_debug(CAN_TypeDef* can){
  if(can != CAN2) return;

  g_can2_deneme_debug.last_mcr = can->MCR;
  g_can2_deneme_debug.last_msr = can->MSR;
  g_can2_deneme_debug.last_tsr = can->TSR;
  g_can2_deneme_debug.last_rf0r = can->RF0R;
  g_can2_deneme_debug.last_ier = can->IER;
  g_can2_deneme_debug.last_btr = can->BTR;
  g_can2_deneme_debug.last_esr = can->ESR;
}

can_deneme_status_t can_deneme_init(CAN_TypeDef* can, osThreadId_t notification_task){
  if(can != CAN2 || notification_task == NULL){
    g_can2_deneme_debug.last_status = can_deneme_invalid_argument;
    return can_deneme_invalid_argument;
  }

  can2_context.can = can;
  can2_context.notification_task = notification_task;
  can2_context.rx_write_index = 0;
  can2_context.rx_read_index = 0;
  can2_context.rx_count = 0;
  can2_context.initialized = false;

  can_all_interrupts_disable(can);

  const can_master_control_config_t control_config = {
    .sleep_mode = false,
    .automatic_retransmission = true,
    .receive_fifo_locked_mode = false,
    .transmit_fifo_request_order = false,
    .automatic_bus_off_management = true
  };
  can_master_control_apply_config(can, &control_config);

  can_filter_configure_accept_all_for_can2();
  can_receive_fifo_discard_all(can, 0);
  can_receive_fifo_discard_all(can, 1);
  can_transmit_clear_all_completions(can);
  can_clear_status_interrupt_reasons(can);
  can_clear_last_error_code(can);
  can_runtime_interrupts_enable(can);
  can_request_normal_mode(can);

  uint32_t start_tick = osKernelGetTickCount();
  while(can_is_in_initialization_mode(can)){
    if((osKernelGetTickCount() - start_tick) >= CAN_DENEME_START_TIMEOUT_MS){
      g_can2_deneme_debug.last_status = can_deneme_start_timeout;
      can_deneme_refresh_debug(can);
      return can_deneme_start_timeout;
    }
  }

  can2_context.initialized = true;
  g_can2_deneme_debug.initialized = 1;
  g_can2_deneme_debug.init_count++;
  g_can2_deneme_debug.last_status = can_deneme_ok;
  can_deneme_refresh_debug(can);
  return can_deneme_ok;
}

can_deneme_status_t can_deneme_send(CAN_TypeDef* can, const can_deneme_frame_t* frame){
  g_can2_deneme_debug.send_call_count++;

  if(can != CAN2 || !frame_is_valid(frame)){
    g_can2_deneme_debug.last_status = can_deneme_invalid_argument;
    return can_deneme_invalid_argument;
  }

  if(!can2_context.initialized){
    g_can2_deneme_debug.last_status = can_deneme_not_initialized;
    return can_deneme_not_initialized;
  }

  int8_t mailbox_index = can_find_empty_transmit_mailbox(can);
  if(mailbox_index < 0){
    g_can2_deneme_debug.no_tx_mailbox_count++;
    g_can2_deneme_debug.last_status = can_deneme_no_tx_mailbox;
    can_deneme_refresh_debug(can);
    return can_deneme_no_tx_mailbox;
  }

  can_transmit_mailbox_load_frame(can, (uint8_t)mailbox_index, frame);
  copy_frame_to_volatile(&g_can2_deneme_last_tx_frame, frame);
  g_can2_deneme_debug.tx_requested_count++;
  g_can2_deneme_debug.last_status = can_deneme_ok;
  can_transmit_mailbox_request_transmission(can, (uint8_t)mailbox_index);
  can_deneme_refresh_debug(can);
  return can_deneme_ok;
}

can_deneme_status_t can_deneme_receive(CAN_TypeDef* can, can_deneme_frame_t* frame){
  if(can != CAN2 || frame == NULL) return can_deneme_invalid_argument;
  if(!can2_context.initialized) return can_deneme_not_initialized;
  if(can2_context.rx_count == 0) return can_deneme_rx_empty;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  copy_frame(frame, &can2_context.rx_queue[can2_context.rx_read_index]);
  can2_context.rx_read_index = (uint8_t)((can2_context.rx_read_index + 1) % CAN_DENEME_RX_QUEUE_SIZE);
  can2_context.rx_count--;

  if(primask == 0) __enable_irq();

  g_can2_deneme_debug.rx_processed_count++;
  return can_deneme_ok;
}

void can_deneme_tx_irq_handler(CAN_TypeDef* can){
  if(can != CAN2 || !can2_context.initialized) return;

  uint32_t transmit_status = can->TSR;
  g_can2_deneme_debug.last_tsr = transmit_status;

  for(uint8_t mailbox_index = 0; mailbox_index < CAN_DENEME_TX_MAILBOX_COUNT; mailbox_index++){
    if(!can_transmit_mailbox_request_completed(transmit_status, mailbox_index)) continue;

    if(can_transmit_mailbox_succeeded(transmit_status, mailbox_index)) g_can2_deneme_debug.tx_complete_count++;
    if(can_transmit_mailbox_lost_arbitration(transmit_status, mailbox_index)) g_can2_deneme_debug.arbitration_lost_count++;
    if(can_transmit_mailbox_had_error(transmit_status, mailbox_index)) g_can2_deneme_debug.tx_error_count++;

    can_transmit_clear_mailbox_completion(can, mailbox_index);
  }

  notify_can_task(CAN_DENEME_EVENT_TX);
}

void can_deneme_rx0_irq_handler(CAN_TypeDef* can){
  if(can != CAN2 || !can2_context.initialized) return;

  g_can2_deneme_debug.rx_irq_count++;

  if(can_receive_fifo_overflowed(can, 0)){
    g_can2_deneme_debug.rx_hardware_overflow_count++;
    can_receive_fifo_clear_overflow(can, 0);
  }

  drain_receive_fifo(can, 0);
  notify_can_task(CAN_DENEME_EVENT_RX);
}

void can_deneme_rx1_irq_handler(CAN_TypeDef* can){
  if(can != CAN2 || !can2_context.initialized) return;

  g_can2_deneme_debug.rx_irq_count++;

  if(can_receive_fifo_overflowed(can, 1)){
    g_can2_deneme_debug.rx_hardware_overflow_count++;
    can_receive_fifo_clear_overflow(can, 1);
  }

  drain_receive_fifo(can, 1);
  notify_can_task(CAN_DENEME_EVENT_RX);
}

void can_deneme_sce_irq_handler(CAN_TypeDef* can){
  if(can != CAN2 || !can2_context.initialized) return;

  uint32_t error_status = can->ESR;
  g_can2_deneme_debug.error_irq_count++;
  g_can2_deneme_debug.last_esr = error_status;
  g_can2_deneme_debug.last_msr = can->MSR;

  if(can_error_warning_is_active(error_status)) g_can2_deneme_debug.warning_count++;
  if(can_error_passive_is_active(error_status)) g_can2_deneme_debug.error_passive_count++;
  if(can_bus_off_is_active(error_status)) g_can2_deneme_debug.bus_off_count++;

  can_clear_last_error_code(can);
  can_clear_error_interrupt_reason(can);
  notify_can_task(CAN_DENEME_EVENT_ERROR);
}
