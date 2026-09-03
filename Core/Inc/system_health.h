#ifndef INC_SYSTEM_HEALTH_H_
#define INC_SYSTEM_HEALTH_H_

#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum{
    HEALTH_TASK_BME280 = 0,
    HEALTH_TASK_I2C_DISPATCH,
    HEALTH_TASK_RC522,
    HEALTH_TASK_MPU6500,
    HEALTH_TASK_SPI_DISPATCH,
    HEALTH_TASK_NRF_TX,
    HEALTH_TASK_NRF_RX,
    HEALTH_TASK_DISPLAY,
    HEALTH_TASK_CAN_RX,
    HEALTH_TASK_CAN_TX,
    HEALTH_TASK_ADXL345,
    HEALTH_TASK_VL53L0X,
    HEALTH_TASK_SSD1306_SPI,
    HEALTH_TASK_MONITOR,
    HEALTH_TASK_COUNT
} system_health_task_id_t;

typedef enum{
    HEALTH_FAULT_NONE = 0,
    HEALTH_FAULT_STACK_OVERFLOW,
    HEALTH_FAULT_MALLOC_FAILED,
    HEALTH_FAULT_ASSERT
} system_health_fault_t;

typedef enum{
    SYSTEM_BUS_I2C1 = 0,
    SYSTEM_BUS_I2C2,
    SYSTEM_BUS_I2C3,
    SYSTEM_BUS_SPI1,
    SYSTEM_BUS_SPI2,
    SYSTEM_BUS_SPI3,
    SYSTEM_BUS_CAN1,
    SYSTEM_BUS_CAN2,
    SYSTEM_BUS_COUNT
} system_health_bus_id_t;

typedef enum{
    SYSTEM_BUS_STATE_OK = 0,
    SYSTEM_BUS_STATE_SUSPECT,
    SYSTEM_BUS_STATE_RECOVERING,
    SYSTEM_BUS_STATE_FAILED
} system_health_bus_state_t;

typedef enum{
    SYSTEM_SOURCE_NONE = 0,
    SYSTEM_SOURCE_BME280,
    SYSTEM_SOURCE_MPU6500,
    SYSTEM_SOURCE_RC522,
    SYSTEM_SOURCE_NRF24_TX,
    SYSTEM_SOURCE_NRF24_RX,
    SYSTEM_SOURCE_ILI9341,
    SYSTEM_SOURCE_CAN_TX,
    SYSTEM_SOURCE_CAN_RX,
    SYSTEM_SOURCE_ADXL345,
    SYSTEM_SOURCE_VL53L0X,
    SYSTEM_SOURCE_SSD1306_SPI,
    SYSTEM_SOURCE_COUNT
} system_health_source_t;

typedef enum{
    SYSTEM_OPERATION_NONE = 0,
    SYSTEM_OPERATION_READ,
    SYSTEM_OPERATION_WRITE,
    SYSTEM_OPERATION_TRANSFER
} system_health_operation_t;

typedef struct{
    volatile system_health_bus_state_t state;
    volatile uint32_t last_success_tick;
    volatile uint32_t data_age_ms;
    volatile uint32_t last_failure_tick;
    volatile uint32_t fault_since_tick;
    volatile uint32_t fault_age_ms;
    volatile uint32_t last_recovery_tick;
    volatile uint32_t recovery_attempt_count;
    volatile uint32_t recovery_success_count;
    volatile uint32_t recovery_failure_count;
    volatile uint32_t consecutive_failure_count;
    volatile system_health_source_t last_success_source;
    volatile system_health_operation_t last_success_operation;
    volatile uint32_t last_success_address_or_id;
    volatile uint32_t last_success_detail;
    volatile int32_t last_status;
    volatile bool reset_deadline_expired;
    volatile bool reset_suppressed;
} system_health_bus_diagnostics_t;

typedef struct{
    volatile uint32_t success_count;
    volatile uint32_t last_success_tick;
    volatile uint32_t data_age_ms;
    volatile system_health_bus_id_t last_bus;
    volatile system_health_operation_t last_operation;
    volatile uint32_t last_address_or_id;
    volatile uint32_t last_detail;
} system_health_source_diagnostics_t;

typedef struct{
    volatile uint32_t reset_cause;
    volatile uint32_t heartbeat_mask;
    volatile uint32_t missing_heartbeat_mask;
    volatile uint32_t watchdog_feed_count;
    volatile uint32_t watchdog_rejected_feed_count;
    volatile uint32_t watchdog_start_failure_count;
    volatile uint32_t watchdog_status_register;
    volatile uint32_t monitor_cycle_count;
    volatile uint32_t free_heap_bytes;
    volatile uint32_t minimum_ever_free_heap_bytes;
    volatile bool watchdog_active;
    volatile uint32_t minimum_free_stack_words[HEALTH_TASK_COUNT];
    volatile system_health_fault_t last_fault;
    volatile uintptr_t fault_task;
    volatile char fault_task_name[16];
} system_health_diagnostics_t;

extern system_health_diagnostics_t g_system_health;
extern volatile system_health_bus_diagnostics_t g_system_bus_health[SYSTEM_BUS_COUNT];
extern volatile system_health_source_diagnostics_t g_system_source_health[SYSTEM_SOURCE_COUNT];
extern volatile bool is_mcu_reset_allowed;

void system_health_init(void);
void system_health_register_task(system_health_task_id_t id,
        osThreadId_t thread);
void system_health_heartbeat(system_health_task_id_t id);
bool system_health_evaluate_window(uint32_t required_mask);
bool system_health_watchdog_start(void);
void system_health_watchdog_feed(void);
void system_health_record_fatal(system_health_fault_t fault,
        uintptr_t task);
void system_health_assert_failed(void);
void system_health_report_stack_overflow(uintptr_t task,
        const char *task_name);
void system_health_report_malloc_failure(void);
void system_health_bus_record_success(system_health_bus_id_t bus,
        system_health_source_t source, system_health_operation_t operation,
        uint32_t address_or_id, uint32_t detail);
void system_health_bus_record_failure(system_health_bus_id_t bus,
        int32_t status);
void system_health_bus_mark_recovering(system_health_bus_id_t bus);
void system_health_bus_record_recovery(system_health_bus_id_t bus,
        bool successful, int32_t status);
void system_health_bus_refresh_ages(uint32_t now);

#define SYSTEM_HEALTH_TASK_MASK(id) (1 << (uint32_t)(id))
#define SYSTEM_HEALTH_ALL_APPLICATION_TASKS \
    ((1 << (uint32_t)HEALTH_TASK_COUNT) - 1)

#endif
