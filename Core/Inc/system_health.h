#ifndef INC_SYSTEM_HEALTH_H_
#define INC_SYSTEM_HEALTH_H_

#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum{
    HEALTH_TASK_BME280 = 0,
    HEALTH_TASK_I2C_DISPATCH,
    HEALTH_TASK_RC522,
    HEALTH_TASK_MPU6050,
    HEALTH_TASK_SPI_DISPATCH,
    HEALTH_TASK_NRF_TX,
    HEALTH_TASK_NRF_RX,
    HEALTH_TASK_DISPLAY,
    HEALTH_TASK_MONITOR,
    HEALTH_TASK_COUNT
} system_health_task_id_t;

typedef enum{
    HEALTH_FAULT_NONE = 0,
    HEALTH_FAULT_STACK_OVERFLOW,
    HEALTH_FAULT_MALLOC_FAILED,
    HEALTH_FAULT_ASSERT
} system_health_fault_t;

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

#define SYSTEM_HEALTH_TASK_MASK(id) (1 << (uint32_t)(id))
#define SYSTEM_HEALTH_ALL_APPLICATION_TASKS \
    ((1 << (uint32_t)HEALTH_TASK_COUNT) - 1)

#endif
