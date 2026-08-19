#include "system_health.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f407xx.h"
#include <limits.h>

system_health_diagnostics_t g_system_health;
volatile system_health_bus_diagnostics_t g_system_bus_health[SYSTEM_BUS_COUNT];
volatile system_health_source_diagnostics_t g_system_source_health[SYSTEM_SOURCE_COUNT];
volatile bool is_mcu_reset_allowed = false;
static osThreadId_t monitored_threads[HEALTH_TASK_COUNT];
static bool watchdog_started;

static void system_health_store_fault_task_name(const char* task_name){
    uint32_t index = 0;
    if(task_name != NULL){
        while(index < (sizeof(g_system_health.fault_task_name) - 1) &&
              task_name[index] != '\0'){
            g_system_health.fault_task_name[index] = task_name[index];
            index++;
        }
    }
    g_system_health.fault_task_name[index] = '\0';
}

void system_health_init(void){
    g_system_health.reset_cause = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    g_system_health.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    g_system_health.minimum_ever_free_heap_bytes =
            (uint32_t)xPortGetMinimumEverFreeHeapSize();
    g_system_health.watchdog_active = false;
    for(uint32_t index = 0; index < HEALTH_TASK_COUNT; index++){
        g_system_health.minimum_free_stack_words[index] = UINT32_MAX;
        monitored_threads[index] = NULL;
    }
    g_system_health.last_fault = HEALTH_FAULT_NONE;
    for(uint32_t index = 0; index < SYSTEM_BUS_COUNT; index++){
        g_system_bus_health[index].state = SYSTEM_BUS_STATE_OK;
        g_system_bus_health[index].last_success_source = SYSTEM_SOURCE_NONE;
        g_system_bus_health[index].last_success_operation = SYSTEM_OPERATION_NONE;
    }
    for(uint32_t index = 0; index < SYSTEM_SOURCE_COUNT; index++){
        g_system_source_health[index].last_bus = SYSTEM_BUS_COUNT;
        g_system_source_health[index].last_operation = SYSTEM_OPERATION_NONE;
        g_system_source_health[index].data_age_ms = UINT32_MAX;
    }
}

static uint32_t system_health_enter_short_critical(void){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void system_health_leave_short_critical(uint32_t primask){
    __DMB();
    if(primask == 0) __enable_irq();
}

void system_health_bus_record_success(system_health_bus_id_t bus,
        system_health_source_t source, system_health_operation_t operation,
        uint32_t address_or_id, uint32_t detail){
    if((uint32_t)bus >= SYSTEM_BUS_COUNT) return;
    uint32_t now = osKernelGetTickCount();
    uint32_t primask = system_health_enter_short_critical();
    volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[bus];
    bool recovered_from_fault = health->state != SYSTEM_BUS_STATE_OK;
    health->state = SYSTEM_BUS_STATE_OK;
    health->last_success_tick = now;
    health->data_age_ms = 0;
    health->fault_since_tick = 0;
    health->fault_age_ms = 0;
    health->consecutive_failure_count = 0;
    health->last_success_source = source;
    health->last_success_operation = operation;
    health->last_success_address_or_id = address_or_id;
    health->last_success_detail = detail;
    health->last_status = 0;
    health->reset_deadline_expired = false;
    health->reset_suppressed = false;

    if((source > SYSTEM_SOURCE_NONE) && (source < SYSTEM_SOURCE_COUNT)){
        volatile system_health_source_diagnostics_t *source_health =
                &g_system_source_health[source];
        source_health->success_count++;
        source_health->last_success_tick = now;
        source_health->data_age_ms = 0;
        source_health->last_bus = bus;
        source_health->last_operation = operation;
        source_health->last_address_or_id = address_or_id;
        source_health->last_detail = detail;
    }
    /* Recovery ancak gercek bir sensor/peripheral transferi yeniden basarili
     * oldugunda dogrulanmis sayilir. Hatlari HIGH gormek tek basina yeterli
     * degildir; sokulmus SCL pini de MCU tarafindaki pull-up nedeniyle HIGH
     * gorunebilir. */
    if(recovered_from_fault) health->recovery_success_count++;
    system_health_leave_short_critical(primask);
}

void system_health_bus_record_failure(system_health_bus_id_t bus,
        int32_t status){
    if((uint32_t)bus >= SYSTEM_BUS_COUNT) return;
    uint32_t now = osKernelGetTickCount();
    uint32_t primask = system_health_enter_short_critical();
    volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[bus];
    if(health->state == SYSTEM_BUS_STATE_OK){
        health->fault_since_tick = now;
    }
    health->state = SYSTEM_BUS_STATE_SUSPECT;
    health->last_failure_tick = now;
    health->last_status = status;
    health->consecutive_failure_count++;
    system_health_leave_short_critical(primask);
}

void system_health_bus_mark_recovering(system_health_bus_id_t bus){
    if((uint32_t)bus >= SYSTEM_BUS_COUNT) return;
    uint32_t primask = system_health_enter_short_critical();
    volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[bus];
    health->state = SYSTEM_BUS_STATE_RECOVERING;
    health->last_recovery_tick = osKernelGetTickCount();
    health->recovery_attempt_count++;
    system_health_leave_short_critical(primask);
}

void system_health_bus_record_recovery(system_health_bus_id_t bus,
        bool successful, int32_t status){
    if((uint32_t)bus >= SYSTEM_BUS_COUNT) return;
    uint32_t primask = system_health_enter_short_critical();
    volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[bus];
    health->last_status = status;
    if(successful){
        /* Donanim kurtarma komutunun basarili olmasi, sensor verisinin geri
         * geldigi anlamina gelmez. Hat ancak yeni bir basarili transfer
         * kaydedildiginde OK olur; bu arada ilk hata zamani korunur. */
        health->state = SYSTEM_BUS_STATE_SUSPECT;
    }else{
        health->state = SYSTEM_BUS_STATE_FAILED;
        health->recovery_failure_count++;
    }
    system_health_leave_short_critical(primask);
}

void system_health_bus_refresh_ages(uint32_t now){
    for(uint32_t index = 0; index < SYSTEM_BUS_COUNT; index++){
        volatile system_health_bus_diagnostics_t *health = &g_system_bus_health[index];
        if(health->last_success_tick == 0){
            health->data_age_ms = UINT32_MAX;
        }else{
            health->data_age_ms = now - health->last_success_tick;
        }

        if(health->fault_since_tick == 0){
            health->fault_age_ms = 0;
        }else{
            health->fault_age_ms = now - health->fault_since_tick;
        }
    }

    for(uint32_t index = 1; index < SYSTEM_SOURCE_COUNT; index++){
        volatile system_health_source_diagnostics_t *source_health =
                &g_system_source_health[index];
        if(source_health->last_success_tick == 0){
            source_health->data_age_ms = UINT32_MAX;
        }else{
            source_health->data_age_ms = now - source_health->last_success_tick;
        }
    }
}

void system_health_register_task(system_health_task_id_t id,
        osThreadId_t thread){
    if((uint32_t)id >= HEALTH_TASK_COUNT) return;
    monitored_threads[id] = thread;
}

void system_health_heartbeat(system_health_task_id_t id){
    if((uint32_t)id >= HEALTH_TASK_COUNT) return;
    taskENTER_CRITICAL();
    g_system_health.heartbeat_mask |= SYSTEM_HEALTH_TASK_MASK(id);
    taskEXIT_CRITICAL();
}

bool system_health_evaluate_window(uint32_t required_mask){
    uint32_t received_mask;
    taskENTER_CRITICAL();
    received_mask = g_system_health.heartbeat_mask;
    g_system_health.heartbeat_mask = 0;
    taskEXIT_CRITICAL();

    g_system_health.missing_heartbeat_mask = required_mask & ~received_mask;
    g_system_health.monitor_cycle_count++;
    g_system_health.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    g_system_health.minimum_ever_free_heap_bytes =
            (uint32_t)xPortGetMinimumEverFreeHeapSize();

    for(uint32_t index = 0; index < HEALTH_TASK_COUNT; index++){
        if(monitored_threads[index] == NULL) continue;
        UBaseType_t free_words = uxTaskGetStackHighWaterMark(
                (TaskHandle_t)monitored_threads[index]);
        if(free_words < g_system_health.minimum_free_stack_words[index]){
            g_system_health.minimum_free_stack_words[index] = free_words;
        }
    }
    return g_system_health.missing_heartbeat_mask == 0;
}

bool system_health_watchdog_start(void){
    if(watchdog_started) return true;

    /* LSI is nominally 32 kHz. Prescaler 64 and reload 2499 give an
     * approximately 5 second timeout; the exact value follows LSI tolerance. */
    const uint32_t startup_timeout_ms = 100;
    uint32_t wait_start = osKernelGetTickCount();
    RCC->CSR |= RCC_CSR_LSION;
    while((RCC->CSR & RCC_CSR_LSIRDY) == 0){
        if((osKernelGetTickCount() - wait_start) >= startup_timeout_ms){
            g_system_health.watchdog_start_failure_count++;
            return false;
        }
        osDelay(1);
    }

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    /* On STM32F4 the IWDG must be running before PVU/RVU can complete.
     * Starting it is irreversible until reset, so configure and reload it
     * immediately afterwards. This follows ST's HAL_IWDG_Init sequence. */
    IWDG->KR = 0xCCCC;
    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    IWDG->RLR = 2499;

    wait_start = osKernelGetTickCount();
    while((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0){
        g_system_health.watchdog_status_register = IWDG->SR;
        if((osKernelGetTickCount() - wait_start) >= startup_timeout_ms){
            g_system_health.watchdog_start_failure_count++;
            return false;
        }
        osDelay(1);
    }

    IWDG->KR = 0xAAAA;
    g_system_health.watchdog_status_register = IWDG->SR;
    watchdog_started = true;
    g_system_health.watchdog_active = true;
    return true;
}

void system_health_watchdog_feed(void){
    if(!watchdog_started){
        g_system_health.watchdog_rejected_feed_count++;
        return;
    }
    IWDG->KR = 0xAAAA;
    g_system_health.watchdog_feed_count++;
}

void system_health_record_fatal(system_health_fault_t fault,
        uintptr_t task){
    taskDISABLE_INTERRUPTS();
    g_system_health.last_fault = fault;
    g_system_health.fault_task = task;
    __DSB();
    __ISB();

    if((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0){
        __BKPT(0);
        while(1) { }
    }
    NVIC_SystemReset();
    while(1) { }
}

void system_health_assert_failed(void){
    system_health_store_fault_task_name(pcTaskGetName(NULL));
    system_health_record_fatal(HEALTH_FAULT_ASSERT,
            (uintptr_t)xTaskGetCurrentTaskHandle());
}

void system_health_report_stack_overflow(uintptr_t task,
        const char *task_name){
    system_health_store_fault_task_name(task_name);
    system_health_record_fatal(HEALTH_FAULT_STACK_OVERFLOW,
            task);
}

void system_health_report_malloc_failure(void){
    system_health_record_fatal(HEALTH_FAULT_MALLOC_FAILED, 0);
}
