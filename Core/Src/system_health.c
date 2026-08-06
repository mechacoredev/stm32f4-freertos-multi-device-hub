#include "system_health.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f407xx.h"
#include <limits.h>

system_health_diagnostics_t g_system_health;
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
