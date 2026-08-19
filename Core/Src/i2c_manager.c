#include "i2c_manager.h"

typedef enum{
    I2C_DMA_IDLE = 0,
    I2C_DMA_START_SENT,
    I2C_DMA_WRITE_ADDRESS_SENT,
    I2C_DMA_REGISTER_SENT,
    I2C_DMA_RESTART_SENT,
    I2C_DMA_READ_ADDRESS_SENT,
    I2C_DMA_READING
} i2c_dma_state_t;

typedef struct{
    I2C_TypeDef* i2c;
    osMutexId_t mutex;
    osSemaphoreId_t completion_semaphore;
    osThreadId_t dispatcher;
    uint32_t dispatcher_flag;
    i2c_manager_bus_recovery_t recovery;
    DMA_TypeDef* dma;
    uint32_t dma_stream;
    uint8_t device_address;
    uint8_t register_address;
    uint8_t* receive_buffer;
    uint16_t size;
    volatile i2c_dma_state_t state;
    volatile i2c_manager_return_status result;
} i2c_bus_context_t;

static i2c_bus_context_t buses[3];
volatile i2c_manager_diagnostics_t g_i2c_manager_diagnostics[3];

typedef struct{
    osMessageQueueId_t queue;
    osThreadId_t dispatcher;
    uint32_t new_job_flag;
} i2c_job_service_t;

static i2c_job_service_t job_service;

static int8_t get_bus_index(I2C_TypeDef* i2c){
    if(i2c == I2C1) return 0;
    if(i2c == I2C2) return 1;
    if(i2c == I2C3) return 2;
    return -1;
}

static i2c_bus_context_t* get_bus(I2C_TypeDef* i2c){
    int8_t index = get_bus_index(i2c);
    return index < 0 ? NULL : &buses[index];
}

static volatile i2c_manager_diagnostics_t* get_diagnostics(I2C_TypeDef* i2c){
    int8_t index = get_bus_index(i2c);
    return index < 0 ? NULL : &g_i2c_manager_diagnostics[index];
}

void i2c_manager_assign_bus(I2C_TypeDef* i2c, osMutexId_t mutex,
        osSemaphoreId_t semaphore){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL) return;
    bus->i2c = i2c;
    bus->mutex = mutex;
    bus->completion_semaphore = semaphore;
    bus->state = I2C_DMA_IDLE;
    bus->result = _i2c_manager_ok;
}

void i2c_manager_set_dispatch_notification(I2C_TypeDef* i2c,
        osThreadId_t dispatcher, uint32_t completion_flag){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL) return;
    bus->dispatcher = dispatcher;
    bus->dispatcher_flag = completion_flag;
}

void i2c_manager_set_job_service(osMessageQueueId_t queue,
        osThreadId_t dispatcher, uint32_t new_job_flag){
    job_service.queue = queue;
    job_service.dispatcher = dispatcher;
    job_service.new_job_flag = new_job_flag;
}

void i2c_manager_set_bus_recovery(I2C_TypeDef* i2c,
        i2c_manager_bus_recovery_t recovery){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL) return;
    bus->recovery = recovery;
}

static uint32_t get_tick(void){
    return osKernelGetTickCount();
}

typedef bool (*i2c_condition_t)(I2C_TypeDef* i2c);

static bool bus_is_free(I2C_TypeDef* i2c){ return !LL_I2C_IsActiveFlag_BUSY(i2c); }
static bool start_was_sent(I2C_TypeDef* i2c){ return LL_I2C_IsActiveFlag_SB(i2c); }
static bool transmit_empty(I2C_TypeDef* i2c){ return LL_I2C_IsActiveFlag_TXE(i2c); }
static bool receive_not_empty(I2C_TypeDef* i2c){ return LL_I2C_IsActiveFlag_RXNE(i2c); }
static bool byte_transfer_finished(I2C_TypeDef* i2c){ return LL_I2C_IsActiveFlag_BTF(i2c); }
static bool stop_was_completed(I2C_TypeDef* i2c){
    return (i2c->CR1 & I2C_CR1_STOP) == 0 &&
            !LL_I2C_IsActiveFlag_BUSY(i2c);
}
static bool address_or_nack(I2C_TypeDef* i2c){
    return LL_I2C_IsActiveFlag_ADDR(i2c) || LL_I2C_IsActiveFlag_AF(i2c);
}

static i2c_manager_return_status active_hardware_error(I2C_TypeDef* i2c){
    if(LL_I2C_IsActiveFlag_AF(i2c)) return _i2c_manager_nack;
    if(LL_I2C_IsActiveFlag_BERR(i2c)){
        volatile i2c_manager_diagnostics_t* diagnostics =
                get_diagnostics(i2c);
        bool controlled_final_stop = diagnostics != NULL &&
                diagnostics->phase == I2C_MANAGER_PHASE_DATA &&
                LL_I2C_IsActiveFlag_MSL(i2c) &&
                (i2c->CR1 & I2C_CR1_STOP) != 0;
        if(controlled_final_stop){
            /* Controller modunda BERR mevcut transferi durdurmaz. Bu hata
             * yalnizca bizim son-byte STOP penceremizde gorulurse temizlenip
             * RXNE beklenmeye devam edilir. Diger fazlarda gercek hatadir. */
            LL_I2C_ClearFlag_BERR(i2c);
            diagnostics->controlled_stop_berr_count++;
        }
        else{
            return _i2c_manager_bus_error;
        }
    }
    if(LL_I2C_IsActiveFlag_ARLO(i2c)) return _i2c_manager_arbitration_lost;
    if(LL_I2C_IsActiveFlag_OVR(i2c)) return _i2c_manager_overrun;
    return _i2c_manager_ok;
}

static void capture_wait_failure(I2C_TypeDef* i2c,
        i2c_manager_return_status result){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics == NULL) return;

    diagnostics->last_wait_result = result;
    diagnostics->last_wait_phase = diagnostics->phase;
    diagnostics->last_wait_sr1 = i2c->SR1;
    diagnostics->last_wait_sr2 = i2c->SR2;
    diagnostics->last_wait_cr1 = i2c->CR1;
    diagnostics->last_wait_cr2 = i2c->CR2;
}

static i2c_manager_return_status wait_for(I2C_TypeDef* i2c,
        i2c_condition_t condition){
    uint32_t start = get_tick();
    uint32_t spins = 0;
    while(!condition(i2c)){
        i2c_manager_return_status error = active_hardware_error(i2c);
        if(error != _i2c_manager_ok){
            capture_wait_failure(i2c, error);
            return error;
        }
        if((get_tick() - start) >= i2c_manager_timeout_ms){
            /* Bayrak, dongu kosulu ile timeout kontrolu arasinda degisebilir.
             * Timeout demeden once son kez tekrar kontrol et. */
            if(condition(i2c)) return _i2c_manager_ok;
            capture_wait_failure(i2c, _i2c_manager_timeout);
            return _i2c_manager_timeout;
        }
        if(++spins >= 4096){
            spins = 0;
            osDelay(1);
        }
    }
    return _i2c_manager_ok;
}

static void clear_hardware_errors(I2C_TypeDef* i2c){
    if(LL_I2C_IsActiveFlag_AF(i2c)) LL_I2C_ClearFlag_AF(i2c);
    if(LL_I2C_IsActiveFlag_BERR(i2c)) LL_I2C_ClearFlag_BERR(i2c);
    if(LL_I2C_IsActiveFlag_ARLO(i2c)) LL_I2C_ClearFlag_ARLO(i2c);
    if(LL_I2C_IsActiveFlag_OVR(i2c)) LL_I2C_ClearFlag_OVR(i2c);
}

static void set_phase(I2C_TypeDef* i2c, i2c_manager_phase_t phase){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL) diagnostics->phase = phase;
}

static void capture_failure(I2C_TypeDef* i2c,
        i2c_manager_return_status result){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics == NULL) return;

    diagnostics->last_error_phase = diagnostics->phase;
    diagnostics->last_result = result;
    diagnostics->last_sr1 = i2c->SR1;
    diagnostics->last_sr2 = i2c->SR2;
    diagnostics->last_cr1 = i2c->CR1;
    diagnostics->last_cr2 = i2c->CR2;
    diagnostics->last_owner_task = diagnostics->owner_task;
    diagnostics->last_owner_name = diagnostics->owner_name;
    if(result == _i2c_manager_timeout) diagnostics->timeout_count++;
    else if(result == _i2c_manager_nack) diagnostics->nack_count++;
    else diagnostics->hardware_error_count++;
}

static void begin_transaction(I2C_TypeDef* i2c,
        i2c_manager_operation_t operation, uint8_t device_address,
        uint8_t register_address){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics == NULL) return;

    diagnostics->transaction_count++;
    osThreadId_t current_task = osThreadGetId();
    diagnostics->owner_task = (uintptr_t)current_task;
    diagnostics->owner_name = osThreadGetName(current_task);
    diagnostics->device_address = device_address;
    diagnostics->register_address = register_address;
    diagnostics->operation = operation;
    diagnostics->phase = I2C_MANAGER_PHASE_MUTEX_ACQUIRED;
    diagnostics->bus_free_after_cleanup = false;
}

static i2c_manager_return_status acquire_bus(i2c_bus_context_t* bus,
        I2C_TypeDef* i2c, uint32_t timeout,
        i2c_manager_operation_t operation, uint8_t device_address,
        uint8_t register_address){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL && osMutexGetOwner(bus->mutex) != NULL){
        diagnostics->mutex_wait_count++;
    }
    if(osMutexAcquire(bus->mutex, timeout) != osOK){
        return _i2c_manager_busy;
    }
    begin_transaction(i2c, operation, device_address, register_address);
    return _i2c_manager_ok;
}

static void restore_receive_defaults(I2C_TypeDef* i2c){
    LL_I2C_DisableBitPOS(i2c);
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
}

static uint32_t enter_short_critical_section(void){
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    return interrupt_state;
}

static void leave_short_critical_section(uint32_t interrupt_state){
    if(interrupt_state == 0) __enable_irq();
}

static bool wait_for_bus_release(I2C_TypeDef* i2c){
    uint32_t start = get_tick();
    uint32_t spins = 0;
    while(!stop_was_completed(i2c)){
        if((get_tick() - start) >= i2c_manager_timeout_ms) return false;
        if(++spins >= 4096){
            spins = 0;
            osDelay(1);
        }
    }
    return true;
}

static void software_reset_peripheral(I2C_TypeDef* i2c){
    uint32_t saved_cr1 = i2c->CR1;
    uint32_t saved_cr2 = i2c->CR2;
    uint32_t saved_oar1 = i2c->OAR1;
    uint32_t saved_oar2 = i2c->OAR2;
    uint32_t saved_ccr = i2c->CCR;
    uint32_t saved_trise = i2c->TRISE;

    LL_I2C_Disable(i2c);
    LL_I2C_EnableReset(i2c);
    __DSB();
    LL_I2C_DisableReset(i2c);

    i2c->CR2 = saved_cr2 & I2C_CR2_FREQ;
    i2c->OAR1 = saved_oar1;
    i2c->OAR2 = saved_oar2;
    i2c->CCR = saved_ccr;
    i2c->TRISE = saved_trise;
    i2c->CR1 = saved_cr1 & (I2C_CR1_ENGC | I2C_CR1_NOSTRETCH);
    restore_receive_defaults(i2c);
    LL_I2C_Enable(i2c);
}

static void drain_receive_register(I2C_TypeDef* i2c){
    /* F4 I2C'de DR ve shift register ayni anda dolu olabilir. Iki okuma,
     * hata yolunda bekleyen en fazla iki byte'i guvenli sekilde bosaltir. */
    for(uint8_t read_count = 0; read_count < 2; read_count++){
        if(!LL_I2C_IsActiveFlag_RXNE(i2c)) break;
        volatile uint8_t discarded = LL_I2C_ReceiveData8(i2c);
        (void)discarded;
    }
}

static bool run_physical_bus_recovery(i2c_bus_context_t* bus,
        I2C_TypeDef* i2c){
    if(bus == NULL || bus->recovery == NULL) return false;

    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL) diagnostics->physical_recovery_count++;

    bool recovered = bus->recovery(i2c) && bus_is_free(i2c);
    if(diagnostics != NULL && recovered){
        diagnostics->physical_recovery_success_count++;
    }
    return recovered;
}

static bool recover_stuck_bus(i2c_bus_context_t* bus, I2C_TypeDef* i2c){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_RECOVERY);
    software_reset_peripheral(i2c);
    if(diagnostics != NULL) diagnostics->software_reset_count++;

    bool recovered = bus_is_free(i2c);
    if(!recovered) recovered = run_physical_bus_recovery(bus, i2c);
    restore_receive_defaults(i2c);

    if(diagnostics != NULL){
        diagnostics->bus_free_after_cleanup = recovered && bus_is_free(i2c);
        if(diagnostics->bus_free_after_cleanup){
            diagnostics->recovery_retry_count++;
        }
        else{
            diagnostics->recovery_failed_count++;
        }
    }
    return recovered && bus_is_free(i2c);
}

static void release_bus(i2c_bus_context_t* bus, I2C_TypeDef* i2c,
        i2c_manager_return_status result){
    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL){
        diagnostics->last_result = result;
        if(result == _i2c_manager_ok) diagnostics->completed_count++;
        diagnostics->owner_task = 0;
        diagnostics->owner_name = NULL;
        diagnostics->operation = I2C_MANAGER_OPERATION_NONE;
        diagnostics->phase = I2C_MANAGER_PHASE_IDLE;
    }
    osMutexRelease(bus->mutex);
}

static i2c_manager_return_status finish_polling_success(
        I2C_TypeDef* i2c, i2c_bus_context_t* bus,
        i2c_manager_return_status result){
    set_phase(i2c, I2C_MANAGER_PHASE_STOP);
    bool bus_released = wait_for_bus_release(i2c);
    restore_receive_defaults(i2c);
    if(!bus_released){
        volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
        if(diagnostics != NULL) diagnostics->stop_wait_timeout_count++;
        capture_failure(i2c, _i2c_manager_timeout);
        set_phase(i2c, I2C_MANAGER_PHASE_RECOVERY);
        software_reset_peripheral(i2c);
        if(diagnostics != NULL) diagnostics->software_reset_count++;
        bool recovered = bus_is_free(i2c);
        if(!recovered) recovered = run_physical_bus_recovery(bus, i2c);
        if(diagnostics != NULL){
            diagnostics->bus_free_after_cleanup = recovered;
            if(!recovered) diagnostics->recovery_failed_count++;
        }
        release_bus(bus, i2c, _i2c_manager_timeout);
        return _i2c_manager_timeout;
    }

    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL) diagnostics->bus_free_after_cleanup = true;
    release_bus(bus, i2c, result);
    return result;
}

static i2c_manager_return_status polling_failure_cleanup(I2C_TypeDef* i2c,
        i2c_bus_context_t* bus, i2c_manager_return_status result){
    bool address_active = LL_I2C_IsActiveFlag_ADDR(i2c);
    bool was_master = LL_I2C_IsActiveFlag_MSL(i2c);
    capture_failure(i2c, result);

    set_phase(i2c, I2C_MANAGER_PHASE_STOP);
    /* Tek byte receive veya yarim kalmis ADDR fazinda once sonraki byte'a
     * NACK hazirlanir. Master isek STOP istenir; adres NACK'indeki AF ise
     * ST'nin HAL surucusundeki gibi STOP isteginden sonra temizlenir. */
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
    if(was_master){
        uint32_t interrupt_state = enter_short_critical_section();
        if(address_active) LL_I2C_ClearFlag_ADDR(i2c);
        LL_I2C_GenerateStopCondition(i2c);
        leave_short_critical_section(interrupt_state);
    }

    if(LL_I2C_IsActiveFlag_AF(i2c)) LL_I2C_ClearFlag_AF(i2c);
    if(LL_I2C_IsActiveFlag_BERR(i2c)) LL_I2C_ClearFlag_BERR(i2c);
    if(LL_I2C_IsActiveFlag_ARLO(i2c)) LL_I2C_ClearFlag_ARLO(i2c);
    if(LL_I2C_IsActiveFlag_OVR(i2c)) LL_I2C_ClearFlag_OVR(i2c);
    drain_receive_register(i2c);

    bool bus_released = bus_is_free(i2c);
    if(!bus_released && was_master){
        bus_released = wait_for_bus_release(i2c);
    }
    if(!bus_released){
        volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
        if(diagnostics != NULL) diagnostics->stop_wait_timeout_count++;
        set_phase(i2c, I2C_MANAGER_PHASE_RECOVERY);
        software_reset_peripheral(i2c);
        if(diagnostics != NULL) diagnostics->software_reset_count++;
        bus_released = bus_is_free(i2c);
        if(!bus_released){
            bus_released = run_physical_bus_recovery(bus, i2c);
        }
    }

    restore_receive_defaults(i2c);

    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL){
        diagnostics->bus_free_after_cleanup = bus_released && bus_is_free(i2c);
        if(!diagnostics->bus_free_after_cleanup){
            diagnostics->recovery_failed_count++;
        }
    }
    release_bus(bus, i2c, result);
    return result;
}

static i2c_manager_return_status read_poll_direct(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address, uint8_t* data,
        uint16_t size){
    if(size == 0 || data == NULL) return _i2c_manager_size_0;
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL) return _i2c_manager_uninited_struct;
    i2c_manager_return_status status = acquire_bus(bus, i2c, osWaitForever,
            I2C_MANAGER_OPERATION_READ_POLL, device_address, register_address);
    if(status != _i2c_manager_ok) return status;
    clear_hardware_errors(i2c);
    restore_receive_defaults(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_WAIT_BUS_FREE);
    status = wait_for(i2c, bus_is_free);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_START);
    LL_I2C_GenerateStartCondition(i2c);
    status = wait_for(i2c, start_was_sent);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_WRITE_ADDRESS);
    LL_I2C_TransmitData8(i2c, device_address & 0xFE);
    status = wait_for(i2c, address_or_nack);
    if(status != _i2c_manager_ok) goto failure;
    if(LL_I2C_IsActiveFlag_AF(i2c)){
        status = _i2c_manager_nack;
        goto failure;
    }
    LL_I2C_ClearFlag_ADDR(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_REGISTER_ADDRESS);
    LL_I2C_TransmitData8(i2c, register_address);
    status = wait_for(i2c, byte_transfer_finished);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_RESTART);
    LL_I2C_GenerateStartCondition(i2c);
    status = wait_for(i2c, start_was_sent);
    if(status != _i2c_manager_ok) goto failure;

    if(size == 2) LL_I2C_EnableBitPOS(i2c);
    set_phase(i2c, I2C_MANAGER_PHASE_READ_ADDRESS);
    LL_I2C_TransmitData8(i2c, device_address | 0x01);
    status = wait_for(i2c, address_or_nack);
    if(status != _i2c_manager_ok) goto failure;
    if(LL_I2C_IsActiveFlag_AF(i2c)){
        status = _i2c_manager_nack;
        goto failure;
    }

    set_phase(i2c, I2C_MANAGER_PHASE_DATA);
    if(size == 1){
        LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
        uint32_t interrupt_state = enter_short_critical_section();
        LL_I2C_ClearFlag_ADDR(i2c);
        LL_I2C_GenerateStopCondition(i2c);
        leave_short_critical_section(interrupt_state);

        status = wait_for(i2c, receive_not_empty);
        if(status != _i2c_manager_ok) goto failure;
        *data = LL_I2C_ReceiveData8(i2c);
    }
    else if(size == 2){
        uint32_t interrupt_state = enter_short_critical_section();
        LL_I2C_ClearFlag_ADDR(i2c);
        LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
        leave_short_critical_section(interrupt_state);

        status = wait_for(i2c, byte_transfer_finished);
        if(status != _i2c_manager_ok) goto failure;

        interrupt_state = enter_short_critical_section();
        LL_I2C_GenerateStopCondition(i2c);
        *data++ = LL_I2C_ReceiveData8(i2c);
        leave_short_critical_section(interrupt_state);
        *data = LL_I2C_ReceiveData8(i2c);
    }
    else{
        LL_I2C_ClearFlag_ADDR(i2c);
        while(size > 3){
            status = wait_for(i2c, receive_not_empty);
            if(status != _i2c_manager_ok) goto failure;
            *data++ = LL_I2C_ReceiveData8(i2c);
            size--;
        }

        status = wait_for(i2c, byte_transfer_finished);
        if(status != _i2c_manager_ok) goto failure;
        LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
        *data++ = LL_I2C_ReceiveData8(i2c);

        status = wait_for(i2c, byte_transfer_finished);
        if(status != _i2c_manager_ok) goto failure;
        uint32_t interrupt_state = enter_short_critical_section();
        LL_I2C_GenerateStopCondition(i2c);
        *data++ = LL_I2C_ReceiveData8(i2c);
        leave_short_critical_section(interrupt_state);
        *data = LL_I2C_ReceiveData8(i2c);
    }

    return finish_polling_success(i2c, bus, _i2c_manager_ok);

failure:
    return polling_failure_cleanup(i2c, bus, status);
}

static i2c_manager_return_status write_poll_direct(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address, uint8_t* data,
        uint16_t size){
    if(size == 0 || data == NULL) return _i2c_manager_size_0;
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL) return _i2c_manager_uninited_struct;
    i2c_manager_return_status status = acquire_bus(bus, i2c, osWaitForever,
            I2C_MANAGER_OPERATION_WRITE_POLL, device_address, register_address);
    if(status != _i2c_manager_ok) return status;
    clear_hardware_errors(i2c);
    restore_receive_defaults(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_WAIT_BUS_FREE);
    status = wait_for(i2c, bus_is_free);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_START);
    LL_I2C_GenerateStartCondition(i2c);
    status = wait_for(i2c, start_was_sent);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_WRITE_ADDRESS);
    LL_I2C_TransmitData8(i2c, device_address & 0xFE);
    status = wait_for(i2c, address_or_nack);
    if(status != _i2c_manager_ok) goto failure;
    if(LL_I2C_IsActiveFlag_AF(i2c)){
        status = _i2c_manager_nack;
        goto failure;
    }
    LL_I2C_ClearFlag_ADDR(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_REGISTER_ADDRESS);
    LL_I2C_TransmitData8(i2c, register_address);
    status = wait_for(i2c, transmit_empty);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_DATA);
    while(size > 0){
        LL_I2C_TransmitData8(i2c, *data++);
        size--;
        status = wait_for(i2c, transmit_empty);
        if(status != _i2c_manager_ok) goto failure;
    }
    status = wait_for(i2c, byte_transfer_finished);
    if(status != _i2c_manager_ok) goto failure;
    LL_I2C_GenerateStopCondition(i2c);
    return finish_polling_success(i2c, bus, _i2c_manager_ok);

failure:
    return polling_failure_cleanup(i2c, bus, status);
}

static i2c_manager_return_status check_device_direct(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address, uint8_t device_id){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL) return _i2c_manager_uninited_struct;
    i2c_manager_return_status status = acquire_bus(bus, i2c, osWaitForever,
            I2C_MANAGER_OPERATION_CHECK_DEVICE, device_address, register_address);
    if(status != _i2c_manager_ok) return status;
    clear_hardware_errors(i2c);
    restore_receive_defaults(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_WAIT_BUS_FREE);
    status = wait_for(i2c, bus_is_free);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_START);
    LL_I2C_GenerateStartCondition(i2c);
    status = wait_for(i2c, start_was_sent);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_WRITE_ADDRESS);
    LL_I2C_TransmitData8(i2c, device_address & 0xFE);
    status = wait_for(i2c, address_or_nack);
    if(status != _i2c_manager_ok) goto failure;
    if(LL_I2C_IsActiveFlag_AF(i2c)){
        status = _i2c_manager_nack;
        goto failure;
    }
    LL_I2C_ClearFlag_ADDR(i2c);

    set_phase(i2c, I2C_MANAGER_PHASE_REGISTER_ADDRESS);
    LL_I2C_TransmitData8(i2c, register_address);
    status = wait_for(i2c, byte_transfer_finished);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_RESTART);
    LL_I2C_GenerateStartCondition(i2c);
    status = wait_for(i2c, start_was_sent);
    if(status != _i2c_manager_ok) goto failure;

    set_phase(i2c, I2C_MANAGER_PHASE_READ_ADDRESS);
    LL_I2C_TransmitData8(i2c, device_address | 0x01);
    status = wait_for(i2c, address_or_nack);
    if(status != _i2c_manager_ok) goto failure;
    if(LL_I2C_IsActiveFlag_AF(i2c)){
        status = _i2c_manager_nack;
        goto failure;
    }

    set_phase(i2c, I2C_MANAGER_PHASE_DATA);
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
    uint32_t interrupt_state = enter_short_critical_section();
    LL_I2C_ClearFlag_ADDR(i2c);
    LL_I2C_GenerateStopCondition(i2c);
    leave_short_critical_section(interrupt_state);

    status = wait_for(i2c, receive_not_empty);
    if(status != _i2c_manager_ok) goto failure;
    uint8_t received_id = LL_I2C_ReceiveData8(i2c);
    status = received_id == device_id ? _i2c_manager_ok : _i2c_manager_fail;
    return finish_polling_success(i2c, bus, status);

failure:
    return polling_failure_cleanup(i2c, bus, status);
}

static i2c_manager_return_status submit_synchronous_job(i2c_job_t* job){
    if(job == NULL || job_service.queue == NULL ||
            job_service.dispatcher == NULL || job_service.new_job_flag == 0){
        return _i2c_manager_uninited_struct;
    }

    osThreadId_t caller = osThreadGetId();
    if(caller == NULL) return _i2c_manager_fail;

    volatile i2c_manager_return_status completion_result =
            _i2c_manager_busy;
    job->completion_result = &completion_result;
    job->notify_task = caller;
    job->notify_flag = I2C_MANAGER_SYNC_DONE_FLAG;
    job->error_flag = I2C_MANAGER_SYNC_DONE_FLAG;

    osThreadFlagsClear(I2C_MANAGER_SYNC_DONE_FLAG);
    if(osMessageQueuePut(job_service.queue, job, 0, osWaitForever) != osOK){
        return _i2c_manager_busy;
    }
    osThreadFlagsSet(job_service.dispatcher, job_service.new_job_flag);

    uint32_t flags = osThreadFlagsWait(I2C_MANAGER_SYNC_DONE_FLAG,
            osFlagsWaitAny, osWaitForever);
    if((flags & osFlagsError) != 0 ||
            (flags & I2C_MANAGER_SYNC_DONE_FLAG) == 0){
        return _i2c_manager_fail;
    }
    return completion_result;
}

i2c_manager_return_status i2c_manager_read_poll(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address, uint8_t* data,
        uint16_t size){
    if(job_service.dispatcher != NULL &&
            osThreadGetId() == job_service.dispatcher){
        return read_poll_direct(i2c, device_address, register_address,
                data, size);
    }

    i2c_job_t job = {0};
    job.operation = I2C_JOB_READ_POLL;
    job.i2c_handle = i2c;
    job.dev_addr = device_address;
    job.reg_addr = register_address;
    job.rxdata = data;
    job.size = size;
    return submit_synchronous_job(&job);
}

i2c_manager_return_status i2c_manager_write_poll(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address, uint8_t* data,
        uint16_t size){
    if(job_service.dispatcher != NULL &&
            osThreadGetId() == job_service.dispatcher){
        return write_poll_direct(i2c, device_address, register_address,
                data, size);
    }

    i2c_job_t job = {0};
    job.operation = I2C_JOB_WRITE_POLL;
    job.i2c_handle = i2c;
    job.dev_addr = device_address;
    job.reg_addr = register_address;
    job.txdata = data;
    job.size = size;
    return submit_synchronous_job(&job);
}

i2c_manager_return_status i2c_manager_check_device(I2C_TypeDef* i2c,
        uint8_t device_address, uint8_t register_address,
        uint8_t device_id){
    if(job_service.dispatcher != NULL &&
            osThreadGetId() == job_service.dispatcher){
        return check_device_direct(i2c, device_address, register_address,
                device_id);
    }

    i2c_job_t job = {0};
    job.operation = I2C_JOB_CHECK_DEVICE;
    job.i2c_handle = i2c;
    job.dev_addr = device_address;
    job.reg_addr = register_address;
    job.expected_device_id = device_id;
    return submit_synchronous_job(&job);
}

i2c_manager_return_status i2c_manager_execute_polling_job(i2c_job_t* job){
    if(job == NULL) return _i2c_manager_fail;
    if(job_service.dispatcher == NULL ||
            osThreadGetId() != job_service.dispatcher){
        return _i2c_manager_fail;
    }

    switch(job->operation){
    case I2C_JOB_READ_POLL:
        return read_poll_direct(job->i2c_handle, job->dev_addr,
                job->reg_addr, job->rxdata, job->size);
    case I2C_JOB_WRITE_POLL:
        return write_poll_direct(job->i2c_handle, job->dev_addr,
                job->reg_addr, job->txdata, job->size);
    case I2C_JOB_CHECK_DEVICE:{
        i2c_manager_return_status result = check_device_direct(
                job->i2c_handle, job->dev_addr, job->reg_addr,
                job->expected_device_id);
        bool retryable = result == _i2c_manager_timeout ||
                result == _i2c_manager_bus_error ||
                result == _i2c_manager_arbitration_lost ||
                result == _i2c_manager_overrun;
        volatile i2c_manager_diagnostics_t* diagnostics =
                get_diagnostics(job->i2c_handle);
        if(retryable && diagnostics != NULL &&
                diagnostics->bus_free_after_cleanup){
            diagnostics->recovery_retry_count++;
            result = check_device_direct(job->i2c_handle, job->dev_addr,
                    job->reg_addr, job->expected_device_id);
        }
        return result;
    }
    case I2C_JOB_READ_DMA:
    default:
        return _i2c_manager_fail;
    }
}

static void clear_dma_flags(DMA_TypeDef* dma, uint32_t stream){
    switch(stream){
    case LL_DMA_STREAM_0: LL_DMA_ClearFlag_TC0(dma); LL_DMA_ClearFlag_HT0(dma); LL_DMA_ClearFlag_TE0(dma); LL_DMA_ClearFlag_DME0(dma); LL_DMA_ClearFlag_FE0(dma); break;
    case LL_DMA_STREAM_1: LL_DMA_ClearFlag_TC1(dma); LL_DMA_ClearFlag_HT1(dma); LL_DMA_ClearFlag_TE1(dma); LL_DMA_ClearFlag_DME1(dma); LL_DMA_ClearFlag_FE1(dma); break;
    case LL_DMA_STREAM_2: LL_DMA_ClearFlag_TC2(dma); LL_DMA_ClearFlag_HT2(dma); LL_DMA_ClearFlag_TE2(dma); LL_DMA_ClearFlag_DME2(dma); LL_DMA_ClearFlag_FE2(dma); break;
    case LL_DMA_STREAM_3: LL_DMA_ClearFlag_TC3(dma); LL_DMA_ClearFlag_HT3(dma); LL_DMA_ClearFlag_TE3(dma); LL_DMA_ClearFlag_DME3(dma); LL_DMA_ClearFlag_FE3(dma); break;
    case LL_DMA_STREAM_4: LL_DMA_ClearFlag_TC4(dma); LL_DMA_ClearFlag_HT4(dma); LL_DMA_ClearFlag_TE4(dma); LL_DMA_ClearFlag_DME4(dma); LL_DMA_ClearFlag_FE4(dma); break;
    case LL_DMA_STREAM_5: LL_DMA_ClearFlag_TC5(dma); LL_DMA_ClearFlag_HT5(dma); LL_DMA_ClearFlag_TE5(dma); LL_DMA_ClearFlag_DME5(dma); LL_DMA_ClearFlag_FE5(dma); break;
    case LL_DMA_STREAM_6: LL_DMA_ClearFlag_TC6(dma); LL_DMA_ClearFlag_HT6(dma); LL_DMA_ClearFlag_TE6(dma); LL_DMA_ClearFlag_DME6(dma); LL_DMA_ClearFlag_FE6(dma); break;
    case LL_DMA_STREAM_7: LL_DMA_ClearFlag_TC7(dma); LL_DMA_ClearFlag_HT7(dma); LL_DMA_ClearFlag_TE7(dma); LL_DMA_ClearFlag_DME7(dma); LL_DMA_ClearFlag_FE7(dma); break;
    default: break;
    }
}

static void dma_cleanup(i2c_bus_context_t* bus){
    LL_I2C_DisableIT_EVT(bus->i2c);
    LL_I2C_DisableIT_BUF(bus->i2c);
    LL_I2C_DisableIT_ERR(bus->i2c);
    LL_I2C_DisableDMAReq_RX(bus->i2c);
    LL_I2C_DisableLastDMA(bus->i2c);
    LL_DMA_DisableIT_TC(bus->dma, bus->dma_stream);
    LL_DMA_DisableIT_TE(bus->dma, bus->dma_stream);
    LL_DMA_DisableStream(bus->dma, bus->dma_stream);
}

static void notify_dispatcher(i2c_bus_context_t* bus){
    if(bus->completion_semaphore != NULL){
        osSemaphoreRelease(bus->completion_semaphore);
    }
    if(bus->dispatcher != NULL && bus->dispatcher_flag != 0){
        osThreadFlagsSet(bus->dispatcher, bus->dispatcher_flag);
    }
}

static void finish_from_isr(i2c_bus_context_t* bus,
        i2c_manager_return_status result){
    set_phase(bus->i2c, I2C_MANAGER_PHASE_STOP);
    LL_I2C_GenerateStopCondition(bus->i2c);
    dma_cleanup(bus);
    bus->result = result;
    volatile i2c_manager_diagnostics_t* diagnostics =
            get_diagnostics(bus->i2c);
    if(diagnostics != NULL) diagnostics->last_result = result;
    bus->state = I2C_DMA_IDLE;
    notify_dispatcher(bus);
}

i2c_manager_return_status i2c_manager_read_dma(DMA_TypeDef* dma,
        uint32_t stream, I2C_TypeDef* i2c, uint8_t device_address,
        uint8_t register_address, uint8_t* data, uint16_t size){
    if(job_service.dispatcher != NULL &&
            osThreadGetId() != job_service.dispatcher){
        return _i2c_manager_fail;
    }
    if(size == 0 || data == NULL) return _i2c_manager_size_0;
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL) return _i2c_manager_uninited_struct;
    i2c_manager_return_status lock_status = acquire_bus(bus, i2c, 0,
            I2C_MANAGER_OPERATION_READ_DMA, device_address, register_address);
    if(lock_status != _i2c_manager_ok) return lock_status;
    if(LL_I2C_IsActiveFlag_BUSY(i2c)){
        /* BUSY burada mutex cakismasi degildir: mutex artik bu task'tadir.
         * Hat onceki transferden kilitli kalmistir. Ayni isi sonsuza kadar
         * kuyruga geri koymak yerine bir kez bus recovery uygula. */
        capture_failure(i2c, _i2c_manager_busy);
        if(!recover_stuck_bus(bus, i2c)){
            release_bus(bus, i2c, _i2c_manager_bus_error);
            return _i2c_manager_bus_error;
        }
    }
    clear_hardware_errors(i2c);
    restore_receive_defaults(i2c);

    LL_I2C_DisableIT_EVT(i2c);
    LL_I2C_DisableIT_BUF(i2c);
    LL_I2C_DisableIT_ERR(i2c);
    LL_I2C_DisableDMAReq_RX(i2c);
    LL_I2C_DisableLastDMA(i2c);
    LL_DMA_DisableIT_TC(dma, stream);
    LL_DMA_DisableIT_TE(dma, stream);
    LL_DMA_DisableStream(dma, stream);
    clear_dma_flags(dma, stream);

    bus->dma = dma;
    bus->dma_stream = stream;
    bus->device_address = device_address;
    bus->register_address = register_address;
    bus->receive_buffer = data;
    bus->size = size;
    bus->result = _i2c_manager_busy;
    bus->state = I2C_DMA_START_SENT;

    /* Her RX transferinde DMA'nin temel kurallarini yeniden kur. Bu ayarlardan
     * ozellikle MEMORY_INCREMENT kapali kalirsa butun byte'lar data[0]'in
     * ustune yazilir ve data[1..size-1] eski degerlerinde kalir. */
    LL_DMA_SetDataTransferDirection(dma, stream, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(dma, stream, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(dma, stream, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(dma, stream, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(dma, stream, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(dma, stream, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_ConfigAddresses(dma, stream, LL_I2C_DMA_GetRegAddr(i2c),
            (uint32_t)data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(dma, stream, size);
    LL_I2C_EnableIT_EVT(i2c);
    LL_I2C_EnableIT_BUF(i2c);
    LL_I2C_EnableIT_ERR(i2c);
    set_phase(i2c, I2C_MANAGER_PHASE_START);
    LL_I2C_GenerateStartCondition(i2c);
    return _i2c_manager_ok;
}

void i2c_manager_event_handler(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->state == I2C_DMA_IDLE) return;

    switch(bus->state){
    case I2C_DMA_START_SENT:
        if(LL_I2C_IsActiveFlag_SB(i2c)){
            set_phase(i2c, I2C_MANAGER_PHASE_WRITE_ADDRESS);
            LL_I2C_TransmitData8(i2c, bus->device_address & 0xFE);
            bus->state = I2C_DMA_WRITE_ADDRESS_SENT;
        }
        break;
    case I2C_DMA_WRITE_ADDRESS_SENT:
        if(LL_I2C_IsActiveFlag_ADDR(i2c)){
            set_phase(i2c, I2C_MANAGER_PHASE_REGISTER_ADDRESS);
            LL_I2C_ClearFlag_ADDR(i2c);
            LL_I2C_TransmitData8(i2c, bus->register_address);
            /* TXE can stay asserted while the byte is shifting. Disable the
             * buffer interrupt and wait for the BTF event without an IRQ loop. */
            LL_I2C_DisableIT_BUF(i2c);
            bus->state = I2C_DMA_REGISTER_SENT;
        }
        break;
    case I2C_DMA_REGISTER_SENT:
        if(LL_I2C_IsActiveFlag_BTF(i2c)){
            set_phase(i2c, I2C_MANAGER_PHASE_RESTART);
            LL_I2C_GenerateStartCondition(i2c);
            bus->state = I2C_DMA_RESTART_SENT;
        }
        break;
    case I2C_DMA_RESTART_SENT:
        if(LL_I2C_IsActiveFlag_SB(i2c)){
            set_phase(i2c, I2C_MANAGER_PHASE_READ_ADDRESS);
            LL_I2C_TransmitData8(i2c, bus->device_address | 0x01);
            bus->state = I2C_DMA_READ_ADDRESS_SENT;
        }
        break;
    case I2C_DMA_READ_ADDRESS_SENT:
        if(LL_I2C_IsActiveFlag_ADDR(i2c)){
            set_phase(i2c, I2C_MANAGER_PHASE_DATA);
            if(bus->size == 1){
                LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
            }
            else{
                LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
                LL_I2C_EnableLastDMA(i2c);
            }
            LL_DMA_EnableIT_TC(bus->dma, bus->dma_stream);
            LL_DMA_EnableIT_TE(bus->dma, bus->dma_stream);
            LL_I2C_EnableDMAReq_RX(i2c);
            LL_DMA_EnableStream(bus->dma, bus->dma_stream);
            bus->state = I2C_DMA_READING;
            LL_I2C_ClearFlag_ADDR(i2c);
            LL_I2C_DisableIT_EVT(i2c);
            LL_I2C_DisableIT_BUF(i2c);
        }
        break;
    case I2C_DMA_READING:
    case I2C_DMA_IDLE:
    default:
        break;
    }
}

static void read_dma_flags(DMA_TypeDef* dma, uint32_t stream,
        bool* complete, bool* error){
    switch(stream){
    case LL_DMA_STREAM_0: *complete = LL_DMA_IsActiveFlag_TC0(dma); *error = LL_DMA_IsActiveFlag_TE0(dma); break;
    case LL_DMA_STREAM_1: *complete = LL_DMA_IsActiveFlag_TC1(dma); *error = LL_DMA_IsActiveFlag_TE1(dma); break;
    case LL_DMA_STREAM_2: *complete = LL_DMA_IsActiveFlag_TC2(dma); *error = LL_DMA_IsActiveFlag_TE2(dma); break;
    case LL_DMA_STREAM_3: *complete = LL_DMA_IsActiveFlag_TC3(dma); *error = LL_DMA_IsActiveFlag_TE3(dma); break;
    case LL_DMA_STREAM_4: *complete = LL_DMA_IsActiveFlag_TC4(dma); *error = LL_DMA_IsActiveFlag_TE4(dma); break;
    case LL_DMA_STREAM_5: *complete = LL_DMA_IsActiveFlag_TC5(dma); *error = LL_DMA_IsActiveFlag_TE5(dma); break;
    case LL_DMA_STREAM_6: *complete = LL_DMA_IsActiveFlag_TC6(dma); *error = LL_DMA_IsActiveFlag_TE6(dma); break;
    case LL_DMA_STREAM_7: *complete = LL_DMA_IsActiveFlag_TC7(dma); *error = LL_DMA_IsActiveFlag_TE7(dma); break;
    default: break;
    }
}

void i2c_manager_dma_handler(DMA_TypeDef* dma, uint32_t stream){
    i2c_bus_context_t* bus = NULL;
    for(uint8_t index = 0; index < 3; index++){
        if(buses[index].state != I2C_DMA_IDLE && buses[index].dma == dma &&
                buses[index].dma_stream == stream){
            bus = &buses[index];
            break;
        }
    }
    if(bus == NULL) return;
    bool complete = false;
    bool error = false;
    read_dma_flags(dma, stream, &complete, &error);
    clear_dma_flags(dma, stream);
    if(error) finish_from_isr(bus, _i2c_manager_dma_error);
    else if(complete) finish_from_isr(bus, _i2c_manager_ok);
}

void i2c_manager_error_handler(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->state == I2C_DMA_IDLE) return;
    i2c_manager_return_status result = _i2c_manager_fail;
    if(LL_I2C_IsActiveFlag_AF(i2c)) result = _i2c_manager_nack;
    if(LL_I2C_IsActiveFlag_BERR(i2c)) result = _i2c_manager_bus_error;
    if(LL_I2C_IsActiveFlag_ARLO(i2c)) result = _i2c_manager_arbitration_lost;
    if(LL_I2C_IsActiveFlag_OVR(i2c)) result = _i2c_manager_overrun;
    capture_failure(i2c, result);
    clear_hardware_errors(i2c);
    finish_from_isr(bus, result);
}

bool i2c_manager_is_dma_busy(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    return bus == NULL || bus->state != I2C_DMA_IDLE;
}

bool i2c_manager_last_transfer_succeeded(I2C_TypeDef* i2c){
    return i2c_manager_last_result(i2c) == _i2c_manager_ok;
}

i2c_manager_return_status i2c_manager_last_result(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL) return _i2c_manager_uninited_struct;
    return bus->result;
}

i2c_manager_return_status i2c_manager_unlock_bus(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL) return _i2c_manager_uninited_struct;
    if(bus->completion_semaphore != NULL){
        osSemaphoreAcquire(bus->completion_semaphore, 0);
    }

    i2c_manager_return_status result = bus->result;
    set_phase(i2c, I2C_MANAGER_PHASE_STOP);
    bool bus_released = wait_for_bus_release(i2c);
    restore_receive_defaults(i2c);
    if(!bus_released){
        volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
        if(diagnostics != NULL) diagnostics->stop_wait_timeout_count++;
        if(result == _i2c_manager_ok) capture_failure(i2c, _i2c_manager_timeout);
        recover_stuck_bus(bus, i2c);
        result = _i2c_manager_timeout;
        bus->result = result;
    }

    volatile i2c_manager_diagnostics_t* diagnostics = get_diagnostics(i2c);
    if(diagnostics != NULL){
        diagnostics->bus_free_after_cleanup = bus_is_free(i2c);
        if(!diagnostics->bus_free_after_cleanup){
            diagnostics->recovery_failed_count++;
        }
    }
    release_bus(bus, i2c, result);
    return result;
}

i2c_manager_return_status i2c_manager_recover_bus(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->mutex == NULL || bus->recovery == NULL){
        return _i2c_manager_uninited_struct;
    }

    /* Dispatcher aktif bir transferi tamamlarken veya temizlerken peripheral'i
     * yeniden init etme. Mutex alinabiliyorsa I2C manager ile recovery ayni
     * donanima ayni anda dokunmuyor demektir. */
    i2c_manager_return_status status = acquire_bus(bus, i2c, 0,
            I2C_MANAGER_OPERATION_NONE, 0, 0);
    if(status != _i2c_manager_ok) return status;

    if(bus->state != I2C_DMA_IDLE){
        release_bus(bus, i2c, _i2c_manager_busy);
        return _i2c_manager_busy;
    }

    set_phase(i2c, I2C_MANAGER_PHASE_RECOVERY);
    bool recovered = run_physical_bus_recovery(bus, i2c);
    clear_hardware_errors(i2c);
    restore_receive_defaults(i2c);

    status = recovered ? _i2c_manager_ok : _i2c_manager_bus_error;
    release_bus(bus, i2c, status);
    return status;
}

void i2c_manager_abort_transfer(I2C_TypeDef* i2c){
    i2c_bus_context_t* bus = get_bus(i2c);
    if(bus == NULL || bus->state == I2C_DMA_IDLE) return;
    LL_I2C_GenerateStopCondition(i2c);
    dma_cleanup(bus);
    bus->result = _i2c_manager_aborted;
    bus->state = I2C_DMA_IDLE;
    i2c_manager_unlock_bus(i2c);
}
