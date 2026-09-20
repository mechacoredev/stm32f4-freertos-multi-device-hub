/*
 * i2c_manager.h
 *
 *  Created on: Jul 20, 2026
 *      Author: Enes
 */

#ifndef INC_I2C_MANAGER_H_
#define INC_I2C_MANAGER_H_

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_dma.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum{
	_i2c_manager_ok = 0,
	_i2c_manager_fail,
	_i2c_manager_busy,
	_i2c_manager_timeout,
	_i2c_manager_size_0,
	_i2c_manager_uninited_struct,
	_i2c_manager_nack,
	_i2c_manager_bus_error,
	_i2c_manager_arbitration_lost,
	_i2c_manager_overrun,
	_i2c_manager_dma_error,
	_i2c_manager_aborted,
}i2c_manager_return_status;

typedef enum{
	I2C_JOB_READ_DMA = 0,
	I2C_JOB_WRITE_DMA,
	I2C_JOB_READ_POLL,
	I2C_JOB_WRITE_POLL,
	I2C_JOB_CHECK_DEVICE
}i2c_job_operation_t;

typedef struct{
	i2c_job_operation_t operation;
	I2C_TypeDef* i2c_handle;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
	uint8_t dev_addr;
	uint8_t reg_addr;
	uint8_t* rxdata;
	uint8_t* txdata;
	uint16_t size;
	uint8_t expected_device_id;
	volatile i2c_manager_return_status* completion_result;

	osThreadId_t notify_task;
	uint32_t notify_flag;
	uint32_t error_flag;
}i2c_job_t;

typedef enum{
	I2C_MANAGER_OPERATION_NONE = 0,
	I2C_MANAGER_OPERATION_READ_POLL,
	I2C_MANAGER_OPERATION_WRITE_POLL,
	I2C_MANAGER_OPERATION_CHECK_DEVICE,
	I2C_MANAGER_OPERATION_READ_DMA,
	I2C_MANAGER_OPERATION_WRITE_DMA
}i2c_manager_operation_t;

typedef enum{
	I2C_MANAGER_PHASE_IDLE = 0,
	I2C_MANAGER_PHASE_MUTEX_ACQUIRED,
	I2C_MANAGER_PHASE_WAIT_BUS_FREE,
	I2C_MANAGER_PHASE_START,
	I2C_MANAGER_PHASE_WRITE_ADDRESS,
	I2C_MANAGER_PHASE_REGISTER_ADDRESS,
	I2C_MANAGER_PHASE_RESTART,
	I2C_MANAGER_PHASE_READ_ADDRESS,
	I2C_MANAGER_PHASE_DATA,
	I2C_MANAGER_PHASE_STOP,
	I2C_MANAGER_PHASE_RECOVERY
}i2c_manager_phase_t;

typedef bool (*i2c_manager_bus_recovery_t)(I2C_TypeDef* i2c_handle);

/* Live Expressions'ta g_i2c_manager_diagnostics[0] I2C1'i gosterir.
 * last_sr1/last_sr2 bir hata veya timeout olustugu andaki kayittir. */
typedef struct{
	volatile uint32_t transaction_count;
	volatile uint32_t completed_count;
	volatile uint32_t mutex_wait_count;
	volatile uint32_t timeout_count;
	volatile uint32_t nack_count;
	volatile uint32_t hardware_error_count;
	volatile uint32_t bus_error_count;
	volatile uint32_t arbitration_lost_count;
	volatile uint32_t overrun_count;
	volatile uint32_t dma_error_count;
	volatile uint32_t controller_berr_ignored_count;
	volatile uint32_t controlled_stop_berr_count;
	volatile uint32_t stop_wait_timeout_count;
	volatile uint32_t software_reset_count;
	volatile uint32_t physical_recovery_count;
	volatile uint32_t physical_recovery_success_count;
	volatile uint32_t recovery_retry_count;
	volatile uint32_t recovery_failed_count;
	volatile uintptr_t owner_task;
	volatile uintptr_t last_owner_task;
	const char* volatile owner_name;
	const char* volatile last_owner_name;
	volatile uint8_t device_address;
	volatile uint8_t register_address;
	volatile i2c_manager_operation_t operation;
	volatile i2c_manager_phase_t phase;
	volatile i2c_manager_phase_t last_error_phase;
	volatile i2c_manager_return_status last_result;
	volatile i2c_manager_return_status last_wait_result;
	volatile i2c_manager_phase_t last_wait_phase;
	volatile uint32_t last_wait_sr1;
	volatile uint32_t last_wait_sr2;
	volatile uint32_t last_wait_cr1;
	volatile uint32_t last_wait_cr2;
	volatile uint32_t last_sr1;
	volatile uint32_t last_sr2;
	volatile uint32_t last_cr1;
	volatile uint32_t last_cr2;
	volatile uint32_t last_failure_tick;
	volatile bool bus_free_after_cleanup;
}i2c_manager_diagnostics_t;

extern volatile i2c_manager_diagnostics_t g_i2c_manager_diagnostics[3];

#define i2c_manager_timeout_ms 100
#define I2C_MANAGER_SYNC_DONE_FLAG 0x10000000

// Çoklu Veriyolu Kayıt Fonksiyonu (YENİ)
void i2c_manager_assign_bus(I2C_TypeDef* i2c_handle, osMutexId_t mutex, osSemaphoreId_t sem);
void i2c_manager_set_dispatch_notification(I2C_TypeDef* i2c_handle,
		osThreadId_t dispatch_task, uint32_t completion_flag);
void i2c_manager_set_job_service(osMessageQueueId_t queue,
		osThreadId_t dispatch_task, uint32_t new_job_flag);
void i2c_manager_set_bus_recovery(I2C_TypeDef* i2c_handle,
		i2c_manager_bus_recovery_t recovery);

i2c_manager_return_status i2c_manager_read_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);
i2c_manager_return_status i2c_manager_write_poll(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* txdata, uint16_t size);
i2c_manager_return_status i2c_manager_check_device(I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t device_id);
i2c_manager_return_status i2c_manager_execute_polling_job(i2c_job_t* job);
i2c_manager_return_status i2c_manager_read_dma(DMA_TypeDef* dma_handle, uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t* rxdata, uint16_t size);
i2c_manager_return_status i2c_manager_write_dma(DMA_TypeDef* dma_handle,
		uint32_t dma_stream, I2C_TypeDef* i2c_handle, uint8_t dev_addr,
		uint8_t control_or_register, uint8_t* txdata, uint16_t size);

// Artık Hangi Hat Olduğunu Sormamız Gerekiyor (YENİ)
bool i2c_manager_is_dma_busy(I2C_TypeDef* i2c_handle);
bool i2c_manager_last_transfer_succeeded(I2C_TypeDef* i2c_handle);
i2c_manager_return_status i2c_manager_last_result(I2C_TypeDef* i2c_handle);
i2c_manager_return_status i2c_manager_unlock_bus(I2C_TypeDef* i2c_handle);
i2c_manager_return_status i2c_manager_recover_bus(I2C_TypeDef* i2c_handle);
void i2c_manager_abort_transfer(I2C_TypeDef* i2c_handle);
void i2c_manager_dma_handler(DMA_TypeDef* dma_handle, uint32_t dma_stream);
void i2c_manager_event_handler(I2C_TypeDef* i2c_handle);
void i2c_manager_error_handler(I2C_TypeDef* i2c_handle);

#endif /* INC_I2C_MANAGER_H_ */
