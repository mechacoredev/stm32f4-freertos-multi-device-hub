/*
 * spi_manager.c
 *
 *  Created on: Jul 21, 2026
 *      Author: Enes
 */

#include "spi_manager.h"

typedef enum{
	SPI_DMA_STATE_IDLE = 0,
	SPI_DMA_STATE_BUSY,
}spi_dma_state_t;

struct spi_dma_management{
	SPI_TypeDef* spi_handle;
	osMutexId_t mutexHandle;
	osSemaphoreId_t semaphoreHandle;
	osThreadId_t dispatch_task;
	uint32_t completion_flag;

	DMA_TypeDef* dma_handle;
	uint32_t rx_stream;
	uint32_t tx_stream;

	GPIO_TypeDef* cs_port;
	uint32_t cs_pin;

	// Full-duplex'te TX ve RX ayrı DMA stream'leri olduğu için,
	// ikisinin de tamamlanmasını AYRI AYRI takip etmemiz lazım.
	// Hangisi önce biterse bitsin, gerçek bitiş ikisi de true olduğunda.
	volatile bool tx_done;
	volatile bool rx_done;
	volatile bool transfer_error;

	spi_dma_state_t state;
};

static struct spi_dma_management spi_jobs[3];

static struct spi_dma_management* get_job(SPI_TypeDef* spi_handle){
	if(spi_handle == SPI1) return &spi_jobs[0];
	if(spi_handle == SPI2) return &spi_jobs[1];
	if(spi_handle == SPI3) return &spi_jobs[2];
	return NULL;
}

void spi_manager_assign_bus(SPI_TypeDef* spi_handle, osMutexId_t mutex, osSemaphoreId_t sem){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job != NULL){
		job->spi_handle = spi_handle;
		job->mutexHandle = mutex;
		job->semaphoreHandle = sem;
		job->state = SPI_DMA_STATE_IDLE;
		LL_SPI_Enable(spi_handle);
	}
}

void spi_manager_set_dispatch_notification(SPI_TypeDef* spi_handle,
		osThreadId_t dispatch_task, uint32_t completion_flag){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL) return;
	job->dispatch_task = dispatch_task;
	job->completion_flag = completion_flag;
}

static uint32_t get_tick(void){
	return osKernelGetTickCount();
}

static uint8_t flag_txe(SPI_TypeDef* spi_handle){
	return LL_SPI_IsActiveFlag_TXE(spi_handle);
}

static uint8_t flag_rxne(SPI_TypeDef* spi_handle){
	return LL_SPI_IsActiveFlag_RXNE(spi_handle);
}

static uint8_t flag_not_busy(SPI_TypeDef* spi_handle){
	return !LL_SPI_IsActiveFlag_BSY(spi_handle);
}

typedef uint8_t (*spi_condition_t)(SPI_TypeDef* spi_handle);

static spi_manager_return_status wait_for_condition(SPI_TypeDef* spi_handle, spi_condition_t condition_met){
	uint32_t start_time = get_tick();
	uint32_t fast_spin_count = 0;
	while(condition_met(spi_handle) == 0){
		if((get_tick() - start_time) >= spi_manager_timeout_ms){
			return _spi_manager_timeout;
		}

		// Normal bir SPI byte'i mikrosaniyeler icinde tamamlanir. Her flag icin
		// 1 ms uyumak yerine once kisa bir aktif kontrol yap. Donanim gercekten
		// gecikirse taski uyutarak diger tasklara zaman birak.
		fast_spin_count++;
		if(fast_spin_count >= 4096){
			fast_spin_count = 0;
			osDelay(1);
		}
	}
	return _spi_manager_ok;
}

static inline void cs_assert(GPIO_TypeDef* cs_port, uint32_t cs_pin){
	LL_GPIO_ResetOutputPin(cs_port, cs_pin); // active-low: LOW = seçili
}

static inline void cs_deassert(GPIO_TypeDef* cs_port, uint32_t cs_pin){
	LL_GPIO_SetOutputPin(cs_port, cs_pin); // HIGH = serbest
}

spi_manager_return_status spi_manager_transfer_poll(SPI_TypeDef* spi_handle, GPIO_TypeDef* cs_port, uint32_t cs_pin, uint8_t* txdata, uint8_t* rxdata, uint16_t size){
	if(size == 0) return _spi_manager_size_0;

	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL || job->mutexHandle == NULL) return _spi_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, osWaitForever) != osOK) return _spi_manager_busy; // burada kilitleniyor

	cs_assert(cs_port, cs_pin);

	for(uint16_t i = 0; i < size; i++){
		if(wait_for_condition(spi_handle, flag_txe) != _spi_manager_ok) goto spi_error;
		LL_SPI_TransmitData8(spi_handle, txdata[i]);

		if(wait_for_condition(spi_handle, flag_rxne) != _spi_manager_ok) goto spi_error;
		rxdata[i] = LL_SPI_ReceiveData8(spi_handle);
	}

	// Son byte'ın gerçekten hatta çıktığından emin olmadan CS'i kaldırırsak
	// veri yarıda kesilebilir, o yüzden BSY bayrağını da bekliyoruz.
	if(wait_for_condition(spi_handle, flag_not_busy) != _spi_manager_ok) goto spi_error;

	cs_deassert(cs_port, cs_pin);
	osMutexRelease(job->mutexHandle);
	return _spi_manager_ok;

			spi_error:
			cs_deassert(cs_port, cs_pin);
			osMutexRelease(job->mutexHandle);
			return _spi_manager_timeout;
}

static void spi_clear_dma_flags(DMA_TypeDef* dma, uint32_t stream){
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

spi_manager_return_status spi_manager_transfer_dma(SPI_TypeDef* spi_handle, DMA_TypeDef* dma_handle, uint32_t rx_stream, uint32_t tx_stream, GPIO_TypeDef* cs_port, uint32_t cs_pin, uint8_t* txdata, uint8_t* rxdata, uint16_t size){
	if(size == 0) return _spi_manager_size_0;

	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL || job->mutexHandle == NULL) return _spi_manager_uninited_struct;

	if(osMutexAcquire(job->mutexHandle, 0) != osOK) return _spi_manager_busy;

	// Önceki kalıntı ayarları temizle
	LL_DMA_DisableStream(dma_handle, rx_stream);
	LL_DMA_DisableStream(dma_handle, tx_stream);
	LL_SPI_DisableDMAReq_RX(spi_handle);
	LL_SPI_DisableDMAReq_TX(spi_handle);
	spi_clear_dma_flags(dma_handle, rx_stream);
	spi_clear_dma_flags(dma_handle, tx_stream);

	job->dma_handle = dma_handle;
	job->rx_stream = rx_stream;
	job->tx_stream = tx_stream;
	job->cs_port = cs_port;
	job->cs_pin = cs_pin;
	job->tx_done = false;
	job->rx_done = false;
	job->transfer_error = false;
	job->state = SPI_DMA_STATE_BUSY;

	// RX stream: periferikten memory'e (gelen veri rxdata'ya yazılacak)
	LL_DMA_ConfigAddresses(dma_handle, rx_stream, LL_SPI_DMA_GetRegAddr(spi_handle), (uint32_t)rxdata, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(dma_handle, rx_stream, size);

	// TX stream: memory'den periferiğe (txdata gönderilecek, gerçek veya dummy)
	LL_DMA_ConfigAddresses(dma_handle, tx_stream, (uint32_t)txdata, LL_SPI_DMA_GetRegAddr(spi_handle), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(dma_handle, tx_stream, size);

	LL_DMA_EnableIT_TC(dma_handle, rx_stream);
	LL_DMA_EnableIT_TE(dma_handle, rx_stream);
	LL_DMA_EnableIT_TC(dma_handle, tx_stream);
	LL_DMA_EnableIT_TE(dma_handle, tx_stream);

	cs_assert(cs_port, cs_pin);

	// Full-duplex DMA'da RX'i TX'ten ÖNCE aç: aksi halde ilk gelen byte
	// RX DMA henüz hazır olmadan kaybolabilir.
	LL_DMA_EnableStream(dma_handle, rx_stream);
	LL_DMA_EnableStream(dma_handle, tx_stream);

	LL_SPI_EnableDMAReq_RX(spi_handle);
	LL_SPI_EnableDMAReq_TX(spi_handle);

	return _spi_manager_ok;
}

bool spi_manager_is_dma_busy(SPI_TypeDef* spi_handle){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL) return true;
	return (job->state != SPI_DMA_STATE_IDLE);
}

bool spi_manager_last_transfer_succeeded(SPI_TypeDef* spi_handle){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL) return false;
	return (job->state == SPI_DMA_STATE_IDLE) && (job->transfer_error == false);
}

static void spi_dma_cleanup(struct spi_dma_management* job){
	LL_SPI_DisableDMAReq_RX(job->spi_handle);
	LL_SPI_DisableDMAReq_TX(job->spi_handle);

	LL_DMA_DisableIT_TC(job->dma_handle, job->rx_stream);
	LL_DMA_DisableIT_TE(job->dma_handle, job->rx_stream);
	LL_DMA_DisableIT_TC(job->dma_handle, job->tx_stream);
	LL_DMA_DisableIT_TE(job->dma_handle, job->tx_stream);

	LL_DMA_DisableStream(job->dma_handle, job->rx_stream);
	LL_DMA_DisableStream(job->dma_handle, job->tx_stream);
}

static void finish_transfer_if_ready(struct spi_dma_management* job){
	// TX ve RX ikisi de tamamlanmadan CS'i kaldırma/işi bitirme!
	if(job->tx_done && job->rx_done){
		/* DMA TC means the last byte reached SPI_DR, not necessarily that the
		 * final bit left the shift register. Keep CS low until BSY clears. */
		uint32_t guard = SystemCoreClock / 1000U;
		while(LL_SPI_IsActiveFlag_BSY(job->spi_handle) && guard > 0U){
			guard--;
			__NOP();
		}
		if(guard == 0U){
			job->transfer_error = true;
		}
		cs_deassert(job->cs_port, job->cs_pin);
		spi_dma_cleanup(job);
		job->state = SPI_DMA_STATE_IDLE;

        // Hem Zili (Semaphore) Çal Hem de Kilidi (Mutex) Aç!
        osSemaphoreRelease(job->semaphoreHandle);
		if(job->dispatch_task != NULL){
			osThreadFlagsSet(job->dispatch_task, job->completion_flag);
		}

	}
}

void spi_manager_unlock_bus(SPI_TypeDef* spi_handle){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job != NULL && job->mutexHandle != NULL){
		if(job->semaphoreHandle != NULL){
			osSemaphoreAcquire(job->semaphoreHandle, 0);
		}
		osMutexRelease(job->mutexHandle);
	}
}

// NOT: Bu fonksiyon her DMA stream interrupt'ında (hem RX hem TX stream'i
// için ayrı ayrı) çağrılacak şekilde stm32f4xx_it.c'den tetiklenecek.
void spi_manager_dma_handler(DMA_TypeDef* dma_handle, uint32_t dma_stream){
	struct spi_dma_management* job = NULL;
	bool is_rx_stream = false;

	for(int i = 0; i < 3; i++){
		if(spi_jobs[i].state != SPI_DMA_STATE_IDLE && spi_jobs[i].dma_handle == dma_handle){
			if(spi_jobs[i].rx_stream == dma_stream){ job = &spi_jobs[i]; is_rx_stream = true; break; }
			if(spi_jobs[i].tx_stream == dma_stream){ job = &spi_jobs[i]; is_rx_stream = false; break; }
		}
	}
	if(job == NULL) return;

	bool tc = false, te = false;

	switch(dma_stream){
		case LL_DMA_STREAM_0:
			if(LL_DMA_IsActiveFlag_TC0(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC0(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE0(dma_handle)) { te = true; LL_DMA_ClearFlag_TE0(dma_handle); }
			break;
		case LL_DMA_STREAM_1:
			if(LL_DMA_IsActiveFlag_TC1(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC1(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE1(dma_handle)) { te = true; LL_DMA_ClearFlag_TE1(dma_handle); }
			break;
		case LL_DMA_STREAM_2:
			if(LL_DMA_IsActiveFlag_TC2(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC2(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE2(dma_handle)) { te = true; LL_DMA_ClearFlag_TE2(dma_handle); }
			break;
		case LL_DMA_STREAM_3:
			if(LL_DMA_IsActiveFlag_TC3(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC3(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE3(dma_handle)) { te = true; LL_DMA_ClearFlag_TE3(dma_handle); }
			break;
		case LL_DMA_STREAM_4:
			if(LL_DMA_IsActiveFlag_TC4(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC4(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE4(dma_handle)) { te = true; LL_DMA_ClearFlag_TE4(dma_handle); }
			break;
		case LL_DMA_STREAM_5:
			if(LL_DMA_IsActiveFlag_TC5(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC5(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE5(dma_handle)) { te = true; LL_DMA_ClearFlag_TE5(dma_handle); }
			break;
		case LL_DMA_STREAM_6:
			if(LL_DMA_IsActiveFlag_TC6(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC6(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE6(dma_handle)) { te = true; LL_DMA_ClearFlag_TE6(dma_handle); }
			break;
		case LL_DMA_STREAM_7:
			if(LL_DMA_IsActiveFlag_TC7(dma_handle)) { tc = true; LL_DMA_ClearFlag_TC7(dma_handle); }
			else if(LL_DMA_IsActiveFlag_TE7(dma_handle)) { te = true; LL_DMA_ClearFlag_TE7(dma_handle); }
			break;
		default:
			return;
	}

	if(te){
		// Hata durumunda ikisini de bitmiş sayıp temizliyoruz, yarım kalmasın.
		job->transfer_error = true;
		job->tx_done = true;
		job->rx_done = true;
		finish_transfer_if_ready(job);
		return;
	}

	if(tc){
		if(is_rx_stream) job->rx_done = true;
		else job->tx_done = true;
		finish_transfer_if_ready(job);
	}
}

void spi_manager_error_handler(SPI_TypeDef* spi_handle){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL || job->state == SPI_DMA_STATE_IDLE) return;

	if(LL_SPI_IsActiveFlag_OVR(spi_handle)) LL_SPI_ClearFlag_OVR(spi_handle);
	if(LL_SPI_IsActiveFlag_MODF(spi_handle)) LL_SPI_ClearFlag_MODF(spi_handle);

	cs_deassert(job->cs_port, job->cs_pin);
	job->transfer_error = true;
	spi_dma_cleanup(job);
	job->state = SPI_DMA_STATE_IDLE;
	osSemaphoreRelease(job->semaphoreHandle);
	if(job->dispatch_task != NULL){
		osThreadFlagsSet(job->dispatch_task, job->completion_flag);
	}
}

void spi_manager_abort_transfer(SPI_TypeDef* spi_handle){
	struct spi_dma_management* job = get_job(spi_handle);
	if(job == NULL || job->state == SPI_DMA_STATE_IDLE) return;

	cs_deassert(job->cs_port, job->cs_pin);
	job->transfer_error = true;
	spi_dma_cleanup(job);
	job->state = SPI_DMA_STATE_IDLE;
	osMutexRelease(job->mutexHandle);
}
