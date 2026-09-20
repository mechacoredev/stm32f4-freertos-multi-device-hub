#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "spi_manager.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "stddef.h"

#define NRF24L01_PAYLOAD_SIZE 32
#define NRF24L01_FIFO_DEPTH   3

#define NRF24L01_IRQ_RX_DR  0x40
#define NRF24L01_IRQ_TX_DS  0x20
#define NRF24L01_IRQ_MAX_RT 0x10

typedef enum{
	_nrf24l01_ok = 0,
	_nrf24l01_fail,
	_nrf24l01_timeout,
	_nrf24l01_device_not_found
}nrf24l01_return_status;

typedef struct{
	uint8_t raw;
	bool rx_dr;
	bool tx_ds;
	bool max_rt;
}nrf24l01_irq_status_t;

typedef struct{
	uint8_t raw;
	bool tx_reuse;
	bool tx_full;
	bool tx_empty;
	bool rx_full;
	bool rx_empty;
}nrf24l01_fifo_status_t;

// Kullanıcı Konfigürasyonu (bme280_user_configs gibi)
typedef struct{
	SPI_TypeDef* spi_handle;
	DMA_TypeDef* dma_handle;
	uint32_t rx_stream;
	uint32_t tx_stream;

	GPIO_TypeDef* ce_port;
	uint32_t ce_pin;
	GPIO_TypeDef* csn_port;
	uint32_t csn_pin;

    // DMA'nın bitişini bekleyeceğimiz zil (spi_manager'a verdiğimiz ile aynı olmalı)
	osSemaphoreId_t spi_semaphore;
	// EXTI kesmesini bekleyeceğimiz Görev (Task)
	osThreadId_t notify_task;
	uint32_t notify_flag;     // YENİ: MPU6500'deki gibi hangi bayrağı kaldıracağımızı belirler
}nrf24l01_user_configs;

struct nrf24l01_t;
typedef struct nrf24l01_t* nrf24l01_handle_t;

// Kullanılacak Fonksiyon Prototipleri
nrf24l01_handle_t nrf24l01_init(nrf24l01_user_configs* config);
nrf24l01_return_status nrf24l01_reinitialize(nrf24l01_handle_t dev);
bool nrf24l01_validate(nrf24l01_handle_t dev);

// DMA Tabanlı Asenkron Veri Transferleri
nrf24l01_return_status nrf24l01_send_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length);
nrf24l01_return_status nrf24l01_receive_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length);

// SPI DMA worker task ile kullanilan job tabanli API
nrf24l01_return_status nrf24l01_build_tx_job(nrf24l01_handle_t dev, const uint8_t *data, uint8_t length,
		osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag, spi_job_t* out_job);
nrf24l01_return_status nrf24l01_build_rx_fifo_job(nrf24l01_handle_t dev,
		osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
		uint8_t* out_payload_length, spi_job_t* out_job);
nrf24l01_return_status nrf24l01_finish_rx_fifo_job(nrf24l01_handle_t dev, uint8_t *data, uint8_t length);
nrf24l01_return_status nrf24l01_trigger_transmission(nrf24l01_handle_t dev);
nrf24l01_return_status nrf24l01_get_irq_status(nrf24l01_handle_t dev, nrf24l01_irq_status_t* out_status);
nrf24l01_return_status nrf24l01_get_fifo_status(nrf24l01_handle_t dev, nrf24l01_fifo_status_t* out_status);
void nrf24l01_clear_irq_sources(nrf24l01_handle_t dev, uint8_t irq_mask);

// Durum Kontrolleri
void nrf24l01_clear_interrupts(nrf24l01_handle_t dev);
void nrf24l01_start_listening(nrf24l01_handle_t dev);
void nrf24l01_stop_listening(nrf24l01_handle_t dev);

void nrf24l01_assign_interrupt_task(nrf24l01_handle_t dev, osThreadId_t task, uint32_t flag);
void nrf24l01_irq_handler(nrf24l01_handle_t dev);

void flush_rx(nrf24l01_handle_t dev);
void flush_tx(nrf24l01_handle_t dev);

#endif /* INC_NRF24L01_H_ */
