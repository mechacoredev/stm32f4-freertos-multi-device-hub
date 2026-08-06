#include "nrf24l01.h"
#include "string.h"

// nRF24L01 Yazmaç (Register) Adresleri
typedef enum{
	REG_CONFIG = 0x00,
	REG_EN_AA = 0x01,
	REG_EN_RXADDR = 0x02,
	REG_SETUP_AW = 0x03,
	REG_SETUP_RETR = 0x04,
	REG_RF_CH = 0x05,
	REG_RF_SETUP = 0x06,
	REG_STATUS = 0x07,
	REG_RX_ADDR_P0 = 0x0A,
	REG_TX_ADDR = 0x10,
	REG_RX_PW_P0 = 0x11,
	REG_FIFO_STATUS = 0x17,
	REG_DYNPD = 0x1C,
	REG_FEATURE = 0x1D
}nrf24l01_register_map;

struct nrf24l01_t{
	SPI_TypeDef* spi_handle;
	DMA_TypeDef* dma_handle;
	uint32_t rx_stream;
	uint32_t tx_stream;
	GPIO_TypeDef* ce_port;
	uint32_t ce_pin;
	GPIO_TypeDef* csn_port;
	uint32_t csn_pin;

	osSemaphoreId_t spi_semaphore;
	osThreadId_t notify_task;
	uint32_t notify_flag;     // YENİ: MPU6050'deki gibi hangi bayrağı kaldıracağımızı belirler
	uint8_t address[5];

    // DMA transferleri için dahili RAM alanları (Max 33 Byte)
    uint8_t dma_tx_buffer[33];
    uint8_t dma_rx_buffer[33];
};

// MALLOC YERİNE STATİK HAVUZ (Max 2 adet nRF cihazı destekler)
#define max_nrf_instances 2
static struct nrf24l01_t nrf_pool[max_nrf_instances];
static uint8_t next_nrf_free_index = 0;

// --- DÜŞÜK SEVİYE OKUMA/YAZMA (SPI MANAGER ÜZERİNDEN) ---

static void write_data(nrf24l01_handle_t dev, uint8_t register_address, uint8_t txdata){
	uint8_t tx[2] = { (register_address & 0x1F) | 0x20, txdata }; // 0x20 = Write Komutu
	uint8_t rx[2] = {0};
	// spi_manager CS pinini kendisi indirir ve kaldırır!
	spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, tx, rx, 2);
}

static uint8_t read_data(nrf24l01_handle_t dev, uint8_t register_address){
	uint8_t tx[2] = { register_address & 0x1F, 0xFF }; // 0xFF = Dummy
	uint8_t rx[2] = {0};
	spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, tx, rx, 2);
	return rx[1];
}

static void activate_feature_registers(nrf24l01_handle_t dev){
	uint8_t tx[2] = {0x50, 0x73};
	uint8_t rx[2] = {0};
	spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, tx, rx, 2);
}

static nrf24l01_return_status enable_dynamic_payloads(nrf24l01_handle_t dev){
	write_data(dev, REG_FEATURE, 0x04);

	uint8_t feature_register = read_data(dev, REG_FEATURE);
	bool feature_write_succeeded = (feature_register & 0x04) != 0;
	if(feature_write_succeeded == false){
		activate_feature_registers(dev);
		write_data(dev, REG_FEATURE, 0x04);
	}

	write_data(dev, REG_DYNPD, 0x01);

	feature_register = read_data(dev, REG_FEATURE);
	uint8_t dynpd_register = read_data(dev, REG_DYNPD);
	bool dynamic_payload_enabled = (feature_register & 0x04) != 0;
	bool pipe_zero_enabled = (dynpd_register & 0x01) != 0;

	if(dynamic_payload_enabled == false) return _nrf24l01_fail;
	if(pipe_zero_enabled == false) return _nrf24l01_fail;
	return _nrf24l01_ok;
}

static nrf24l01_return_status read_dynamic_payload_length(nrf24l01_handle_t dev,
		uint8_t* out_payload_length){
	if(dev == NULL || out_payload_length == NULL) return _nrf24l01_fail;

	uint8_t tx[2] = {0x60, 0xFF};
	uint8_t rx[2] = {0};
	spi_manager_return_status transfer_status;
	transfer_status = spi_manager_transfer_poll(dev->spi_handle, dev->csn_port,
			dev->csn_pin, tx, rx, 2);
	if(transfer_status != _spi_manager_ok) return _nrf24l01_fail;

	uint8_t payload_length = rx[1];
	if(payload_length == 0 || payload_length > NRF24L01_PAYLOAD_SIZE){
		flush_rx(dev);
		nrf24l01_clear_irq_sources(dev, NRF24L01_IRQ_RX_DR);
		return _nrf24l01_fail;
	}

	*out_payload_length = payload_length;
	return _nrf24l01_ok;
}

static void write_data_burst(nrf24l01_handle_t dev, uint8_t register_address, uint8_t *txdata, uint8_t size){
	dev->dma_tx_buffer[0] = (register_address & 0x1F) | 0x20;
	memcpy(&(dev->dma_tx_buffer[1]), txdata, size);
	spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, dev->dma_tx_buffer, dev->dma_rx_buffer, size + 1);
}

static void set_bit_mask(nrf24l01_handle_t dev, uint8_t register_address, uint8_t mask){
	uint8_t buffer = read_data(dev, register_address);
	write_data(dev, register_address, buffer | mask);
}

static void clear_bit_mask(nrf24l01_handle_t dev, uint8_t register_address, uint8_t mask){
	uint8_t buffer = read_data(dev, register_address);
	write_data(dev, register_address, buffer & (~mask));
}

void nrf24l01_clear_interrupts(nrf24l01_handle_t dev){
    write_data(dev, REG_STATUS, 0x70); // RX_DR | TX_DS | MAX_RT bayraklarını indir
}

void nrf24l01_clear_irq_sources(nrf24l01_handle_t dev, uint8_t irq_mask){
	if(dev == NULL) return;
	write_data(dev, REG_STATUS, irq_mask & 0x70);
}

void flush_tx(nrf24l01_handle_t dev){
    uint8_t tx = 0xE1; // FLUSH_TX Komutu
    uint8_t rx = 0;
    spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, &tx, &rx, 1);
}

void flush_rx(nrf24l01_handle_t dev){
    uint8_t tx = 0xE2; // FLUSH_RX Komutu
    uint8_t rx = 0;
    spi_manager_transfer_poll(dev->spi_handle, dev->csn_port, dev->csn_pin, &tx, &rx, 1);
}

// --- BAŞLATMA (INIT) FONKSİYONU ---

nrf24l01_handle_t nrf24l01_init(nrf24l01_user_configs* config){
	if(config == NULL || next_nrf_free_index >= max_nrf_instances) return NULL;

	nrf24l01_handle_t dev = &nrf_pool[next_nrf_free_index];
	next_nrf_free_index++;

	if(config->spi_handle == NULL || config->dma_handle == NULL ||
	   config->spi_semaphore == NULL || config->ce_port == NULL ||
	   config->csn_port == NULL){
		return NULL;
	}

	dev->spi_handle = config->spi_handle;
	dev->dma_handle = config->dma_handle;
	dev->rx_stream  = config->rx_stream;
	dev->tx_stream  = config->tx_stream;
	dev->ce_port    = config->ce_port;
	dev->ce_pin     = config->ce_pin;
	dev->csn_port   = config->csn_port;
	dev->csn_pin    = config->csn_pin;
	dev->spi_semaphore = config->spi_semaphore;
	dev->notify_task   = config->notify_task;

	// nRF24L01 bos durumdayken CSN HIGH, CE LOW olmalidir.
	LL_GPIO_SetOutputPin(dev->csn_port, dev->csn_pin);
	LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin);
    osDelay(2); // Modül uyanma süresi (1.5ms)

	uint8_t default_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	memcpy(dev->address, default_address, 5);

	write_data(dev, REG_CONFIG, 0x0A); // PWR_UP=1, CRCO=1
	write_data(dev, REG_EN_AA, 0x01);
	write_data(dev, REG_EN_RXADDR, 0x01);
	write_data(dev, REG_SETUP_AW, 0x03); // 5 Byte Adres
	write_data(dev, REG_SETUP_RETR, 0x3F); // 15 Tekrar, 1000us
	write_data(dev, REG_RF_CH, 76);
	write_data(dev, REG_RF_SETUP, 0x07); // 0x07 yerine 0x01
	// Dynamic Payload Length: Pipe 0 icin gercek paket uzunlugunu kullan.
	write_data(dev, REG_RX_PW_P0, 0);
	nrf24l01_return_status dynamic_payload_status;
	dynamic_payload_status = enable_dynamic_payloads(dev);
	if(dynamic_payload_status != _nrf24l01_ok) return NULL;

	write_data_burst(dev, REG_RX_ADDR_P0, dev->address, 5);
	write_data_burst(dev, REG_TX_ADDR, dev->address, 5);

	flush_tx(dev);
	flush_rx(dev);
	nrf24l01_clear_interrupts(dev);

	return dev;
}

void nrf24l01_start_listening(nrf24l01_handle_t dev){
    set_bit_mask(dev, REG_CONFIG, 0x03); // PRIM_RX=1
    LL_GPIO_SetOutputPin(dev->ce_port, dev->ce_pin); // Anteni aç
}

void nrf24l01_stop_listening(nrf24l01_handle_t dev){
    LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin); // Anteni kapat
}

// --- %100 ASENKRON, BLOKLAMASIZ DMA TRANSFERLERİ ---

nrf24l01_return_status nrf24l01_send_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length){
	if(dev == NULL || data == NULL || length == 0 || length > NRF24L01_PAYLOAD_SIZE){
		return _nrf24l01_fail;
	}

    // 1. Anteni kapat ve TX moduna geç
    LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin);
    set_bit_mask(dev, REG_CONFIG, 0x02);
    clear_bit_mask(dev, REG_CONFIG, 0x01); // PRIM_RX=0

    // 2. Bufferları Hazırla (0xA0 = W_TX_PAYLOAD)
    dev->dma_tx_buffer[0] = 0xA0;
    memset(&(dev->dma_tx_buffer[1]), 0, NRF24L01_PAYLOAD_SIZE);
    memcpy(&(dev->dma_tx_buffer[1]), data, length);
    memset(dev->dma_rx_buffer, 0, sizeof(dev->dma_rx_buffer));

    // 3. Daha önceden kalan EXTI bildirimlerini temizle
    osThreadFlagsClear(dev->notify_flag);

    // 4. SPI Yöneticisine "Bu veriyi DMA ile bas" emrini ver
    spi_manager_return_status transfer_status;
    transfer_status = spi_manager_transfer_dma(dev->spi_handle, dev->dma_handle,
            dev->rx_stream, dev->tx_stream, dev->csn_port, dev->csn_pin,
            dev->dma_tx_buffer, dev->dma_rx_buffer, length + 1);
    if(transfer_status != _spi_manager_ok){
        return _nrf24l01_fail;
    }

    // 5. DMA Transferinin bitmesini RTOS uyku modunda bekle (%0 CPU)
    osStatus_t semaphore_status = osSemaphoreAcquire(dev->spi_semaphore, osWaitForever);
    if(semaphore_status != osOK){
        spi_manager_unlock_bus(dev->spi_handle);
        return _nrf24l01_fail;
    }

    // EKLENEN KRİTİK SATIR: UYANDIK, DMA İŞİ BİTTİ! HEMEN MUTEX'İ SERBEST BIRAK!
    spi_manager_unlock_bus(dev->spi_handle);

    // 6. Veriyi Donanımsal Olarak Havalandır (CE Pinini en az 10us ateşle)
        LL_GPIO_SetOutputPin(dev->ce_port, dev->ce_pin);
        osDelay(1); // 1ms pulse
        LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin);

        // EKLENEN KRİTİK FİZİKSEL PAY: Radyo dalgasının havada gidip, alıcının işleyip
        // Auto-ACK'in geri dönmesi için gereken "Uçuş Süresi (Flight Time)"
        osDelay(5);

        // 7. Karşı taraftan ACK geldiğine dair EXTI (IRQ) kesmesini bekle!
        uint32_t exti_flag = osThreadFlagsWait(dev->notify_flag, osFlagsWaitAny, 100);

        nrf24l01_clear_interrupts(dev);

        // EKLENEN KRİTİK FİZİKSEL PAY 2: Çipin kendini tamamen toparlaması için (Recovery)
        osDelay(2);

        if(exti_flag == osFlagsErrorTimeout) return _nrf24l01_timeout;
        return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_receive_dma(nrf24l01_handle_t dev, uint8_t *data, uint8_t length){
	if(dev == NULL || data == NULL || length == 0 || length > NRF24L01_PAYLOAD_SIZE){
		return _nrf24l01_fail;
	}

	uint8_t payload_length = 0;
	nrf24l01_return_status payload_length_status;
	payload_length_status = read_dynamic_payload_length(dev, &payload_length);
	if(payload_length_status != _nrf24l01_ok) return _nrf24l01_fail;
	if(payload_length > length) return _nrf24l01_fail;

    // 1. Bufferları Hazırla (0x61 = R_RX_PAYLOAD)
    dev->dma_tx_buffer[0] = 0x61;
    memset(&(dev->dma_tx_buffer[1]), 0xFF, payload_length); // Okuma icin clock uret
    // 2. SPI Yöneticisine DMA emrini ver
    spi_manager_return_status transfer_status;
    transfer_status = spi_manager_transfer_dma(dev->spi_handle, dev->dma_handle,
            dev->rx_stream, dev->tx_stream, dev->csn_port, dev->csn_pin,
            dev->dma_tx_buffer, dev->dma_rx_buffer, payload_length + 1);
    if(transfer_status != _spi_manager_ok){
        return _nrf24l01_fail;
    }

    // 3. İşlemci DMA okumasını bitirene kadar uyusun
    osStatus_t semaphore_status = osSemaphoreAcquire(dev->spi_semaphore, osWaitForever);
    if(semaphore_status != osOK){
        spi_manager_unlock_bus(dev->spi_handle);
        return _nrf24l01_fail;
    }
    // EKLENEN KRİTİK SATIR: UYANDIK! MUTEX'İ SERBEST BIRAK!
    spi_manager_unlock_bus(dev->spi_handle);
    // 4. Gelen veriyi kullanıcı dizisine kopyala (0. indeks Status byte'ı olduğu için 1'den başlıyoruz)
    memcpy(data, &(dev->dma_rx_buffer[1]), payload_length);

    nrf24l01_clear_interrupts(dev);
    return _nrf24l01_ok;
}

static void fill_spi_job(nrf24l01_handle_t dev, osThreadId_t notify_task,
		uint32_t notify_flag, uint32_t error_flag, uint16_t transfer_size,
		spi_job_t* out_job){
	out_job->spi_handle = dev->spi_handle;
	out_job->dma_handle = dev->dma_handle;
	out_job->rx_stream = dev->rx_stream;
	out_job->tx_stream = dev->tx_stream;
	out_job->cs_port = dev->csn_port;
	out_job->cs_pin = dev->csn_pin;
	out_job->txdata = dev->dma_tx_buffer;
	out_job->rxdata = dev->dma_rx_buffer;
	out_job->size = transfer_size;
	out_job->completion_semaphore = dev->spi_semaphore;
	out_job->notify_task = notify_task;
	out_job->notify_flag = notify_flag;
	out_job->error_flag = error_flag;
}

nrf24l01_return_status nrf24l01_get_irq_status(nrf24l01_handle_t dev, nrf24l01_irq_status_t* out_status){
	if(dev == NULL || out_status == NULL) return _nrf24l01_fail;
	uint8_t raw = read_data(dev, REG_STATUS);
	out_status->raw = raw;
	out_status->rx_dr = (raw & NRF24L01_IRQ_RX_DR) != 0;
	out_status->tx_ds = (raw & NRF24L01_IRQ_TX_DS) != 0;
	out_status->max_rt = (raw & NRF24L01_IRQ_MAX_RT) != 0;
	return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_get_fifo_status(nrf24l01_handle_t dev, nrf24l01_fifo_status_t* out_status){
	if(dev == NULL || out_status == NULL) return _nrf24l01_fail;
	uint8_t raw = read_data(dev, REG_FIFO_STATUS);
	out_status->raw = raw;
	out_status->tx_reuse = (raw & 0x40) != 0;
	out_status->tx_full = (raw & 0x20) != 0;
	out_status->tx_empty = (raw & 0x10) != 0;
	out_status->rx_full = (raw & 0x02) != 0;
	out_status->rx_empty = (raw & 0x01) != 0;
	return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_build_tx_job(nrf24l01_handle_t dev, const uint8_t *data, uint8_t length,
		osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag, spi_job_t* out_job){
	if(dev == NULL || data == NULL || out_job == NULL || notify_task == NULL ||
	   length == 0 || length > NRF24L01_PAYLOAD_SIZE) return _nrf24l01_fail;

	nrf24l01_fifo_status_t fifo;
	nrf24l01_return_status fifo_read_status = nrf24l01_get_fifo_status(dev, &fifo);
	if(fifo_read_status != _nrf24l01_ok) return _nrf24l01_fail;
	if(fifo.tx_full) return _nrf24l01_fail;

	LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin);
	set_bit_mask(dev, REG_CONFIG, 0x02);
	clear_bit_mask(dev, REG_CONFIG, 0x01);
	nrf24l01_clear_irq_sources(dev, NRF24L01_IRQ_TX_DS | NRF24L01_IRQ_MAX_RT);

	dev->dma_tx_buffer[0] = 0xA0;
	memset(&dev->dma_tx_buffer[1], 0, NRF24L01_PAYLOAD_SIZE);
	memcpy(&dev->dma_tx_buffer[1], data, length);
	memset(dev->dma_rx_buffer, 0, sizeof(dev->dma_rx_buffer));
	fill_spi_job(dev, notify_task, notify_flag, error_flag, length + 1, out_job);
	return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_build_rx_fifo_job(nrf24l01_handle_t dev,
		osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
		uint8_t* out_payload_length, spi_job_t* out_job){
	if(dev == NULL || out_payload_length == NULL || out_job == NULL || notify_task == NULL){
		return _nrf24l01_fail;
	}

	nrf24l01_fifo_status_t fifo;
	nrf24l01_return_status fifo_read_status = nrf24l01_get_fifo_status(dev, &fifo);
	if(fifo_read_status != _nrf24l01_ok) return _nrf24l01_fail;
	if(fifo.rx_empty) return _nrf24l01_fail;

	uint8_t payload_length = 0;
	nrf24l01_return_status payload_length_status;
	payload_length_status = read_dynamic_payload_length(dev, &payload_length);
	if(payload_length_status != _nrf24l01_ok) return _nrf24l01_fail;

	dev->dma_tx_buffer[0] = 0x61;
	memset(&dev->dma_tx_buffer[1], 0xFF, payload_length);
	memset(dev->dma_rx_buffer, 0, sizeof(dev->dma_rx_buffer));
	*out_payload_length = payload_length;
	fill_spi_job(dev, notify_task, notify_flag, error_flag, payload_length + 1, out_job);
	return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_finish_rx_fifo_job(nrf24l01_handle_t dev, uint8_t *data, uint8_t length){
	if(dev == NULL || data == NULL || length == 0 || length > NRF24L01_PAYLOAD_SIZE) return _nrf24l01_fail;
	memcpy(data, &dev->dma_rx_buffer[1], length);
	return _nrf24l01_ok;
}

nrf24l01_return_status nrf24l01_trigger_transmission(nrf24l01_handle_t dev){
	if(dev == NULL) return _nrf24l01_fail;
	LL_GPIO_SetOutputPin(dev->ce_port, dev->ce_pin);
	osDelay(1);
	LL_GPIO_ResetOutputPin(dev->ce_port, dev->ce_pin);
	return _nrf24l01_ok;
}

void nrf24l01_assign_interrupt_task(nrf24l01_handle_t dev, osThreadId_t task, uint32_t flag){
    if(dev != NULL){
        dev->notify_task = task;
        // struct içine uint32_t notify_flag eklediğini varsayıyoruz
        dev->notify_flag = flag;
    }
}

void nrf24l01_irq_handler(nrf24l01_handle_t dev){
    if(dev != NULL && dev->notify_task != NULL){
        osThreadFlagsSet(dev->notify_task, dev->notify_flag);
    }
}
