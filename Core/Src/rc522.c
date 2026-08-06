/*
 * rc522.c
 *
 *  Created on: Jul 21, 2026
 *      Author: Enes
 */

#include "rc522.h"
#include "cmsis_os.h"

struct rc522_t{
	SPI_TypeDef* spi_handle;
	GPIO_TypeDef* cs_port;
	uint32_t cs_pin;
	GPIO_TypeDef* rst_port;
	uint32_t rst_pin;

	osThreadId_t interrupt_task_handle;
	uint32_t interrupt_flag;

};

#define max_rc522_instances 2
static struct rc522_t rc522_pool[max_rc522_instances];
static uint8_t rc522_next_free_index = 0;

/* rc522.c dosyasının başlarına bir yere ekle */
static rc522_handle_t active_rc522_irq_device = NULL;

void rc522_assign_interrupt_task(rc522_handle_t dev, osThreadId_t task_handle, uint32_t flag){
	dev->interrupt_task_handle = task_handle;
	dev->interrupt_flag = flag;
}

void rc522_set_active_irq_device(rc522_handle_t dev){
	active_rc522_irq_device = dev;
}

void rc522_irq_handler(void){
	if(active_rc522_irq_device == NULL) return;
	if(active_rc522_irq_device->interrupt_task_handle == NULL) return;
	osThreadFlagsSet(active_rc522_irq_device->interrupt_task_handle, active_rc522_irq_device->interrupt_flag);
}


// --- Ham SPI erişim katmanı (spi_manager üzerinden) ---

static void _rc522_write_data(rc522_handle_t dev, uint8_t reg_addr, uint8_t txdata){
	uint8_t txbuf[2] = { (uint8_t)((reg_addr << 1) & 0x7E), txdata };
	uint8_t rxbuf[2];
	spi_manager_transfer_poll(dev->spi_handle, dev->cs_port, dev->cs_pin, txbuf, rxbuf, 2);
}

static uint8_t _rc522_read_data(rc522_handle_t dev, uint8_t reg_addr){
	uint8_t txbuf[2] = { (uint8_t)(((reg_addr << 1) & 0x7E) | 0x80), 0x00 };
	/* Transfer basarisiz olursa rastgele stack verisini cihaz cevabi sanma. */
	uint8_t rxbuf[2] = {0};
	spi_manager_transfer_poll(dev->spi_handle, dev->cs_port, dev->cs_pin, txbuf, rxbuf, 2);
	return rxbuf[1];
}

uint8_t rc522_get_version(rc522_handle_t dev){
	if(dev == NULL) return 0;
	return _rc522_read_data(dev, REG_VersionReg);
}

static void _rc522_set_bitmask(rc522_handle_t dev, uint8_t reg_addr, uint8_t mask){
	uint8_t temp = _rc522_read_data(dev, reg_addr);
	_rc522_write_data(dev, reg_addr, temp | mask);
}

static void _rc522_clear_bitmask(rc522_handle_t dev, uint8_t reg_addr, uint8_t mask){
	uint8_t temp = _rc522_read_data(dev, reg_addr);
	_rc522_write_data(dev, reg_addr, temp & (~mask));
}

static void _rc522_antenna_on(rc522_handle_t dev){
	_rc522_set_bitmask(dev, REG_TxControlReg, 0x03);
}

static inline void _rc522_antenna_off(rc522_handle_t dev){
	_rc522_clear_bitmask(dev, REG_TxControlReg, 0x03);
}

static void _rc522_reset(rc522_handle_t dev){
	_rc522_write_data(dev, REG_CommandReg, PCD_SOFTRESET);
}

// --- Init ---

rc522_handle_t rc522_init(rc522_user_configs* config){
	if(config == NULL) return NULL;
	if(rc522_next_free_index >= max_rc522_instances) return NULL;

	rc522_handle_t dev = &rc522_pool[rc522_next_free_index];
	rc522_next_free_index++;

	dev->spi_handle = config->spi_handle;
	dev->cs_port = config->cs_port;
	dev->cs_pin = config->cs_pin;
	dev->rst_port = config->rst_port;
	dev->rst_pin = config->rst_pin;

	// CS'i idle (HIGH) durumda garantiye al
	LL_GPIO_SetOutputPin(dev->cs_port, dev->cs_pin);

	// Donanımsal reset darbesi
	LL_GPIO_ResetOutputPin(dev->rst_port, dev->rst_pin);
	osDelay(10);
	LL_GPIO_SetOutputPin(dev->rst_port, dev->rst_pin);
	osDelay(50);

	_rc522_reset(dev);
	_rc522_set_bitmask(dev, REG_DivlEnReg, 0x80);
	_rc522_write_data(dev, REG_TModeReg, 0x8D);
	_rc522_write_data(dev, REG_TPrescalerReg, 0x3E);
	_rc522_write_data(dev, REG_TReloadReg_LSB, 30);
	_rc522_write_data(dev, REG_TReloadReg_MSB, 0);
	_rc522_write_data(dev, REG_TxASKReg, 0x40);
	_rc522_write_data(dev, REG_ModeReg, 0x3D);
	_rc522_set_bitmask(dev, REG_DivlEnReg, 0x80); // pull-up için eklendi
	_rc522_antenna_on(dev);

	return dev;
}

// --- Kart haberleşme mantığı (protokol seviyesi, değişmedi) ---

static uint8_t _rc522_to_card(rc522_handle_t dev, uint8_t command, uint8_t* senddata, uint8_t sendlen, uint8_t* backdata, uint32_t* backlen){
	uint8_t status = MI_ERROR;
	uint8_t n;
	uint8_t lastbits;
	uint8_t waitirq = 0;
	uint8_t irqen = 0;
	uint32_t i;

	// KATI RTOS KURALI: Eğer Interrupt Görevi atanmamışsa, sistemi kitleme, direkt reddet!
	if (dev->interrupt_task_handle == NULL) {
		return MI_ERROR;
	}

	switch(command){
		case PCD_MFAUTHENT:
			waitirq = 0x10;
			irqen = waitirq | 0x01;
			break;
		case PCD_TRANSCEIVE:
			waitirq = 0x30;
			irqen = waitirq | 0x01;
			break;
		default:
			break;
	}

	_rc522_write_data(dev, REG_ComlEnReg, irqen | 0x80);
	_rc522_write_data(dev, REG_ComIrqReg, 0x7F);
	_rc522_set_bitmask(dev, REG_FIFOLevelReg, 0x80);
	_rc522_write_data(dev, REG_CommandReg, PCD_IDLE);

	for(i = 0; i < sendlen; i++){
		_rc522_write_data(dev, REG_FIFODataReg, senddata[i]);
	}

	osThreadFlagsClear(dev->interrupt_flag);

	_rc522_write_data(dev, REG_CommandReg, command);
	if(command == PCD_TRANSCEIVE){
		_rc522_set_bitmask(dev, REG_BitFramingReg, 0x80);
	}

	// --- %100 SAF RTOS MİMARİSİ (POLLING TAMAMEN SİLİNDİ) ---
	uint32_t flags = osThreadFlagsWait(dev->interrupt_flag, osFlagsWaitAny, 40);

	n = _rc522_read_data(dev, REG_ComIrqReg);
	_rc522_write_data(dev, REG_ComIrqReg, 0x7F);

	if (flags == osFlagsErrorTimeout) {
		i = 0;
		_rc522_write_data(dev, REG_CommandReg, PCD_IDLE);
	} else {
		if (n & waitirq) {
			i = 1;
		} else {
			i = 0;
		}
	}

	_rc522_clear_bitmask(dev, REG_BitFramingReg, 0x80);

	if(i != 0){
		if(!(_rc522_read_data(dev, REG_ErrorReg) & 0x1B)){
			if(n & 0x01 & irqen){
				status = MI_NOTAGERR;
				return status;
			}
			status = MI_OK;
			if(command == PCD_TRANSCEIVE){
				n = _rc522_read_data(dev, REG_FIFOLevelReg);
				lastbits = _rc522_read_data(dev, REG_ControlReg) & 0x07;
				if(lastbits){
					*backlen = (n - 1) * 8 + lastbits;
				}
				else{
					*backlen = n * 8;
				}
				if(n == 0) { n = 1; }
				if(n > 16) { n = 16; }
				for(i = 0; i < n; i++){
					backdata[i] = _rc522_read_data(dev, REG_FIFODataReg);
				}
			}
		}
	}
	return status;
}

uint8_t rc522_request(rc522_handle_t dev, uint8_t reqmode, uint8_t* tagtype){
	uint32_t backlen;
	_rc522_write_data(dev, REG_BitFramingReg, 0x07);
	uint8_t status = _rc522_to_card(dev, PCD_TRANSCEIVE, &reqmode, 1, tagtype, &backlen);
	if((status != MI_OK) || (backlen != 0x10)){
		status = MI_ERROR;
	}
	return status;
}

uint8_t rc522_anticoll(rc522_handle_t dev, uint8_t* psernum){
	uint8_t sernumcheck = 0;
	uint32_t backlen_bits;
	uint8_t buffer[2];
	buffer[0] = PICC_ANTICOLL;
	buffer[1] = 0x20;
	_rc522_write_data(dev, REG_BitFramingReg, 0x00);
	uint8_t status = _rc522_to_card(dev, PCD_TRANSCEIVE, buffer, 2, psernum, &backlen_bits);
	if((status == MI_OK) && backlen_bits == 40){
		for(uint8_t i = 0; i < 4; i++){
			sernumcheck ^= psernum[i];
		}
		if(sernumcheck != psernum[4]){
			status = MI_ERROR;
		}
	}
	else{
		status = MI_ERROR;
	}
	return status;
}

void rc522_calculate_crc(rc522_handle_t dev, uint8_t* pin, uint8_t sendlen, uint8_t* pout){
	_rc522_clear_bitmask(dev, REG_DivIrqReg, 0x04);
	_rc522_set_bitmask(dev, REG_FIFOLevelReg, 0x80);
	uint8_t i, n;
	for(i = 0; i < sendlen; i++){
		_rc522_write_data(dev, REG_FIFODataReg, pin[i]);
	}
	_rc522_write_data(dev, REG_CommandReg, PCD_CALC_CRC);
	i = 255;
	do{
		n = _rc522_read_data(dev, REG_DivIrqReg);
		i--;
	} while((i != 0) && !(n & 0x04));
	pout[0] = _rc522_read_data(dev, REG_CRCResultReg_LSB);
	pout[1] = _rc522_read_data(dev, REG_CRCResultReg_MSB);
}

uint8_t rc522_select_tag(rc522_handle_t dev, uint8_t* sernum){
	uint8_t i;
	uint8_t buffer[9];
	uint8_t size;
	uint8_t status;
	uint32_t receive_bits;
	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for(i = 0; i < 5; i++){
		buffer[i + 2] = sernum[i];
	}
	rc522_calculate_crc(dev, buffer, 7, &buffer[7]);
	status = _rc522_to_card(dev, PCD_TRANSCEIVE, buffer, 9, buffer, &receive_bits);
	if(status == MI_OK && receive_bits == 24){
		size = buffer[0];
	}
	else{
		size = 0;
	}
	return size;
}

uint8_t rc522_auth(rc522_handle_t dev, uint8_t authMode, uint8_t blockAddr, uint8_t* pKey, uint8_t* pSerNum){
	uint8_t status;
	uint8_t buffer[12];
	buffer[0] = authMode;
	buffer[1] = blockAddr;
	uint8_t i, n;
	uint32_t backbits;
	for(i = 0; i < 6; i++){
		buffer[i + 2] = pKey[i];
	}
	for(i = 0; i < 4; i++){
		buffer[i + 8] = pSerNum[i];
	}
	status = _rc522_to_card(dev, PCD_MFAUTHENT, buffer, 12, buffer, &backbits);
	n = _rc522_read_data(dev, REG_Status2Reg) & 0x08;
	if(n != 8 || status != MI_OK){
		status = MI_ERROR;
	}
	return status;
}

void rc522_stop_crypto1(rc522_handle_t dev){
	_rc522_clear_bitmask(dev, REG_Status2Reg, 0x08);
}

uint8_t rc522_read(rc522_handle_t dev, uint8_t blockAddr, uint8_t* recvdata){
	uint8_t status;
	uint8_t buffer[4];
	uint32_t backlen;
	buffer[0] = PICC_READ;
	buffer[1] = blockAddr;
	rc522_calculate_crc(dev, buffer, 2, &buffer[2]);
	status = _rc522_to_card(dev, PCD_TRANSCEIVE, buffer, 4, recvdata, &backlen);
	if(status != MI_OK || backlen != 0x90){
		status = MI_ERROR;
	}
	return status;
}

uint8_t rc522_write(rc522_handle_t dev, uint8_t blockAddr, uint8_t* writedata){
	uint8_t status;
	uint8_t sendbuffer[18];
	uint8_t receivebuffer[4];
	uint32_t backlen;
	uint8_t i;
	sendbuffer[0] = PICC_WRITE;
	sendbuffer[1] = blockAddr;
	rc522_calculate_crc(dev, sendbuffer, 2, &sendbuffer[2]);
	status = _rc522_to_card(dev, PCD_TRANSCEIVE, sendbuffer, 4, receivebuffer, &backlen);
	if((receivebuffer[0] & 0x0F) != 0x0A || backlen != 4 || status != MI_OK){
		status = MI_ERROR;
		return status;
	}
	receivebuffer[0] = 0;
	for(i = 0; i < 16; i++){
		sendbuffer[i] = writedata[i];
	}
	rc522_calculate_crc(dev, sendbuffer, 16, &sendbuffer[16]);
	status = _rc522_to_card(dev, PCD_TRANSCEIVE, sendbuffer, 18, receivebuffer, &backlen);
	if((receivebuffer[0] & 0x0F) != 0x0A || backlen != 4 || status != MI_OK){
		status = MI_ERROR;
	}
	return status;
}

void rc522_halt(rc522_handle_t dev){
	uint8_t buffer[4];
	uint32_t backlen;
	buffer[0] = PICC_HALT;
	buffer[1] = 0;
	rc522_calculate_crc(dev, buffer, 2, &buffer[2]);
	_rc522_to_card(dev, PCD_TRANSCEIVE, buffer, 4, &buffer[4], &backlen);
}
