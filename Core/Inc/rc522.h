/*
 * rc522.h
 *
 *  Created on: Jul 21, 2026
 *      Author: Enes
 */

#ifndef INC_RC522_H_
#define INC_RC522_H_

#include "spi_manager.h"
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

// Commands sent to the PICC.
#define PICC_REQIDL           0x26
#define PICC_REQALL           0x52
#define PICC_ANTICOLL         0x93
#define PICC_SElECTTAG        0x93
#define PICC_AUTHENT1A        0x60
#define PICC_AUTHENT1B        0x61
#define PICC_READ             0x30
#define PICC_WRITE            0xA0
#define PICC_DECREMENT        0xC0
#define PICC_INCREMENT        0xC1
#define PICC_RESTORE          0xC2
#define PICC_TRANSFER         0xB0
#define PICC_HALT             0x50

typedef enum{
	MI_OK=0,
	MI_NOTAGERR,
	MI_ERROR,
}rc522_return_status;

typedef enum{
	PCD_IDLE=0,
	PCD_MEM,
	PCD_RNG,
	PCD_CALC_CRC,
	PCD_TRANSMIT,
	PCD_NO_CMD_CHANGE=7,
	PCD_RECEIVE,
	PCD_TRANSCEIVE=12,
	PCD_MFAUTHENT=14,
	PCD_SOFTRESET=15,
}rc522_commands_t;

typedef enum{
	REG_Reserved0=0,
	REG_CommandReg,
	REG_ComlEnReg,
	REG_DivlEnReg,
	REG_ComIrqReg,
	REG_DivIrqReg,
	REG_ErrorReg,
	REG_Status1Reg,
	REG_Status2Reg,
	REG_FIFODataReg,
	REG_FIFOLevelReg,
	REG_WaterLevelReg,
	REG_ControlReg,
	REG_BitFramingReg,
	REG_CollReg,
	REG_Reserved1,
	REG_Reserved2,
	REG_ModeReg,
	REG_TxModeReg,
	REG_RxModeReg,
	REG_TxControlReg,
	REG_TxASKReg,
	REG_TxSelReg,
	REG_RxSelReg,
	REG_RxThresholdReg,
	REG_DemodReg,
	REG_Reserved3,
	REG_Reserved4,
	REG_MfTxReg,
	REG_MfRxReg,
	REG_Reserved5,
	REG_SerialSpeedReg,
	REG_Reserved6,
	REG_CRCResultReg_MSB,
	REG_CRCResultReg_LSB,
	REG_Reserved7,
	REG_ModWidthReg,
	REG_Reserved8,
	REG_RFCfgReg,
	REG_GsNReg,
	REG_CWGsPReg,
	REG_ModGsPReg,
	REG_TModeReg,
	REG_TPrescalerReg,
	REG_TReloadReg_MSB,
	REG_TReloadReg_LSB,
	REG_TCounterValReg_MSB,
	REG_TCounterValReg_LSB,
	REG_Reserved9,
	REG_TestSel1Reg,
	REG_TestSel2Reg,
	REG_TestPinEnReg,
	REG_TestPinValueReg,
	REG_TestBusReg,
	REG_AutoTestReg,
	REG_VersionReg,
	REG_AnalogTestReg,
	REG_TestDAC1Reg,
	REG_TestDAC2Reg,
	REG_TestADCReg
}rc522_register_addr_t;

// Kullanıcı konfigürasyonu
typedef struct{
	SPI_TypeDef* spi_handle;
	GPIO_TypeDef* cs_port;
	uint32_t cs_pin;
	GPIO_TypeDef* rst_port;
	uint32_t rst_pin;
}rc522_user_configs;

struct rc522_t;
typedef struct rc522_t* rc522_handle_t;

rc522_handle_t rc522_init(rc522_user_configs* config);
bool rc522_recover(rc522_handle_t dev);
uint8_t rc522_get_version(rc522_handle_t dev);

uint8_t rc522_request(rc522_handle_t dev, uint8_t reqmode, uint8_t* tagtype);
uint8_t rc522_anticoll(rc522_handle_t dev, uint8_t* psernum);
void rc522_calculate_crc(rc522_handle_t dev, uint8_t* pin, uint8_t sendlen, uint8_t* pout);
uint8_t rc522_select_tag(rc522_handle_t dev, uint8_t* sernum);
uint8_t rc522_auth(rc522_handle_t dev, uint8_t authMode, uint8_t blockAddr, uint8_t* pKey, uint8_t* pSerNum);
uint8_t rc522_read(rc522_handle_t dev, uint8_t blockAddr, uint8_t* recvdata);
uint8_t rc522_write(rc522_handle_t dev, uint8_t blockAddr, uint8_t* writedata);
void rc522_halt(rc522_handle_t dev);
void rc522_stop_crypto1(rc522_handle_t dev);

// ... mevcut kodlar
void rc522_stop_crypto1(rc522_handle_t dev);

// --- YENİ EKLENEN KESME (IRQ) FONKSİYONLARI ---
void rc522_assign_interrupt_task(rc522_handle_t dev, osThreadId_t task_handle, uint32_t flag);
void rc522_set_active_irq_device(rc522_handle_t dev);
void rc522_irq_handler(void);


#endif /* INC_RC522_H_ */
