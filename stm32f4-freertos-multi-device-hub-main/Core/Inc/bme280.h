/*
 * bme280.h
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "i2c_manager.h"
#include "stdint.h"
#include "stddef.h"

typedef enum{
	_bme280_ok = 0,
	_bme280_fail = 1,
	_bme280_timeout = 2,
	_bme280_read_fail = 3,
	_bme280_write_fail = 4,
	_bme280_i2c_busy = 5,
	_bme280_device_not_found = 6
}bme280_return_status;

typedef enum{
	bme280_i2c_addr0 = (0x76 << 1),
	bme280_i2c_addr1 = (0x77 << 1),
}bme280_i2c_address;

typedef union{
	uint8_t raw;
	struct{
		uint8_t spi_en: 1;
		uint8_t reserved: 1;
		uint8_t filter: 3;
		uint8_t t_sb: 3;
	}bits;
}bme280_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t mode: 2;
		uint8_t osrs_p: 3;
		uint8_t osrs_t: 3;
	}bits;
}bme280_ctrlmeas_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t osrs_h: 3;
		uint8_t reserved: 5;
	}bits;
}bme280_ctrlhum_t;

// Kullanıcı konfigürasyonu
typedef struct{
	I2C_TypeDef* i2c_handle;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
	uint8_t i2c_addr;
	bme280_config_t config_t;
	bme280_ctrlhum_t ctrlhum_t;
	bme280_ctrlmeas_t ctrlmeas_t;
}bme280_user_configs;

struct bme280_t;
typedef struct bme280_t* bme280_handle_t;

// Kullanılacak Fonksiyon Prototipleri
bme280_handle_t bme280_init(bme280_user_configs* config);
bme280_return_status bme280_get_value(bme280_handle_t dev);
void bme280_get_measurements(bme280_handle_t dev, float* temperature,
		float* humidity, float* pressure);
bme280_return_status bme280_read_data_poll(bme280_handle_t dev);
bme280_return_status bme280_read_data_dma(bme280_handle_t dev);
bme280_return_status bme280_build_read_job(bme280_handle_t dev, osThreadId_t notify_task,
        uint32_t notify_flag, uint32_t error_flag, i2c_job_t* out_job);

#endif /* INC_BME280_H_ */
