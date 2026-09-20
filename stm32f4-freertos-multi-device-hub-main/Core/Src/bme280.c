/*
 * bme280.c
 */

#include "bme280.h"
#include "cmsis_os.h" // osDelay için eklendi

typedef enum{
	addr_chip_id = 0xD0,
	addr_reset = 0xE0 ,
	addr_ctrlhum = 0xF2 ,
	addr_status = 0xF3 ,
	addr_ctrlmeas = 0xF4 ,
	addr_config = 0xF5 ,
	addr_press_msb = 0xF7 ,
	addr_dig_t1 = 0x88 ,
	addr_dig_h1 = 0xA1 ,
	addr_dig_h2 = 0xE1 ,
}bme280_register_address_t;

struct bme280_t{
	I2C_TypeDef* i2c_handle;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;
	uint8_t i2c_addr;
	bme280_config_t config_t;
	bme280_ctrlhum_t ctrlhum_t;
	bme280_ctrlmeas_t ctrlmeas_t;

	// Kalibrasyon Verileri
	uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
	uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
	int16_t dig_P4;  int16_t dig_P5; int16_t dig_P6;
	int16_t dig_P7;  int16_t dig_P8; int16_t dig_P9;
	uint8_t dig_H1;  int16_t dig_H2; uint8_t dig_H3;
	int16_t dig_H4;  int16_t dig_H5; int8_t dig_H6;

	uint8_t raw_data[8];
	int32_t t_fine;
	float temperature;
	float pressure;
	float humidity;
};

#define max_instances 2
static struct bme280_t sensor_pool[max_instances];
uint8_t next_free_index = 0;


// --- YARDIMCI KOMPANZASYON FONKSİYONLARI (Fizik & Matematik) ---

static int32_t inline compensate_temperature(bme280_handle_t dev, int32_t adc_T){
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t inline compensate_pressure(bme280_handle_t dev, int32_t adc_P){
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    if (var1 == 0) { return 0; }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
    return (uint32_t)p;
}

static uint32_t inline compensate_humidity(bme280_handle_t dev, int32_t adc_H){
    int32_t v_x1_u32r;
    v_x1_u32r = (dev->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - ((((int32_t)dev->dig_H4) << 20) + (((int32_t)dev->dig_H5) * v_x1_u32r))) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dev->dig_H6)) >> 10) *
                 (((v_x1_u32r * ((int32_t)dev->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                 ((int32_t)2097152)) * ((int32_t)dev->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}


// --- ANA FONKSİYONLAR ---

bme280_return_status bme280_soft_reset(bme280_handle_t dev){
	uint8_t buffer = 0xB6;

	if(i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_reset, &buffer, 1) != _i2c_manager_ok) {
        return _bme280_fail;
    }

	while(1){
		if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_status, &buffer, 1) != _i2c_manager_ok) return _bme280_fail;
		if((buffer & 0x01) == 0) break;
        osDelay(2);
	}
	return _bme280_ok;
}


bme280_handle_t bme280_init(bme280_user_configs* config){
	if(config == NULL) return NULL;
	if(next_free_index >= max_instances) return NULL;

	bme280_handle_t dev = &sensor_pool[next_free_index];
	next_free_index++;

	dev->config_t.raw = config->config_t.raw;
	dev->ctrlhum_t.raw = config->ctrlhum_t.raw;
	dev->ctrlmeas_t.raw = config->ctrlmeas_t.raw;
	dev->i2c_addr = config->i2c_addr;
	dev->i2c_handle = config->i2c_handle;
	dev->dma_handle = config->dma_handle;
	dev->dma_stream = config->dma_stream;

	if(i2c_manager_check_device(dev->i2c_handle, dev->i2c_addr, addr_chip_id, 0x60) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	if(bme280_soft_reset(dev) != _bme280_ok){
		next_free_index--; return NULL;
	}

	uint8_t comp_buffer[24];
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_dig_t1, comp_buffer, 24) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	dev->dig_T1 = (uint16_t) (comp_buffer[1] << 8) | comp_buffer[0];
	dev->dig_T2 = (int16_t) (comp_buffer[3] << 8) | comp_buffer[2];
	dev->dig_T3 = (int16_t) (comp_buffer[5] << 8) | comp_buffer[4];
	dev->dig_P1 = (uint16_t) (comp_buffer[7] << 8) | comp_buffer[6];
	dev->dig_P2 = (int16_t) (comp_buffer[9] << 8) | comp_buffer[8];
	dev->dig_P3 = (int16_t) (comp_buffer[11] << 8) | comp_buffer[10];
	dev->dig_P4 = (int16_t) (comp_buffer[13] << 8) | comp_buffer[12];
	dev->dig_P5 = (int16_t) (comp_buffer[15] << 8) | comp_buffer[14];
	dev->dig_P6 = (int16_t) (comp_buffer[17] << 8) | comp_buffer[16];
	dev->dig_P7 = (int16_t) (comp_buffer[19] << 8) | comp_buffer[18];
	dev->dig_P8 = (int16_t) (comp_buffer[21] << 8) | comp_buffer[20];
	dev->dig_P9 = (int16_t) (comp_buffer[23] << 8) | comp_buffer[22];

	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_dig_h1, &(dev->dig_H1), 1) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_dig_h2, comp_buffer, 7) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	dev->dig_H2 = (int16_t) (comp_buffer[1] << 8) | comp_buffer[0];
	dev->dig_H3 = comp_buffer[2];
	dev->dig_H4 = (int16_t) (comp_buffer[3] << 4) | (comp_buffer[4] & 0b00001111);
	dev->dig_H5 = (int16_t) (comp_buffer[4] >> 4) | (comp_buffer[5] << 4);
	dev->dig_H6 = (int8_t) comp_buffer[6];

	if(i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_ctrlhum, &(dev->ctrlhum_t.raw), 1) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	if(i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_config, &(dev->config_t.raw), 1) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	if(i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_ctrlmeas, &(dev->ctrlmeas_t.raw), 1) != _i2c_manager_ok){
		next_free_index--; return NULL;
	}

	return dev;
}

bme280_return_status bme280_read_data_poll(bme280_handle_t dev){
    if(dev == NULL) return _bme280_fail;

    if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_press_msb, dev->raw_data, 8) != _i2c_manager_ok){
        return _bme280_read_fail;
    }
    return _bme280_ok;
}

bme280_return_status bme280_get_value(bme280_handle_t dev){
	if(dev == NULL) return _bme280_fail;

	int32_t adc_P = (int32_t)(((uint32_t)dev->raw_data[0] << 12) | ((uint32_t)dev->raw_data[1] << 4) | ((uint32_t)dev->raw_data[2] >> 4));
	int32_t adc_T = (int32_t)(((uint32_t)dev->raw_data[3] << 12) | ((uint32_t)dev->raw_data[4] << 4) | ((uint32_t)dev->raw_data[5] >> 4));
	int32_t adc_H = (int32_t)(((uint32_t)dev->raw_data[6] << 8) | (uint32_t)dev->raw_data[7]);

	int32_t temp_int = compensate_temperature(dev, adc_T);
	dev->temperature = (float)temp_int / 100.0f;

	uint32_t press_int = compensate_pressure(dev, adc_P);
	dev->pressure = (float)press_int / 256.0f;

	uint32_t hum_int = compensate_humidity(dev, adc_H);
	dev->humidity = (float)hum_int / 1024.0f;

	return _bme280_ok;
}

void bme280_get_measurements(bme280_handle_t dev, float* temperature,
		float* humidity, float* pressure){
	if(dev == NULL) return;
	if(temperature != NULL) *temperature = dev->temperature;
	if(humidity != NULL) *humidity = dev->humidity;
	if(pressure != NULL) *pressure = dev->pressure;
}

bme280_return_status bme280_read_data_dma(bme280_handle_t dev){
    if(dev == NULL) return _bme280_fail;

    if(i2c_manager_read_dma(dev->dma_handle, dev->dma_stream, dev->i2c_handle, dev->i2c_addr, addr_press_msb, dev->raw_data, 8) == _i2c_manager_ok){
        return _bme280_ok;
    }
    return _bme280_fail;
}

bme280_return_status bme280_build_read_job(bme280_handle_t dev, osThreadId_t notify_task,
        uint32_t notify_flag, uint32_t error_flag, i2c_job_t* out_job){
	if(dev == NULL || out_job == NULL) return _bme280_fail;

	out_job->operation = I2C_JOB_READ_DMA;
	out_job->i2c_handle = dev->i2c_handle;
	out_job->dma_handle = dev->dma_handle;
	out_job->dma_stream = dev->dma_stream;
	out_job->dev_addr = dev->i2c_addr;
	out_job->reg_addr = addr_press_msb;
	out_job->rxdata = dev->raw_data;
	out_job->txdata = NULL;
	out_job->size = 8;
	out_job->expected_device_id = 0;
	out_job->completion_result = NULL;
	out_job->notify_task = notify_task;
	out_job->notify_flag = notify_flag;
	out_job->error_flag = error_flag;

	return _bme280_ok;
}
