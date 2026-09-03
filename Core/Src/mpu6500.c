/*
 * mpu6500.c
 *
 *  Created on: Jul 23, 2026
 *      Author: Enes
 */

#include "mpu6500.h"
#include "string.h"

typedef enum{
	addr_who_am_i = 0x75,
	addr_accel_xout_h = 0x3B,
	addr_smplrt_div = 0x19,
	addr_config = 0x1A,
	addr_gyro_config = 0x1B,
	addr_accel_config = 0x1C,
	addr_signal_path_reset = 0x68,
	addr_fifo_en = 0x23,
	addr_int_pin_cfg = 0x37,
	addr_int_enable = 0x38,
	addr_int_status = 0x3A,
	addr_user_ctrl = 0x6A,
	addr_pwr_mgmt_1 = 0x6B,
	addr_pwr_mgmt_2 = 0x6C,
	addr_fifo_count_h = 0x72,
	addr_fifo_count_l = 0x73,
	addr_fifo_r_w = 0x74,
}mpu6500_register_address;

struct mpu6500_t{
	bool is_this_device_safe_to_use;

	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	DMA_TypeDef* dma_handle;
	uint32_t dma_stream;

	uint8_t sample_rate;
	mpu6500_reg_config_t config_t;
	mpu6500_reg_gyro_config_t gyro_config_t;
	mpu6500_reg_accel_config_t accel_config_t;
	mpu6500_reg_fifo_en_t fifo_en_t;
	mpu6500_reg_int_pin_cfg_t int_pin_cfg_t;
	mpu6500_reg_int_enable_t int_enable_t;
	mpu6500_reg_user_ctrl_t user_ctrl_t;
	mpu6500_reg_pwr_mgmt_1_t pwr_mgmt_1_t;
	mpu6500_reg_pwr_mgmt_2_t pwr_mgmt_2_t;

	uint8_t raw_data[14];
	uint8_t burst_buffer[1022]; // FIFO burst okuma için (20 frame'e kadar)

	uint8_t interrupt_status;
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	int16_t temp_raw;

	float accel_g[3];
	float gyro_dps[3];
	float temperature_c;

	osThreadId_t interrupt_task_handle;
	uint32_t interrupt_flag;

};

#define mpu6500_max_instances 2
static struct mpu6500_t sensor_pool[mpu6500_max_instances];
uint8_t next_free_index_mpu6500 = 0;

mpu6500_return_status mpu6500_read_data_poll(mpu6500_handle_t dev){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_accel_xout_h, dev->raw_data, 14) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}
	return _mpu6500_ok;
}

mpu6500_handle_t mpu6500_init(mpu6500_user_configs* config){
	if(config == NULL) return NULL;

	if(next_free_index_mpu6500 >= mpu6500_max_instances) return NULL;

	/* check_device START urettikten sonra BUSY=1 olmasi normaldir. Manager,
	 * STOP tamamlanana kadar mutexi birakmaz ve hata anini diagnostics'e yazar. */
	mpu6500_return_status status = mpu6500_find_or_check_device(config);
	if(status != _mpu6500_ok) return NULL;

	mpu6500_handle_t dev = &sensor_pool[next_free_index_mpu6500];
	next_free_index_mpu6500++;

	dev->i2c_handle = config->i2c_handle;
	dev->i2c_addr = config->i2c_addr;
	dev->dma_handle = config->dma_handle;
	dev->dma_stream = config->dma_stream;
	dev->sample_rate = config->sample_rate;
	dev->config_t.raw = config->config_t.raw;
	dev->gyro_config_t.raw = config->gyro_config_t.raw;
	dev->accel_config_t.raw = config->accel_config_t.raw;
	dev->fifo_en_t.raw = config->fifo_en_t.raw;
	dev->int_pin_cfg_t.raw = config->int_pin_cfg_t.raw;
	dev->int_enable_t.raw = config->int_enable_t.raw;
	dev->user_ctrl_t.raw = config->user_ctrl_t.raw;
	dev->pwr_mgmt_1_t.raw = config->pwr_mgmt_1_t.raw;
	dev->pwr_mgmt_2_t.raw = config->pwr_mgmt_2_t.raw;

	dev->is_this_device_safe_to_use = true;

	// Cihazı tam donanımsal reset'ten geçir (INT pini ve tüm register'lar temiz başlasın)
	uint8_t device_reset = 0x80;
	i2c_manager_return_status r1 = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_pwr_mgmt_1, &device_reset, 1);
	osDelay(100);

	uint8_t signal_reset = 0x07;
	i2c_manager_return_status r2 = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_signal_path_reset, &signal_reset, 1);
	osDelay(100);

	if(r1 != _i2c_manager_ok || r2 != _i2c_manager_ok){
		next_free_index_mpu6500--; // pool'daki yeri geri ver
		return NULL; // reset başarısız oldu, çağıran taraf karar versin
	}

	return dev;
}

mpu6500_return_status mpu6500_configurate(mpu6500_handle_t dev){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	i2c_manager_return_status status;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_smplrt_div, &dev->sample_rate, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_config, &dev->config_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_gyro_config, &dev->gyro_config_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_accel_config, &dev->accel_config_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_fifo_en, &dev->fifo_en_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_int_pin_cfg, &dev->int_pin_cfg_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_int_enable, &dev->int_enable_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_pwr_mgmt_1, &dev->pwr_mgmt_1_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_pwr_mgmt_2, &dev->pwr_mgmt_2_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_user_ctrl, &dev->user_ctrl_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_find_or_check_device(mpu6500_user_configs* config){
	if(config == NULL) return _mpu6500_uninited_device;
	if(i2c_manager_check_device(config->i2c_handle, config->i2c_addr, addr_who_am_i, 0x70) == _i2c_manager_ok){ // bizim mpu6500 0x70 döndürüyor
		return _mpu6500_ok;
	}
	return _mpu6500_device_not_found;
}

volatile uint8_t g_mpu6500_raw_debug[14];
volatile uint8_t g_mpu6500_last_corrupt_raw[14];
volatile uint32_t g_mpu6500_corrupt_frame_count = 0;
volatile uint32_t g_mpu6500_consecutive_corrupt_frame_count = 0;

mpu6500_return_status mpu6500_get_values(mpu6500_handle_t dev){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;

	bool tail_is_all_ff = true;
	for(uint8_t index = 0; index < 14; index++){
		g_mpu6500_raw_debug[index] = dev->raw_data[index];
		if(index > 0 && dev->raw_data[index] != 0xFF){
			tail_is_all_ff = false;
		}
	}

	/* Logic-analyzer kaydinda bozuk transferlerin ortak imzasi, ilk byte
	 * sonrasindaki 13 byte'in tamaminda SDA'nin HIGH kalmasiydi. Bu bir
	 * olcum degildir: Y/Z, sicaklik ve gyro alanlari ayni anda 0xFFFF olur.
	 * Son gecerli ekrani koru ve olayi ayri bir sayacta gorunur yap. */
	if(tail_is_all_ff){
		for(uint8_t index = 0; index < 14; index++){
			g_mpu6500_last_corrupt_raw[index] = dev->raw_data[index];
		}
		g_mpu6500_corrupt_frame_count++;
		g_mpu6500_consecutive_corrupt_frame_count++;
		return _mpu6500_read_fail;
	}
	g_mpu6500_consecutive_corrupt_frame_count = 0;

	dev->accel_raw[0] = (int16_t)((dev->raw_data[0] << 8) | dev->raw_data[1]);
	dev->accel_raw[1] = (int16_t)((dev->raw_data[2] << 8) | dev->raw_data[3]);
	dev->accel_raw[2] = (int16_t)((dev->raw_data[4] << 8) | dev->raw_data[5]);

	dev->temp_raw = (int16_t)((dev->raw_data[6] << 8) | dev->raw_data[7]);

	dev->gyro_raw[0] = (int16_t)((dev->raw_data[8] << 8) | dev->raw_data[9]);
	dev->gyro_raw[1] = (int16_t)((dev->raw_data[10] << 8) | dev->raw_data[11]);
	dev->gyro_raw[2] = (int16_t)((dev->raw_data[12] << 8) | dev->raw_data[13]);

	float accel_sensitivity = 16384.0f / (float)(1 << dev->accel_config_t.bits.afs_sel);
	dev->accel_g[0] = (float)dev->accel_raw[0] / accel_sensitivity;
	dev->accel_g[1] = (float)dev->accel_raw[1] / accel_sensitivity;
	dev->accel_g[2] = (float)dev->accel_raw[2] / accel_sensitivity;

	float gyro_sensitivity = 131.0f / (float)(1 << dev->gyro_config_t.bits.fs_sel);
	dev->gyro_dps[0] = (float)dev->gyro_raw[0] / gyro_sensitivity;
	dev->gyro_dps[1] = (float)dev->gyro_raw[1] / gyro_sensitivity;
	dev->gyro_dps[2] = (float)dev->gyro_raw[2] / gyro_sensitivity;

	dev->temperature_c = (float)dev->temp_raw / 340.0f + 36.53f;

	return _mpu6500_ok;
}

void mpu6500_get_accel_g(mpu6500_handle_t dev, float* out_xyz){
	out_xyz[0] = dev->accel_g[0];
	out_xyz[1] = dev->accel_g[1];
	out_xyz[2] = dev->accel_g[2];
}

void mpu6500_get_gyro_dps(mpu6500_handle_t dev, float* out_xyz){
	out_xyz[0] = dev->gyro_dps[0];
	out_xyz[1] = dev->gyro_dps[1];
	out_xyz[2] = dev->gyro_dps[2];
}

void mpu6500_assign_interrupt_task(mpu6500_handle_t dev, osThreadId_t task_handle, uint32_t flag){
	dev->interrupt_task_handle = task_handle;
	dev->interrupt_flag = flag;
}

static mpu6500_handle_t active_irq_device = NULL;
volatile uint32_t mpu6500_irq_count = 0;

void mpu6500_set_active_irq_device(mpu6500_handle_t dev){
	active_irq_device = dev;
}

void mpu6500_irq_handler(void){
	mpu6500_irq_count++;
	if(active_irq_device == NULL) return;
	if(active_irq_device->interrupt_task_handle == NULL) return;

	osThreadFlagsSet(active_irq_device->interrupt_task_handle, active_irq_device->interrupt_flag);
}

mpu6500_return_status mpu6500_read_data_poll_fifo(mpu6500_handle_t dev){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;

	uint8_t fifo_count_buf[2];
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_fifo_count_h, fifo_count_buf, 2) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}

	uint16_t fifo_count = ((uint16_t)fifo_count_buf[0] << 8) | fifo_count_buf[1];

	if(fifo_count < 14){
		return _mpu6500_fail; // henüz tam bir frame birikmemiş
	}

	uint16_t frames_available = fifo_count / 14;
	uint16_t bytes_to_read = frames_available * 14;

	if(bytes_to_read > sizeof(dev->burst_buffer)){
		bytes_to_read = (sizeof(dev->burst_buffer) / 14) * 14;
	}

	// Tek transaction'da mevcut tüm frame'leri oku
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_fifo_r_w, dev->burst_buffer, bytes_to_read) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}

	// En son (en güncel) frame'i kullan
	memcpy(dev->raw_data, &dev->burst_buffer[bytes_to_read - 14], 14);

	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_reset_fifo(mpu6500_handle_t dev){
	i2c_manager_return_status status;

	uint8_t user_ctrl_fifo_disabled = dev->user_ctrl_t.raw & ~0x40;
	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_user_ctrl, &user_ctrl_fifo_disabled, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	uint8_t user_ctrl_reset = user_ctrl_fifo_disabled | 0x04;
	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_user_ctrl, &user_ctrl_reset, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	status = i2c_manager_write_poll(dev->i2c_handle, dev->i2c_addr, addr_user_ctrl, &dev->user_ctrl_t.raw, 1);
	if(status != _i2c_manager_ok) return _mpu6500_write_fail;

	return _mpu6500_ok;
}



mpu6500_return_status mpu6500_read_int_status(mpu6500_handle_t dev, mpu6500_int_status_t* out_status){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;

	uint8_t raw;
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_int_status, &raw, 1) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}

	if(out_status != NULL){
		out_status->raw = raw;
		out_status->data_rdy = (raw & 0x01) != 0;      // DATA_RDY_INT bit0
		out_status->fifo_overflow = (raw & 0x10) != 0; // FIFO_OFLOW_INT bit4
	}

	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_read_fifo_count(mpu6500_handle_t dev, uint16_t* out_count){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;

	uint8_t buf[2];
	if(i2c_manager_read_poll(dev->i2c_handle, dev->i2c_addr, addr_fifo_count_h, buf, 2) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}

	*out_count = ((uint16_t)buf[0] << 8) | buf[1];
	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_read_data_dma(mpu6500_handle_t dev){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	if(dev->dma_handle == NULL) return _mpu6500_uninited_device;

	if(i2c_manager_read_dma(dev->dma_handle, dev->dma_stream, dev->i2c_handle, dev->i2c_addr,
	                        addr_accel_xout_h, dev->raw_data, 14) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}
	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_read_fifo_dma(mpu6500_handle_t dev, uint16_t byte_count){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	if(dev->dma_handle == NULL) return _mpu6500_uninited_device;
	if(byte_count == 0 || byte_count > sizeof(dev->burst_buffer)) return _mpu6500_fail;

	if(i2c_manager_read_dma(dev->dma_handle, dev->dma_stream, dev->i2c_handle, dev->i2c_addr,
	                        addr_fifo_r_w, dev->burst_buffer, byte_count) != _i2c_manager_ok){
		return _mpu6500_read_fail;
	}
	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_fifo_extract_latest(mpu6500_handle_t dev, uint16_t byte_count){
	if(byte_count < 14 || byte_count > sizeof(dev->burst_buffer)) return _mpu6500_fail;
	memcpy(dev->raw_data, &dev->burst_buffer[byte_count - 14], 14);
	return _mpu6500_ok;
}


// mpu6500.c'ye ekleyin:
mpu6500_return_status mpu6500_build_data_job(mpu6500_handle_t dev, osThreadId_t notify_task,
        uint32_t notify_flag, uint32_t error_flag, i2c_job_t* out_job){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	if(dev->dma_handle == NULL) return _mpu6500_uninited_device;
	if(out_job == NULL) return _mpu6500_fail;

	out_job->operation = I2C_JOB_READ_DMA;
	out_job->i2c_handle = dev->i2c_handle;
	out_job->dma_handle = dev->dma_handle;
	out_job->dma_stream = dev->dma_stream;
	out_job->dev_addr = dev->i2c_addr;
	out_job->reg_addr = addr_accel_xout_h;
	out_job->rxdata = dev->raw_data;
	out_job->txdata = NULL;
	out_job->size = 14;
	out_job->expected_device_id = 0;
	out_job->completion_result = NULL;
	out_job->notify_task = notify_task;
	out_job->notify_flag = notify_flag;
	out_job->error_flag = error_flag;

	return _mpu6500_ok;
}

mpu6500_return_status mpu6500_build_fifo_job(mpu6500_handle_t dev, uint16_t byte_count,
        osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
        i2c_job_t* out_job){
	if(dev->is_this_device_safe_to_use != true) return _mpu6500_not_safe_to_use;
	if(dev->dma_handle == NULL) return _mpu6500_uninited_device;
	if(out_job == NULL || byte_count == 0 || byte_count > sizeof(dev->burst_buffer)) return _mpu6500_fail;

	out_job->operation = I2C_JOB_READ_DMA;
	out_job->i2c_handle = dev->i2c_handle;
	out_job->dma_handle = dev->dma_handle;
	out_job->dma_stream = dev->dma_stream;
	out_job->dev_addr = dev->i2c_addr;
	out_job->reg_addr = addr_fifo_r_w;
	out_job->rxdata = dev->burst_buffer;
	out_job->txdata = NULL;
	out_job->size = byte_count;
	out_job->expected_device_id = 0;
	out_job->completion_result = NULL;
	out_job->notify_task = notify_task;
	out_job->notify_flag = notify_flag;
	out_job->error_flag = error_flag;

	return _mpu6500_ok;
}
