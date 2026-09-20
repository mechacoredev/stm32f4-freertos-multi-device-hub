/*
 * mpu6500.h
 *
 *  Created on: Jul 23, 2026
 *      Author: Enes
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "i2c_manager.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"

struct mpu6500_t;
typedef struct mpu6500_t* mpu6500_handle_t;

typedef enum{
	_mpu6500_ok = 0,
	_mpu6500_fail,
	_mpu6500_read_fail,
	_mpu6500_write_fail,
	_mpu6500_timeout,
	_mpu6500_device_not_found,
	_mpu6500_i2c_busy,
	_mpu6500_uninited_device,
	_mpu6500_not_safe_to_use,
}mpu6500_return_status;

typedef enum{
	mpu6500_i2c_addr_0 = (0x68 << 1),
	mpu6500_i2c_addr_1 = (0x69 << 1),
}mpu6500_i2c_addr;

typedef union{
	uint8_t raw;
	struct{
		uint8_t dlpf_cfg: 3;
		uint8_t ext_sync_set: 3;
		uint8_t reserved: 2;
	}bits;
}mpu6500_reg_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t fs_sel: 2;
		uint8_t reserved2: 3;
	}bits;
}mpu6500_reg_gyro_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t afs_sel: 2;
		uint8_t za_st: 1;
		uint8_t ya_st: 1;
		uint8_t xa_st: 1;
	}bits;
}mpu6500_reg_accel_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t slv0_fifo_en: 1;
		uint8_t slv1_fifo_en: 1;
		uint8_t slv2_fifo_en: 1;
		uint8_t accel_fifo_en: 1;
		uint8_t zg_fifo_en: 1;
		uint8_t yg_fifo_en: 1;
		uint8_t xg_fifo_en: 1;
		uint8_t temp_fifo_en: 1;
	}bits;
}mpu6500_reg_fifo_en_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 1;
		uint8_t i2c_bypass_en: 1;
		uint8_t fsync_int_en: 1;
		uint8_t fsync_int_level: 1;
		uint8_t int_rd_clear: 1;
		uint8_t latch_int_en: 1;
		uint8_t int_open: 1;
		uint8_t int_level: 1;
	}bits;
}mpu6500_reg_int_pin_cfg_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t data_rdy_en: 1;
		uint8_t reserved: 2;
		uint8_t i2c_mst_en: 1;
		uint8_t fifo_oflow_en: 1;
		uint8_t reserved2: 3;
	}bits;
}mpu6500_reg_int_enable_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t sig_cond_reset: 1;
		uint8_t i2c_mst_reset: 1;
		uint8_t fifo_reset: 1;
		uint8_t reserved: 1;
		uint8_t i2c_if_dis: 1;
		uint8_t i2c_mst_en: 1;
		uint8_t fifo_en: 1;
		uint8_t reserved2: 1;
	}bits;
}mpu6500_reg_user_ctrl_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t clksel: 3;
		uint8_t temp_dis: 1;
		uint8_t reserved: 1;
		uint8_t cycle: 1;
		uint8_t sleep: 1;
		uint8_t device_reset: 1;
	}bits;
}mpu6500_reg_pwr_mgmt_1_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t stby_zg: 1;
		uint8_t stby_yg: 1;
		uint8_t stby_xg: 1;
		uint8_t stby_za: 1;
		uint8_t stby_ya: 1;
		uint8_t stby_xa: 1;
		uint8_t lp_wake_ctrl: 2;
	}bits;
}mpu6500_reg_pwr_mgmt_2_t;

typedef struct{
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

}mpu6500_user_configs;

mpu6500_return_status mpu6500_read_data_poll(mpu6500_handle_t dev);
mpu6500_return_status mpu6500_read_data_poll_fifo(mpu6500_handle_t dev);
mpu6500_handle_t mpu6500_init(mpu6500_user_configs* config);
mpu6500_return_status mpu6500_configurate(mpu6500_handle_t dev);
mpu6500_return_status mpu6500_find_or_check_device(mpu6500_user_configs* config);

void mpu6500_assign_interrupt_task(mpu6500_handle_t dev, osThreadId_t task_handle, uint32_t flag);
void mpu6500_irq_handler(void);
void mpu6500_set_active_irq_device(mpu6500_handle_t dev);
extern volatile uint32_t mpu6500_irq_count;
mpu6500_return_status mpu6500_get_values(mpu6500_handle_t dev);
extern volatile uint8_t g_mpu6500_raw_debug[14];
extern volatile uint8_t g_mpu6500_last_corrupt_raw[14];
extern volatile uint32_t g_mpu6500_corrupt_frame_count;
extern volatile uint32_t g_mpu6500_consecutive_corrupt_frame_count;
void mpu6500_get_accel_g(mpu6500_handle_t dev, float* out_xyz);
void mpu6500_get_gyro_dps(mpu6500_handle_t dev, float* out_xyz);

mpu6500_return_status mpu6500_reset_fifo(mpu6500_handle_t dev);

typedef struct{
	uint8_t raw;
	bool data_rdy;
	bool fifo_overflow;
}mpu6500_int_status_t;

// --- Durum okuma (poll, blocking ama tek byte, hızlı) ---
mpu6500_return_status mpu6500_read_int_status(mpu6500_handle_t dev, mpu6500_int_status_t* out_status);
mpu6500_return_status mpu6500_read_fifo_count(mpu6500_handle_t dev, uint16_t* out_count);

// --- DMA başlatma (non-blocking, mutex'i tutarak döner - bme280 modeliyle aynı) ---
mpu6500_return_status mpu6500_read_data_dma(mpu6500_handle_t dev);
mpu6500_return_status mpu6500_read_fifo_dma(mpu6500_handle_t dev, uint16_t byte_count);

// --- DMA tamamlandıktan sonra FIFO buffer'ından en güncel frame'i çıkar ---
mpu6500_return_status mpu6500_fifo_extract_latest(mpu6500_handle_t dev, uint16_t byte_count);

mpu6500_return_status mpu6500_build_data_job(mpu6500_handle_t dev, osThreadId_t notify_task,
        uint32_t notify_flag, uint32_t error_flag, i2c_job_t* out_job);
mpu6500_return_status mpu6500_build_fifo_job(mpu6500_handle_t dev, uint16_t byte_count,
        osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
        i2c_job_t* out_job);

#endif /* INC_MPU6500_H_ */
