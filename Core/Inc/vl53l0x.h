#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

#include "i2c_manager.h"
#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_I2C_ADDRESS (0x29 << 1)

typedef enum{
    VL53L0X_OK = 0,
    VL53L0X_INVALID_ARGUMENT,
    VL53L0X_BUS_ERROR,
    VL53L0X_WRONG_DEVICE_ID,
    VL53L0X_TIMEOUT,
    VL53L0X_NOT_READY
}vl53l0x_status_t;

typedef struct{
    I2C_TypeDef* i2c_handle;
    DMA_TypeDef* dma_handle;
    uint32_t dma_stream;
    uint8_t i2c_address;
}vl53l0x_config_t;

typedef struct{
    uint16_t distance_mm;
    uint8_t range_status;
    bool valid;
}vl53l0x_measurement_t;

typedef struct{
    volatile uint8_t init_stage;
    volatile uint8_t attempted_i2c_address;
    volatile uint32_t init_attempt_count;
    volatile uint8_t model_id;
    volatile uint8_t module_type;
    volatile uint8_t revision_id;
    volatile uint8_t stop_variable;
    volatile uint8_t last_range_status;
    volatile uint16_t last_distance_mm;
    volatile uint32_t ready_poll_count;
    volatile uint32_t ready_count;
    volatile uint32_t dma_read_count;
    volatile uint32_t measurement_count;
    volatile uint32_t timeout_count;
    volatile uint32_t bus_error_count;
    volatile i2c_manager_return_status last_i2c_status;
    volatile i2c_manager_phase_t last_i2c_error_phase;
    volatile uint32_t last_i2c_sr1;
    volatile uint32_t last_i2c_sr2;
    volatile vl53l0x_status_t last_status;
}vl53l0x_debug_t;

struct vl53l0x_t;
typedef struct vl53l0x_t* vl53l0x_handle_t;

extern volatile vl53l0x_debug_t g_vl53l0x_debug;

vl53l0x_handle_t vl53l0x_init(const vl53l0x_config_t* config);
vl53l0x_status_t vl53l0x_is_data_ready(vl53l0x_handle_t device,
        bool* ready);
vl53l0x_status_t vl53l0x_build_read_job(vl53l0x_handle_t device,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job);
vl53l0x_status_t vl53l0x_process_data(vl53l0x_handle_t device,
        vl53l0x_measurement_t* measurement);
vl53l0x_status_t vl53l0x_clear_interrupt(vl53l0x_handle_t device);

#endif /* INC_VL53L0X_H_ */
