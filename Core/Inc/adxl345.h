#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "i2c_manager.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum{
    ADXL345_OK = 0,
    ADXL345_INVALID_ARGUMENT,
    ADXL345_BUS_ERROR,
    ADXL345_WRONG_DEVICE_ID
}adxl345_status_t;

typedef struct{
    I2C_TypeDef* i2c_handle;
    DMA_TypeDef* dma_handle;
    uint32_t dma_stream;
    uint8_t i2c_address;
}adxl345_config_t;

typedef struct{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float x_g;
    float y_g;
    float z_g;
}adxl345_data_t;

struct adxl345_t;
typedef struct adxl345_t* adxl345_handle_t;

#define ADXL345_I2C_ADDRESS_ALT_LOW  (0x53 << 1)
#define ADXL345_I2C_ADDRESS_ALT_HIGH (0x1D << 1)
#define ADXL345_DEVICE_ID 0xE5

adxl345_handle_t adxl345_init(const adxl345_config_t* config);
adxl345_status_t adxl345_build_read_job(adxl345_handle_t device,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job);
adxl345_status_t adxl345_process_data(adxl345_handle_t device,
        adxl345_data_t* output);
uint8_t adxl345_get_device_id(adxl345_handle_t device);

#endif /* INC_ADXL345_H_ */
