#include "adxl345.h"
#include <string.h>

#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32

struct adxl345_t{
    I2C_TypeDef* i2c_handle;
    DMA_TypeDef* dma_handle;
    uint32_t dma_stream;
    uint8_t i2c_address;
    uint8_t device_id;
    uint8_t raw_data[6];
};

static struct adxl345_t adxl345_device;
static bool adxl345_in_use = false;

static bool write_register(adxl345_handle_t device, uint8_t reg,
        uint8_t value){
    return i2c_manager_write_poll(device->i2c_handle, device->i2c_address,
            reg, &value, 1) == _i2c_manager_ok;
}

adxl345_handle_t adxl345_init(const adxl345_config_t* config){
    if(config == NULL || config->i2c_handle == NULL ||
            config->dma_handle == NULL || adxl345_in_use) return NULL;

    memset(&adxl345_device, 0, sizeof(adxl345_device));
    adxl345_device.i2c_handle = config->i2c_handle;
    adxl345_device.dma_handle = config->dma_handle;
    adxl345_device.dma_stream = config->dma_stream;
    adxl345_device.i2c_address = config->i2c_address;

    if(i2c_manager_read_poll(config->i2c_handle, config->i2c_address,
            ADXL345_REG_DEVID, &adxl345_device.device_id, 1) !=
            _i2c_manager_ok) return NULL;
    if(adxl345_device.device_id != ADXL345_DEVICE_ID) return NULL;

    if(!write_register(&adxl345_device, ADXL345_REG_BW_RATE, 0x0A)) return NULL;
    if(!write_register(&adxl345_device, ADXL345_REG_DATA_FORMAT, 0x08)) return NULL;
    if(!write_register(&adxl345_device, ADXL345_REG_POWER_CTL, 0x08)) return NULL;

    adxl345_in_use = true;
    return &adxl345_device;
}

adxl345_status_t adxl345_build_read_job(adxl345_handle_t device,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job){
    if(device == NULL || notify_task == NULL || job == NULL ||
            success_flag == 0 || error_flag == 0 ||
            (success_flag & error_flag) != 0) return ADXL345_INVALID_ARGUMENT;

    memset(job, 0, sizeof(*job));
    job->operation = I2C_JOB_READ_DMA;
    job->i2c_handle = device->i2c_handle;
    job->dma_handle = device->dma_handle;
    job->dma_stream = device->dma_stream;
    job->dev_addr = device->i2c_address;
    job->reg_addr = ADXL345_REG_DATAX0;
    job->rxdata = device->raw_data;
    job->size = sizeof(device->raw_data);
    job->notify_task = notify_task;
    job->notify_flag = success_flag;
    job->error_flag = error_flag;
    return ADXL345_OK;
}

adxl345_status_t adxl345_process_data(adxl345_handle_t device,
        adxl345_data_t* output){
    if(device == NULL || output == NULL) return ADXL345_INVALID_ARGUMENT;

    output->raw_x = (int16_t)(((uint16_t)device->raw_data[1] << 8) |
            device->raw_data[0]);
    output->raw_y = (int16_t)(((uint16_t)device->raw_data[3] << 8) |
            device->raw_data[2]);
    output->raw_z = (int16_t)(((uint16_t)device->raw_data[5] << 8) |
            device->raw_data[4]);
    output->x_g = output->raw_x * 0.0039f;
    output->y_g = output->raw_y * 0.0039f;
    output->z_g = output->raw_z * 0.0039f;
    return ADXL345_OK;
}

uint8_t adxl345_get_device_id(adxl345_handle_t device){
    return device == NULL ? 0 : device->device_id;
}
