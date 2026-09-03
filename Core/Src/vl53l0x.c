#include "vl53l0x.h"
#include <string.h>

#define VL53L0X_REG_SYSRANGE_START                 0x00
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG         0x01
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO   0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR         0x0B
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS        0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS            0x14
#define VL53L0X_REG_FINAL_RANGE_SIGNAL_RATE_LIMIT  0x44
#define VL53L0X_REG_MSRC_CONFIG_CONTROL            0x60
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH        0x84
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP  0x89
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES     0xB0
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID        0xC0
#define VL53L0X_REG_IDENTIFICATION_MODULE_TYPE     0xC1
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID     0xC2

#define VL53L0X_MODEL_ID_EXPECTED    0xEE
#define VL53L0X_MODULE_TYPE_EXPECTED 0xAA
#define VL53L0X_REVISION_EXPECTED    0x10
#define VL53L0X_INTERNAL_TIMEOUT_MS  200
#define VL53L0X_RESULT_BLOCK_SIZE    12

typedef struct{
    uint8_t reg;
    uint8_t value;
}vl53l0x_register_value_t;

struct vl53l0x_t{
    vl53l0x_config_t config;
    uint8_t stop_variable;
    uint8_t result_block[VL53L0X_RESULT_BLOCK_SIZE];
};

static struct vl53l0x_t vl53l0x_device;
static bool vl53l0x_in_use = false;
volatile vl53l0x_debug_t g_vl53l0x_debug = {0};

static void capture_i2c_failure(vl53l0x_handle_t device,
        i2c_manager_return_status result){
    g_vl53l0x_debug.last_i2c_status = result;
    g_vl53l0x_debug.attempted_i2c_address = device->config.i2c_address;

    uint8_t diagnostic_index = 0;
    if(device->config.i2c_handle == I2C2) diagnostic_index = 1;
    else if(device->config.i2c_handle == I2C3) diagnostic_index = 2;

    g_vl53l0x_debug.last_i2c_error_phase =
            g_i2c_manager_diagnostics[diagnostic_index].last_error_phase;
    g_vl53l0x_debug.last_i2c_sr1 =
            g_i2c_manager_diagnostics[diagnostic_index].last_sr1;
    g_vl53l0x_debug.last_i2c_sr2 =
            g_i2c_manager_diagnostics[diagnostic_index].last_sr2;
}

static vl53l0x_status_t write_bytes(vl53l0x_handle_t device, uint8_t reg,
        uint8_t* data, uint16_t size){
    i2c_manager_return_status result = i2c_manager_write_poll(
            device->config.i2c_handle, device->config.i2c_address,
            reg, data, size);
    if(result == _i2c_manager_ok) return VL53L0X_OK;
    capture_i2c_failure(device, result);
    g_vl53l0x_debug.bus_error_count++;
    g_vl53l0x_debug.last_status = VL53L0X_BUS_ERROR;
    return VL53L0X_BUS_ERROR;
}

static vl53l0x_status_t write_u8(vl53l0x_handle_t device, uint8_t reg,
        uint8_t value){
    return write_bytes(device, reg, &value, 1);
}

static vl53l0x_status_t read_bytes(vl53l0x_handle_t device, uint8_t reg,
        uint8_t* data, uint16_t size){
    i2c_manager_return_status result = i2c_manager_read_poll(
            device->config.i2c_handle, device->config.i2c_address,
            reg, data, size);
    if(result == _i2c_manager_ok) return VL53L0X_OK;
    capture_i2c_failure(device, result);
    g_vl53l0x_debug.bus_error_count++;
    g_vl53l0x_debug.last_status = VL53L0X_BUS_ERROR;
    return VL53L0X_BUS_ERROR;
}

static vl53l0x_status_t read_u8(vl53l0x_handle_t device, uint8_t reg,
        uint8_t* value){
    return read_bytes(device, reg, value, 1);
}

static vl53l0x_status_t write_table(vl53l0x_handle_t device,
        const vl53l0x_register_value_t* table, uint16_t count){
    for(uint16_t index = 0; index < count; index++){
        vl53l0x_status_t status = write_u8(device, table[index].reg,
                table[index].value);
        if(status != VL53L0X_OK) return status;
    }
    return VL53L0X_OK;
}

static vl53l0x_status_t wait_register_mask(vl53l0x_handle_t device,
        uint8_t reg, uint8_t mask, bool wait_for_nonzero){
    uint32_t start_tick = osKernelGetTickCount();
    for(;;){
        uint8_t value = 0;
        vl53l0x_status_t status = read_u8(device, reg, &value);
        if(status != VL53L0X_OK) return status;
        bool nonzero = (value & mask) != 0;
        if(nonzero == wait_for_nonzero) return VL53L0X_OK;
        if((osKernelGetTickCount() - start_tick) >=
                VL53L0X_INTERNAL_TIMEOUT_MS){
            g_vl53l0x_debug.timeout_count++;
            g_vl53l0x_debug.last_status = VL53L0X_TIMEOUT;
            return VL53L0X_TIMEOUT;
        }
        osDelay(1);
    }
}

static vl53l0x_status_t get_spad_info(vl53l0x_handle_t device,
        uint8_t* count, bool* aperture){
    uint8_t value = 0;
    if(write_u8(device, 0x80, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x00, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x06) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(read_u8(device, 0x83, &value) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x83, value | 0x04) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x07) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x81, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x80, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x94, 0x6B) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x83, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(wait_register_mask(device, 0x83, 0xFF, true) != VL53L0X_OK)
        return g_vl53l0x_debug.last_status;
    if(write_u8(device, 0x83, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(read_u8(device, 0x92, &value) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    *count = value & 0x7F;
    *aperture = (value & 0x80) != 0;

    if(write_u8(device, 0x81, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x06) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(read_u8(device, 0x83, &value) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x83, value & (uint8_t)~0x04) != VL53L0X_OK)
        return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x00, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x80, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    return VL53L0X_OK;
}

static vl53l0x_status_t configure_reference_spads(vl53l0x_handle_t device){
    uint8_t spad_count = 0;
    bool aperture = false;
    vl53l0x_status_t status = get_spad_info(device, &spad_count, &aperture);
    if(status != VL53L0X_OK) return status;

    uint8_t enables[6] = {0};
    status = read_bytes(device, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES,
            enables, sizeof(enables));
    if(status != VL53L0X_OK) return status;
    if(write_u8(device, 0xFF, 0x01) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x4F, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x4E, 0x2C) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xFF, 0x00) != VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0xB6, 0xB4) != VL53L0X_OK) return VL53L0X_BUS_ERROR;

    uint8_t first_spad = 0;
    if(aperture) first_spad = 12;
    uint8_t enabled_count = 0;
    for(uint8_t index = 0; index < 48; index++){
        uint8_t byte_index = index / 8;
        uint8_t bit_mask = (uint8_t)(1 << (index % 8));
        if(index < first_spad || enabled_count >= spad_count){
            enables[byte_index] &= (uint8_t)~bit_mask;
        }else if((enables[byte_index] & bit_mask) != 0){
            enabled_count++;
        }
    }
    return write_bytes(device, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES,
            enables, sizeof(enables));
}

/* ST'nin VL53L0X API'sindeki varsayilan tuning ayarlari. Sensordeki kapali
 * firmware bu degerlerden sonra olcum dizisini kendi icinde yurutur. */
static const vl53l0x_register_value_t default_tuning[] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},
    {0x11,0x00},{0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},
    {0x4E,0x2C},{0x48,0x00},{0x30,0x20},{0xFF,0x00},{0x30,0x09},
    {0x54,0x00},{0x31,0x04},{0x32,0x03},{0x40,0x83},{0x46,0x25},
    {0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},{0x52,0x96},
    {0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},
    {0x49,0xFF},{0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},
    {0x78,0x21},{0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},
    {0x45,0x26},{0x46,0x05},{0x40,0x40},{0x0E,0x06},{0x20,0x1A},
    {0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},{0xFF,0x01},
    {0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},
    {0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},
    {0xFF,0x01},{0x0D,0x01},{0xFF,0x00},{0x80,0x01},{0x01,0xF8},
    {0xFF,0x01},{0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00}
};

static vl53l0x_status_t perform_reference_calibration(
        vl53l0x_handle_t device, uint8_t vhv_init_byte){
    vl53l0x_status_t status = write_u8(device,
            VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte);
    if(status != VL53L0X_OK) return status;
    status = wait_register_mask(device, VL53L0X_REG_RESULT_INTERRUPT_STATUS,
            0x07, true);
    if(status != VL53L0X_OK) return status;
    if(write_u8(device, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) !=
            VL53L0X_OK) return VL53L0X_BUS_ERROR;
    return write_u8(device, VL53L0X_REG_SYSRANGE_START, 0x00);
}

static vl53l0x_status_t start_continuous(vl53l0x_handle_t device){
    static const vl53l0x_register_value_t unlock[] = {
        {0x80,0x01},{0xFF,0x01},{0x00,0x00}
    };
    static const vl53l0x_register_value_t lock[] = {
        {0x00,0x01},{0xFF,0x00},{0x80,0x00}
    };
    if(write_table(device, unlock, sizeof(unlock) / sizeof(unlock[0])) !=
            VL53L0X_OK) return VL53L0X_BUS_ERROR;
    if(write_u8(device, 0x91, device->stop_variable) != VL53L0X_OK)
        return VL53L0X_BUS_ERROR;
    if(write_table(device, lock, sizeof(lock) / sizeof(lock[0])) !=
            VL53L0X_OK) return VL53L0X_BUS_ERROR;
    return write_u8(device, VL53L0X_REG_SYSRANGE_START, 0x02);
}

vl53l0x_handle_t vl53l0x_init(const vl53l0x_config_t* config){
    if(config == NULL || config->i2c_handle == NULL ||
            config->dma_handle == NULL || vl53l0x_in_use){
        g_vl53l0x_debug.last_status = VL53L0X_INVALID_ARGUMENT;
        return NULL;
    }

    uint32_t init_attempt_count = g_vl53l0x_debug.init_attempt_count + 1;
    memset(&vl53l0x_device, 0, sizeof(vl53l0x_device));
    memset((void*)&g_vl53l0x_debug, 0, sizeof(g_vl53l0x_debug));
    g_vl53l0x_debug.init_attempt_count = init_attempt_count;
    vl53l0x_device.config = *config;
    g_vl53l0x_debug.attempted_i2c_address = config->i2c_address;
    g_vl53l0x_debug.last_i2c_status = _i2c_manager_ok;
    g_vl53l0x_debug.init_stage = 1;

    /* XSHUT serbest birakildiktan sonra sensorun I2C arayuzunun acilmasi
     * icin gereken boot suresini garanti eder. */
    osDelay(5);
    osDelay(10);

    uint8_t model_id = 0;
    uint8_t module_type = 0;
    uint8_t revision_id = 0;
    if(read_u8(&vl53l0x_device, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
            &model_id) != VL53L0X_OK ||
       read_u8(&vl53l0x_device, VL53L0X_REG_IDENTIFICATION_MODULE_TYPE,
            &module_type) != VL53L0X_OK ||
       read_u8(&vl53l0x_device, VL53L0X_REG_IDENTIFICATION_REVISION_ID,
            &revision_id) != VL53L0X_OK){
        return NULL;
    }
    g_vl53l0x_debug.model_id = model_id;
    g_vl53l0x_debug.module_type = module_type;
    g_vl53l0x_debug.revision_id = revision_id;
    if(model_id != VL53L0X_MODEL_ID_EXPECTED ||
       module_type != VL53L0X_MODULE_TYPE_EXPECTED ||
       revision_id != VL53L0X_REVISION_EXPECTED){
        g_vl53l0x_debug.last_status = VL53L0X_WRONG_DEVICE_ID;
        return NULL;
    }
    g_vl53l0x_debug.init_stage = 2;

    uint8_t value = 0;
    if(read_u8(&vl53l0x_device, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP,
            &value) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP,
            value | 0x01) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, 0x88, 0x00) != VL53L0X_OK){
        return NULL;
    }

    static const vl53l0x_register_value_t stop_variable_access[] = {
        {0x80,0x01},{0xFF,0x01},{0x00,0x00}
    };
    static const vl53l0x_register_value_t stop_variable_close[] = {
        {0x00,0x01},{0xFF,0x00},{0x80,0x00}
    };
    if(write_table(&vl53l0x_device, stop_variable_access,
            sizeof(stop_variable_access) / sizeof(stop_variable_access[0])) !=
            VL53L0X_OK ||
       read_u8(&vl53l0x_device, 0x91, &vl53l0x_device.stop_variable) !=
            VL53L0X_OK ||
       write_table(&vl53l0x_device, stop_variable_close,
            sizeof(stop_variable_close) / sizeof(stop_variable_close[0])) !=
            VL53L0X_OK){
        return NULL;
    }
    g_vl53l0x_debug.stop_variable = vl53l0x_device.stop_variable;

    if(read_u8(&vl53l0x_device, VL53L0X_REG_MSRC_CONFIG_CONTROL, &value) !=
            VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_MSRC_CONFIG_CONTROL,
            value | 0x12) != VL53L0X_OK){
        return NULL;
    }
    uint8_t signal_limit[2] = {0x00, 0x20};
    if(write_bytes(&vl53l0x_device, VL53L0X_REG_FINAL_RANGE_SIGNAL_RATE_LIMIT,
            signal_limit, sizeof(signal_limit)) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF) !=
            VL53L0X_OK){
        return NULL;
    }
    g_vl53l0x_debug.init_stage = 3;

    if(configure_reference_spads(&vl53l0x_device) != VL53L0X_OK ||
       write_table(&vl53l0x_device, default_tuning,
            sizeof(default_tuning) / sizeof(default_tuning[0])) != VL53L0X_OK){
        return NULL;
    }
    g_vl53l0x_debug.init_stage = 4;

    if(write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO,
            0x04) != VL53L0X_OK ||
       read_u8(&vl53l0x_device, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
            &value) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
            value & (uint8_t)~0x10) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) !=
            VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) !=
            VL53L0X_OK){
        return NULL;
    }

    if(write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01) !=
            VL53L0X_OK ||
       perform_reference_calibration(&vl53l0x_device, 0x40) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02) !=
            VL53L0X_OK ||
       perform_reference_calibration(&vl53l0x_device, 0x00) != VL53L0X_OK ||
       write_u8(&vl53l0x_device, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8) !=
            VL53L0X_OK){
        return NULL;
    }
    g_vl53l0x_debug.init_stage = 5;

    if(start_continuous(&vl53l0x_device) != VL53L0X_OK) return NULL;
    vl53l0x_in_use = true;
    g_vl53l0x_debug.init_stage = 6;
    g_vl53l0x_debug.last_status = VL53L0X_OK;
    return &vl53l0x_device;
}

vl53l0x_status_t vl53l0x_is_data_ready(vl53l0x_handle_t device,
        bool* ready){
    if(device == NULL || ready == NULL) return VL53L0X_INVALID_ARGUMENT;
    uint8_t status = 0;
    g_vl53l0x_debug.ready_poll_count++;
    vl53l0x_status_t result = read_u8(device,
            VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status);
    if(result != VL53L0X_OK) return result;
    *ready = (status & 0x07) != 0;
    if(*ready) g_vl53l0x_debug.ready_count++;
    return VL53L0X_OK;
}

vl53l0x_status_t vl53l0x_build_read_job(vl53l0x_handle_t device,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job){
    if(device == NULL || notify_task == NULL || job == NULL ||
       success_flag == 0 || error_flag == 0 ||
       (success_flag & error_flag) != 0) return VL53L0X_INVALID_ARGUMENT;

    memset(job, 0, sizeof(*job));
    job->operation = I2C_JOB_READ_DMA;
    job->i2c_handle = device->config.i2c_handle;
    job->dma_handle = device->config.dma_handle;
    job->dma_stream = device->config.dma_stream;
    job->dev_addr = device->config.i2c_address;
    job->reg_addr = VL53L0X_REG_RESULT_RANGE_STATUS;
    job->rxdata = device->result_block;
    job->size = sizeof(device->result_block);
    job->notify_task = notify_task;
    job->notify_flag = success_flag;
    job->error_flag = error_flag;
    g_vl53l0x_debug.dma_read_count++;
    return VL53L0X_OK;
}

vl53l0x_status_t vl53l0x_process_data(vl53l0x_handle_t device,
        vl53l0x_measurement_t* measurement){
    if(device == NULL || measurement == NULL) return VL53L0X_INVALID_ARGUMENT;
    measurement->range_status = (device->result_block[0] & 0x78) >> 3;
    measurement->distance_mm = (uint16_t)(
            ((uint16_t)device->result_block[10] << 8) |
            device->result_block[11]);
    measurement->valid = measurement->distance_mm > 0 &&
            measurement->distance_mm < 8190;
    g_vl53l0x_debug.last_range_status = measurement->range_status;
    g_vl53l0x_debug.last_distance_mm = measurement->distance_mm;
    g_vl53l0x_debug.measurement_count++;
    g_vl53l0x_debug.last_status = VL53L0X_OK;
    return VL53L0X_OK;
}

vl53l0x_status_t vl53l0x_clear_interrupt(vl53l0x_handle_t device){
    if(device == NULL) return VL53L0X_INVALID_ARGUMENT;
    return write_u8(device, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}
