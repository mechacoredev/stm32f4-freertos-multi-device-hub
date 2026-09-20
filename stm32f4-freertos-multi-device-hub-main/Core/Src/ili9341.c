/*
 * ili9341.c
 */

#include "ili9341.h"
#include "string.h"

// Simple 5x7 Font (ASCII 32 to 127)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 Space
    {0x00, 0x00, 0x4f, 0x00, 0x00}, // 33 !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 "
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // 35 #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // 36 $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // 40 (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // 41 )
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // 42 *
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // 43 +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 /
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 48 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 49 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 51 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 52 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 54 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 57 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 ?
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // 64 @
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 65 A
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // 66 B
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // 67 C
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // 68 D
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // 69 E
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // 70 F
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // 71 G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 72 H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // 73 I
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // 74 J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // 75 K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // 76 L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 77 M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // 78 N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 79 O
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // 80 P
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 81 Q
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // 82 R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // 84 T
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // 85 U
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // 86 V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // 87 W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 Z
    {0x00, 0x7f, 0x41, 0x41, 0x00}, // 91 [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 Backslash
    {0x00, 0x41, 0x41, 0x7f, 0x00}, // 93 ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96 `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97 a
    {0x7f, 0x48, 0x44, 0x44, 0x38}, // 98 b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99 c
    {0x38, 0x44, 0x44, 0x48, 0x7f}, // 100 d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101 e
    {0x08, 0x7e, 0x09, 0x01, 0x02}, // 102 f
    {0x0c, 0x52, 0x52, 0x52, 0x3e}, // 103 g
    {0x7f, 0x08, 0x04, 0x04, 0x78}, // 104 h
    {0x00, 0x44, 0x7d, 0x40, 0x00}, // 105 i
    {0x20, 0x40, 0x44, 0x3d, 0x00}, // 106 j
    {0x7f, 0x10, 0x28, 0x44, 0x00}, // 107 k
    {0x00, 0x41, 0x7f, 0x40, 0x00}, // 108 l
    {0x7c, 0x04, 0x18, 0x04, 0x78}, // 109 m
    {0x7c, 0x08, 0x04, 0x04, 0x78}, // 110 n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111 o
    {0x7c, 0x14, 0x14, 0x14, 0x08}, // 112 p
    {0x08, 0x14, 0x14, 0x18, 0x7c}, // 113 q
    {0x7c, 0x08, 0x04, 0x04, 0x08}, // 114 r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115 s
    {0x04, 0x3f, 0x44, 0x40, 0x20}, // 116 t
    {0x3c, 0x40, 0x40, 0x20, 0x7c}, // 117 u
    {0x1c, 0x20, 0x40, 0x20, 0x1c}, // 118 v
    {0x3c, 0x40, 0x30, 0x40, 0x3c}, // 119 w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120 x
    {0x1c, 0xa0, 0xa0, 0xa0, 0x7c}, // 121 y
    {0x44, 0x64, 0x54, 0x4c, 0x44}, // 122 z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123 {
    {0x00, 0x00, 0x7f, 0x00, 0x00}, // 124 |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125 }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 126 ~
    {0x00, 0x00, 0x00, 0x00, 0x00}  // 127
};

typedef enum {
    ILI9341_OPERATION_NONE = 0,
    ILI9341_OPERATION_FILL,
    ILI9341_OPERATION_STRING
} ili9341_operation_t;

struct ili9341_t {
    ili9341_user_configs config;
    uint16_t width;
    uint16_t height;
    ili9341_operation_t operation;
    bool job_pending;
    uint16_t pending_bytes;
    uint32_t remaining_bytes;

    const char* string;
    uint16_t string_length;
    uint16_t string_index;
    uint16_t string_x;
    uint16_t string_y;
    uint16_t string_color;
    uint16_t string_bg_color;

    uint8_t dma_tx_buffer[ILI9341_DMA_BUFFER_SIZE];
    uint8_t dma_rx_buffer[ILI9341_DMA_BUFFER_SIZE];
};

#define ILI9341_MAX_DEVICES 2

static struct ili9341_t device_pool[ILI9341_MAX_DEVICES];
static bool device_pool_used[ILI9341_MAX_DEVICES];

static inline void dc_cmd(struct ili9341_t* dev) {
    LL_GPIO_ResetOutputPin(dev->config.dc_port, dev->config.dc_pin);
}

static inline void dc_data(struct ili9341_t* dev) {
    LL_GPIO_SetOutputPin(dev->config.dc_port, dev->config.dc_pin);
}

static inline void rst_low(struct ili9341_t* dev) {
    LL_GPIO_ResetOutputPin(dev->config.rst_port, dev->config.rst_pin);
}

static inline void rst_high(struct ili9341_t* dev) {
    LL_GPIO_SetOutputPin(dev->config.rst_port, dev->config.rst_pin);
}

static struct ili9341_t* allocate_device(void) {
    for(uint8_t index = 0; index < ILI9341_MAX_DEVICES; index++) {
        if(device_pool_used[index]) continue;
        device_pool_used[index] = true;
        memset(&device_pool[index], 0, sizeof(struct ili9341_t));
        return &device_pool[index];
    }
    return NULL;
}

static void release_device(struct ili9341_t* dev) {
    for(uint8_t index = 0; index < ILI9341_MAX_DEVICES; index++) {
        if(dev != &device_pool[index]) continue;
        device_pool_used[index] = false;
        return;
    }
}

static ili9341_return_status transfer_poll(struct ili9341_t* dev,
        uint8_t* tx_data, uint8_t* rx_data, uint16_t size) {
    spi_manager_return_status spi_status;
    spi_status = spi_manager_transfer_poll(dev->config.spi_handle,
            dev->config.cs_port, dev->config.cs_pin,
            tx_data, rx_data, size);
    if(spi_status != _spi_manager_ok) return _ili9341_fail;
    return _ili9341_ok;
}

static ili9341_return_status write_command(struct ili9341_t* dev,
        uint8_t command, const uint8_t* parameters, uint8_t parameter_count) {
    uint8_t command_rx = 0;
    dc_cmd(dev);

    ili9341_return_status status;
    status = transfer_poll(dev, &command, &command_rx, 1);
    if(status != _ili9341_ok) return status;
    if(parameter_count == 0) return _ili9341_ok;
    if(parameters == NULL || parameter_count > 16) return _ili9341_fail;

    uint8_t parameter_tx[16] = {0};
    uint8_t parameter_rx[16] = {0};
    memcpy(parameter_tx, parameters, parameter_count);
    dc_data(dev);
    return transfer_poll(dev, parameter_tx, parameter_rx, parameter_count);
}

static ili9341_return_status set_window(struct ili9341_t* dev,
        uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t column_data[4] = {
        x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF
    };
    uint8_t page_data[4] = {
        y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF
    };

    ili9341_return_status status;
    status = write_command(dev, 0x2A, column_data, 4);
    if(status != _ili9341_ok) return status;

    status = write_command(dev, 0x2B, page_data, 4);
    if(status != _ili9341_ok) return status;

    status = write_command(dev, 0x2C, NULL, 0);
    if(status != _ili9341_ok) return status;

    dc_data(dev);
    return _ili9341_ok;
}

static void render_string_chunk(struct ili9341_t* dev,
        uint16_t first_character, uint16_t character_count) {
    uint16_t buffer_index = 0;
    for(uint8_t pixel_row = 0; pixel_row < ILI9341_CHARACTER_HEIGHT;
            pixel_row++) {
        uint8_t font_row = pixel_row / 2;

        for(uint16_t character_index = 0;
                character_index < character_count; character_index++) {
            char character = dev->string[first_character + character_index];
            if(character < 32 || character > 127) character = 32;

            for(uint8_t pixel_column = 0;
                    pixel_column < ILI9341_CHARACTER_WIDTH; pixel_column++) {
                uint8_t font_column_index = pixel_column / 2;
                bool pixel_on = false;
                if(font_column_index < 5 && font_row < 7) {
                    uint8_t font_column;
                    font_column = font5x7[character - 32][font_column_index];
                    pixel_on = (font_column & (1 << font_row)) != 0;
                }

                uint16_t pixel_color = dev->string_bg_color;
                if(pixel_on) pixel_color = dev->string_color;

                dev->dma_tx_buffer[buffer_index] = pixel_color >> 8;
                buffer_index++;
                dev->dma_tx_buffer[buffer_index] = pixel_color & 0xFF;
                buffer_index++;
            }
        }
    }
}

ili9341_handle_t ili9341_init(ili9341_user_configs* config) {
    if(config == NULL) return NULL;
    if(config->spi_handle == NULL || config->dma_handle == NULL) return NULL;
    if(config->cs_port == NULL || config->dc_port == NULL) return NULL;
    if(config->rst_port == NULL) return NULL;

    struct ili9341_t* dev;
    dev = allocate_device();
    if(dev == NULL) return NULL;
    memcpy(&dev->config, config, sizeof(ili9341_user_configs));
    dev->width = ILI9341_TFTWIDTH;
    dev->height = ILI9341_TFTHEIGHT;
    if(config->rotation == ILI9341_ROTATION_90 ||
            config->rotation == ILI9341_ROTATION_270) {
        dev->width = ILI9341_LANDSCAPE_WIDTH;
        dev->height = ILI9341_LANDSCAPE_HEIGHT;
    }

    rst_high(dev);
    osDelay(5);
    rst_low(dev);
    osDelay(20);
    rst_high(dev);
    osDelay(150);

    static const uint8_t power_b[] = {0x00, 0xC1, 0x30};
    static const uint8_t power_seq[] = {0x64, 0x03, 0x12, 0x81};
    static const uint8_t driver_timing_a[] = {0x85, 0x00, 0x78};
    static const uint8_t power_a[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    static const uint8_t pump_ratio[] = {0x20};
    static const uint8_t driver_timing_b[] = {0x00, 0x00};
    static const uint8_t power_control_1[] = {0x23};
    static const uint8_t power_control_2[] = {0x10};
    static const uint8_t vcom_control_1[] = {0x3E, 0x28};
    static const uint8_t vcom_control_2[] = {0x86};
    uint8_t memory_access[] = {0x48};
    if(config->rotation == ILI9341_ROTATION_90) {
        memory_access[0] = 0x28;
    }
    else if(config->rotation == ILI9341_ROTATION_180) {
        memory_access[0] = 0x88;
    }
    else if(config->rotation == ILI9341_ROTATION_270) {
        /* MY | MX | MV | BGR: ROTATION_90'in tam 180 derece tersi. */
        memory_access[0] = 0xE8;
    }
    static const uint8_t pixel_format[] = {0x55};
    static const uint8_t frame_rate[] = {0x00, 0x18};
    static const uint8_t display_function[] = {0x08, 0x82, 0x27};
    static const uint8_t interface_control[] = {0x03, 0x80, 0x02};

    ili9341_return_status status;
    status = write_command(dev, 0xEF, interface_control, 3);
    if(status == _ili9341_ok) status = write_command(dev, 0xCF, power_b, 3);
    if(status == _ili9341_ok) status = write_command(dev, 0xED, power_seq, 4);
    if(status == _ili9341_ok) status = write_command(dev, 0xE8, driver_timing_a, 3);
    if(status == _ili9341_ok) status = write_command(dev, 0xCB, power_a, 5);
    if(status == _ili9341_ok) status = write_command(dev, 0xF7, pump_ratio, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0xEA, driver_timing_b, 2);
    if(status == _ili9341_ok) status = write_command(dev, 0xC0, power_control_1, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0xC1, power_control_2, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0xC5, vcom_control_1, 2);
    if(status == _ili9341_ok) status = write_command(dev, 0xC7, vcom_control_2, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0x36, memory_access, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0x3A, pixel_format, 1);
    if(status == _ili9341_ok) status = write_command(dev, 0xB1, frame_rate, 2);
    if(status == _ili9341_ok) status = write_command(dev, 0xB6, display_function, 3);
    if(status == _ili9341_ok) status = write_command(dev, 0x11, NULL, 0);

    if(status != _ili9341_ok) {
        release_device(dev);
        return NULL;
    }

    osDelay(120);
    status = write_command(dev, 0x29, NULL, 0);
    if(status != _ili9341_ok) {
        release_device(dev);
        return NULL;
    }

    return dev;
}

ili9341_return_status ili9341_begin_fill_rect(ili9341_handle_t dev,
        uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if(dev == NULL) return _ili9341_fail;
    if(dev->operation != ILI9341_OPERATION_NONE) return _ili9341_busy;
    if(w == 0 || h == 0) return _ili9341_fail;
    if(x >= dev->width || y >= dev->height) return _ili9341_fail;

    uint32_t x_end = (uint32_t)x + w;
    uint32_t y_end = (uint32_t)y + h;
    if(x_end > dev->width) w = dev->width - x;
    if(y_end > dev->height) h = dev->height - y;

    ili9341_return_status status;
    status = set_window(dev, x, y, x + w - 1, y + h - 1);
    if(status != _ili9341_ok) return status;

    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    for(uint16_t index = 0; index < ILI9341_DMA_BUFFER_SIZE; index += 2) {
        dev->dma_tx_buffer[index] = color_high;
        dev->dma_tx_buffer[index + 1] = color_low;
    }

    dev->remaining_bytes = (uint32_t)w * h * 2;
    dev->pending_bytes = 0;
    dev->job_pending = false;
    dev->operation = ILI9341_OPERATION_FILL;
    return _ili9341_ok;
}

ili9341_return_status ili9341_begin_string(ili9341_handle_t dev,
        uint16_t x, uint16_t y, const char* str,
        uint16_t color, uint16_t bg_color) {
    if(dev == NULL || str == NULL) return _ili9341_fail;
    if(dev->operation != ILI9341_OPERATION_NONE) return _ili9341_busy;
    if(x >= dev->width || y >= dev->height) return _ili9341_fail;
    if((uint32_t)y + ILI9341_CHARACTER_HEIGHT > dev->height) return _ili9341_fail;

    uint16_t character_limit = (dev->width - x) / ILI9341_CHARACTER_WIDTH;
    uint16_t string_length = strlen(str);
    if(string_length > character_limit) string_length = character_limit;

    dev->string = str;
    dev->string_length = string_length;
    dev->string_index = 0;
    dev->string_x = x;
    dev->string_y = y;
    dev->string_color = color;
    dev->string_bg_color = bg_color;
    dev->pending_bytes = 0;
    dev->job_pending = false;

    if(string_length == 0) {
        dev->operation = ILI9341_OPERATION_NONE;
        return _ili9341_ok;
    }

    dev->operation = ILI9341_OPERATION_STRING;
    return _ili9341_ok;
}

ili9341_return_status ili9341_build_next_dma_job(ili9341_handle_t dev,
        osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
        spi_job_t* job) {
    if(dev == NULL || job == NULL || notify_task == NULL) return _ili9341_fail;
    if(dev->operation == ILI9341_OPERATION_NONE) return _ili9341_complete;
    if(dev->job_pending) return _ili9341_busy;

    uint16_t transfer_size = 0;
    if(dev->operation == ILI9341_OPERATION_FILL) {
        transfer_size = ILI9341_DMA_BUFFER_SIZE;
        if(dev->remaining_bytes < transfer_size) {
            transfer_size = dev->remaining_bytes;
        }
    }
    else if(dev->operation == ILI9341_OPERATION_STRING) {
        uint16_t remaining_characters;
        remaining_characters = dev->string_length - dev->string_index;

        uint16_t maximum_characters;
        maximum_characters = ILI9341_DMA_BUFFER_SIZE / ILI9341_CHARACTER_BYTES;

        uint16_t characters_in_job = remaining_characters;
        if(characters_in_job > maximum_characters) {
            characters_in_job = maximum_characters;
        }

        uint16_t character_x;
        character_x = dev->string_x +
                (dev->string_index * ILI9341_CHARACTER_WIDTH);
        uint16_t window_width;
        window_width = characters_in_job * ILI9341_CHARACTER_WIDTH;

        ili9341_return_status status;
        status = set_window(dev, character_x, dev->string_y,
                character_x + window_width - 1,
                dev->string_y + ILI9341_CHARACTER_HEIGHT - 1);
        if(status != _ili9341_ok) {
            dev->operation = ILI9341_OPERATION_NONE;
            return status;
        }

        render_string_chunk(dev, dev->string_index, characters_in_job);
        transfer_size = characters_in_job * ILI9341_CHARACTER_BYTES;
    }

    if(transfer_size == 0) return _ili9341_complete;
    dc_data(dev);

    memset(job, 0, sizeof(spi_job_t));
    job->spi_handle = dev->config.spi_handle;
    job->dma_handle = dev->config.dma_handle;
    job->tx_stream = dev->config.dma_tx_stream;
    job->rx_stream = dev->config.dma_rx_stream;
    job->cs_port = dev->config.cs_port;
    job->cs_pin = dev->config.cs_pin;
    job->txdata = dev->dma_tx_buffer;
    job->rxdata = dev->dma_rx_buffer;
    job->size = transfer_size;
    job->notify_task = notify_task;
    job->notify_flag = notify_flag;
    job->error_flag = error_flag;

    dev->pending_bytes = transfer_size;
    dev->job_pending = true;
    return _ili9341_ok;
}

ili9341_return_status ili9341_finish_dma_job(ili9341_handle_t dev,
        bool transfer_succeeded) {
    if(dev == NULL || dev->job_pending == false) return _ili9341_fail;

    dev->job_pending = false;
    if(transfer_succeeded == false) {
        dev->operation = ILI9341_OPERATION_NONE;
        dev->pending_bytes = 0;
        return _ili9341_fail;
    }

    if(dev->operation == ILI9341_OPERATION_FILL) {
        if(dev->pending_bytes > dev->remaining_bytes) {
            dev->operation = ILI9341_OPERATION_NONE;
            return _ili9341_fail;
        }
        dev->remaining_bytes -= dev->pending_bytes;
        if(dev->remaining_bytes == 0) dev->operation = ILI9341_OPERATION_NONE;
    }
    else if(dev->operation == ILI9341_OPERATION_STRING) {
        uint16_t completed_characters;
        completed_characters = dev->pending_bytes / ILI9341_CHARACTER_BYTES;
        dev->string_index += completed_characters;
        if(dev->string_index >= dev->string_length) {
            dev->operation = ILI9341_OPERATION_NONE;
        }
    }

    dev->pending_bytes = 0;
    return _ili9341_ok;
}

bool ili9341_operation_is_complete(ili9341_handle_t dev) {
    if(dev == NULL) return true;
    return dev->operation == ILI9341_OPERATION_NONE;
}

uint16_t ili9341_get_width(ili9341_handle_t dev) {
    if(dev == NULL) return 0;
    return dev->width;
}

uint16_t ili9341_get_height(ili9341_handle_t dev) {
    if(dev == NULL) return 0;
    return dev->height;
}
