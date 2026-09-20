#ifndef INC_SSD1306_I2C_H_
#define INC_SSD1306_I2C_H_

#include "i2c_manager.h"
#include <stdbool.h>
#include <stdint.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
#define SSD1306_I2C_ADDRESS_SA0_LOW  (0x3C << 1)
#define SSD1306_I2C_ADDRESS_SA0_HIGH (0x3D << 1)

typedef struct{
    I2C_TypeDef* i2c_handle;
    DMA_TypeDef* dma_handle;
    uint32_t dma_stream;
    uint8_t i2c_address;
}ssd1306_i2c_config_t;

struct ssd1306_i2c_t;
typedef struct ssd1306_i2c_t* ssd1306_i2c_handle_t;

/* STM32CubeIDE Live Expressions icin baslangic tanilari. */
extern volatile uint32_t g_ssd1306_i2c_command_ok_count;
extern volatile uint32_t g_ssd1306_i2c_command_error_count;
extern volatile uint8_t g_ssd1306_i2c_init_stage;
extern volatile uint8_t g_ssd1306_i2c_active_address;
extern volatile bool g_ssd1306_i2c_sh1106_test_active;

ssd1306_i2c_handle_t ssd1306_i2c_init(const ssd1306_i2c_config_t* config);
void ssd1306_i2c_clear(ssd1306_i2c_handle_t display);
void ssd1306_i2c_draw_text(ssd1306_i2c_handle_t display, uint8_t x,
        uint8_t page, const char* text);
bool ssd1306_i2c_build_refresh_job(ssd1306_i2c_handle_t display,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job);

#endif /* INC_SSD1306_I2C_H_ */
