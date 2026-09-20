#ifndef INC_SSD1306_SPI_H_
#define INC_SSD1306_SPI_H_

#include "spi_manager.h"
#include "ssd1306_i2c.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct{
    SPI_TypeDef* spi_handle;
    DMA_TypeDef* dma_handle;
    uint32_t rx_stream;
    uint32_t tx_stream;
    GPIO_TypeDef* cs_port;
    uint32_t cs_pin;
    GPIO_TypeDef* dc_port;
    uint32_t dc_pin;
    GPIO_TypeDef* reset_port;
    uint32_t reset_pin;
}ssd1306_spi_config_t;

struct ssd1306_spi_t;
typedef struct ssd1306_spi_t* ssd1306_spi_handle_t;

ssd1306_spi_handle_t ssd1306_spi_init(const ssd1306_spi_config_t* config);
void ssd1306_spi_clear(ssd1306_spi_handle_t display);
void ssd1306_spi_draw_text(ssd1306_spi_handle_t display, uint8_t x,
        uint8_t page, const char* text);
bool ssd1306_spi_build_refresh_job(ssd1306_spi_handle_t display,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        spi_job_t* job);

#endif /* INC_SSD1306_SPI_H_ */
