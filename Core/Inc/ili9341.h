/*
 * ili9341.h
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "spi_manager.h"
#include "stdint.h"
#include "stddef.h"

typedef enum {
    _ili9341_ok = 0,
    _ili9341_fail,
    _ili9341_busy,
    _ili9341_complete
} ili9341_return_status;

typedef enum {
    ILI9341_ROTATION_0 = 0,
    ILI9341_ROTATION_90,
    ILI9341_ROTATION_180,
    ILI9341_ROTATION_270
} ili9341_rotation_t;

typedef struct {
    SPI_TypeDef* spi_handle;
    DMA_TypeDef* dma_handle;
    uint32_t dma_tx_stream;
    uint32_t dma_rx_stream;

    GPIO_TypeDef* cs_port;
    uint32_t cs_pin;
    
    GPIO_TypeDef* dc_port;
    uint32_t dc_pin;
    
    GPIO_TypeDef* rst_port;
    uint32_t rst_pin;

    ili9341_rotation_t rotation;
} ili9341_user_configs;

struct ili9341_t;
typedef struct ili9341_t* ili9341_handle_t;

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320
#define ILI9341_LANDSCAPE_WIDTH  320
#define ILI9341_LANDSCAPE_HEIGHT 240

#define ILI9341_COLOR_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_COLOR_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_COLOR_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_DMA_BUFFER_SIZE   6144
#define ILI9341_CHARACTER_WIDTH   12
#define ILI9341_CHARACTER_HEIGHT  16
#define ILI9341_CHARACTER_BYTES   384

// Init the display (includes software reset and register configuration via polling)
ili9341_handle_t ili9341_init(ili9341_user_configs* config);

// Bir çizim işlemini hazırlar. Piksel aktarımı SPI DMA taskı tarafından yapılır.
ili9341_return_status ili9341_begin_fill_rect(ili9341_handle_t dev,
        uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
ili9341_return_status ili9341_begin_string(ili9341_handle_t dev,
        uint16_t x, uint16_t y, const char* str,
        uint16_t color, uint16_t bg_color);

// Hazır işlemin sıradaki DMA parçasını oluşturur.
// _ili9341_complete dönerse gönderilecek başka parça kalmamıştır.
ili9341_return_status ili9341_build_next_dma_job(ili9341_handle_t dev,
        osThreadId_t notify_task, uint32_t notify_flag, uint32_t error_flag,
        spi_job_t* job);

// SPI DMA taskından gelen sonucu sürücüye bildirir. Buffer ancak bundan sonra
// tekrar kullanıldığı için kuyruktaki işin verisi ezilmez.
ili9341_return_status ili9341_finish_dma_job(ili9341_handle_t dev,
        bool transfer_succeeded);

bool ili9341_operation_is_complete(ili9341_handle_t dev);
uint16_t ili9341_get_width(ili9341_handle_t dev);
uint16_t ili9341_get_height(ili9341_handle_t dev);

#endif /* INC_ILI9341_H_ */
