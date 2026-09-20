#include "ssd1306_spi.h"
#include "ssd1306_font.h"
#include <string.h>

struct ssd1306_spi_t{
    ssd1306_spi_config_t config;
    uint8_t framebuffer[SSD1306_BUFFER_SIZE];
    uint8_t rx_buffer[SSD1306_BUFFER_SIZE];
};

static struct ssd1306_spi_t ssd1306_spi_device;
static bool ssd1306_spi_in_use = false;

static bool send_commands(ssd1306_spi_handle_t display,
        uint8_t* commands, uint16_t size){
    uint8_t rx[32] = {0};
    if(size > sizeof(rx)) return false;
    LL_GPIO_ResetOutputPin(display->config.dc_port, display->config.dc_pin);
    return spi_manager_transfer_poll(display->config.spi_handle,
            display->config.cs_port, display->config.cs_pin,
            commands, rx, size) == _spi_manager_ok;
}

ssd1306_spi_handle_t ssd1306_spi_init(const ssd1306_spi_config_t* config){
    static uint8_t init_commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x21, 0x00, 0x7F, 0x22, 0x00, 0x07, 0xAF
    };
    if(config == NULL || config->spi_handle == NULL ||
            config->dma_handle == NULL || config->cs_port == NULL ||
            config->dc_port == NULL || config->reset_port == NULL ||
            ssd1306_spi_in_use) return NULL;

    memset(&ssd1306_spi_device, 0, sizeof(ssd1306_spi_device));
    ssd1306_spi_device.config = *config;
    LL_GPIO_ResetOutputPin(config->reset_port, config->reset_pin);
    osDelay(1);
    LL_GPIO_SetOutputPin(config->reset_port, config->reset_pin);
    osDelay(10);
    if(!send_commands(&ssd1306_spi_device, init_commands,
            sizeof(init_commands))) return NULL;

    ssd1306_spi_in_use = true;
    return &ssd1306_spi_device;
}

void ssd1306_spi_clear(ssd1306_spi_handle_t display){
    if(display != NULL) memset(display->framebuffer, 0,
            sizeof(display->framebuffer));
}

void ssd1306_spi_draw_text(ssd1306_spi_handle_t display, uint8_t x,
        uint8_t page, const char* text){
    /* Ekran mekanik olarak 90 derece cevrildigi icin 64x128 portre bir
       koordinat sistemi kullanilir. Bes sutunluk font uc sutuna
       sikistirilir; boylece 64 pikselde 16 karakter goruntulenebilir. */
    if(display == NULL || text == NULL || page >= 16 || x >= 64) return;

    uint8_t logical_x = x;
    uint8_t logical_y = page * 8;
    while(*text != '\0' && logical_x + 3 < 64){
        const uint8_t* glyph = ssd1306_font_glyph(*text++);
        uint8_t compact_columns[3];
        compact_columns[0] = glyph[0] | glyph[1];
        compact_columns[1] = glyph[2];
        compact_columns[2] = glyph[3] | glyph[4];

        for(uint8_t column = 0; column < 3; column++){
            for(uint8_t row = 0; row < 7; row++){
                if((compact_columns[column] & (1 << row)) == 0) continue;

                uint8_t rotated_x = 127 - (logical_y + row);
                uint8_t rotated_y = logical_x + column;
                uint16_t framebuffer_index =
                        (uint16_t)(rotated_y / 8) * SSD1306_WIDTH + rotated_x;
                display->framebuffer[framebuffer_index] |=
                        (uint8_t)(1 << (rotated_y % 8));
            }
        }
        logical_x += 4;
    }
}

bool ssd1306_spi_build_refresh_job(ssd1306_spi_handle_t display,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        spi_job_t* job){
    if(display == NULL || notify_task == NULL || job == NULL ||
            success_flag == 0 || error_flag == 0 ||
            (success_flag & error_flag) != 0) return false;
    LL_GPIO_SetOutputPin(display->config.dc_port, display->config.dc_pin);
    memset(job, 0, sizeof(*job));
    job->spi_handle = display->config.spi_handle;
    job->dma_handle = display->config.dma_handle;
    job->rx_stream = display->config.rx_stream;
    job->tx_stream = display->config.tx_stream;
    job->cs_port = display->config.cs_port;
    job->cs_pin = display->config.cs_pin;
    job->txdata = display->framebuffer;
    job->rxdata = display->rx_buffer;
    job->size = sizeof(display->framebuffer);
    job->notify_task = notify_task;
    job->notify_flag = success_flag;
    job->error_flag = error_flag;
    return true;
}
