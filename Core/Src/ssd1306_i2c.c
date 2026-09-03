#include "ssd1306_i2c.h"
#include "ssd1306_font.h"
#include <string.h>

#define SSD1306_CONTROL_COMMAND 0x00
#define SSD1306_CONTROL_DATA    0x40

struct ssd1306_i2c_t{
    ssd1306_i2c_config_t config;
    uint8_t framebuffer[SSD1306_BUFFER_SIZE];
};

static struct ssd1306_i2c_t ssd1306_i2c_device;
static bool ssd1306_i2c_in_use = false;
volatile uint32_t g_ssd1306_i2c_command_ok_count = 0;
volatile uint32_t g_ssd1306_i2c_command_error_count = 0;
volatile uint8_t g_ssd1306_i2c_init_stage = 0;
volatile uint8_t g_ssd1306_i2c_active_address = 0;
volatile bool g_ssd1306_i2c_sh1106_test_active = false;

static bool send_commands(ssd1306_i2c_handle_t display,
        uint8_t* commands, uint16_t size){
    bool succeeded = i2c_manager_write_poll(display->config.i2c_handle,
            display->config.i2c_address, SSD1306_CONTROL_COMMAND,
            commands, size) == _i2c_manager_ok;
    if(succeeded) g_ssd1306_i2c_command_ok_count++;
    else g_ssd1306_i2c_command_error_count++;
    return succeeded;
}

ssd1306_i2c_handle_t ssd1306_i2c_init(const ssd1306_i2c_config_t* config){
    if(config == NULL || config->i2c_handle == NULL ||
            config->dma_handle == NULL || ssd1306_i2c_in_use) return NULL;

    memset(&ssd1306_i2c_device, 0, sizeof(ssd1306_i2c_device));
    ssd1306_i2c_device.config = *config;
    g_ssd1306_i2c_active_address = config->i2c_address;
    g_ssd1306_i2c_init_stage = 1;

    /* Dört pinli I2C modülünde harici RESET yoktur. Besleme ve panelin dahili
     * power-on reset devresinin oturmasi icin ilk komuttan once beklenir. */
    osDelay(100);

    static uint8_t display_off[] = {0xAE};
    static uint8_t clock_divide[] = {0xD5, 0x80};
    static uint8_t multiplex[] = {0xA8, 0x3F};
    static uint8_t display_offset[] = {0xD3, 0x00};
    static uint8_t start_line[] = {0x40};
    static uint8_t charge_pump[] = {0x8D, 0x14};
    static uint8_t memory_mode[] = {0x20, 0x00};
    static uint8_t segment_remap[] = {0xA1};
    static uint8_t com_scan_direction[] = {0xC8};
    static uint8_t com_pins[] = {0xDA, 0x12};
    static uint8_t contrast[] = {0x81, 0xCF};
    static uint8_t precharge[] = {0xD9, 0xF1};
    static uint8_t vcom_deselect[] = {0xDB, 0x40};
    static uint8_t deactivate_scroll[] = {0x2E};
    static uint8_t normal_display[] = {0xA4, 0xA6};
    static uint8_t column_range[] = {0x21, 0x00, 0x7F};
    static uint8_t page_range[] = {0x22, 0x00, 0x07};
    static uint8_t display_on[] = {0xAF};

    if(!send_commands(&ssd1306_i2c_device, display_off,
            sizeof(display_off))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, clock_divide,
            sizeof(clock_divide))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, multiplex,
            sizeof(multiplex))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, display_offset,
            sizeof(display_offset))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, start_line,
            sizeof(start_line))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, charge_pump,
            sizeof(charge_pump))) return NULL;
    osDelay(10);
    if(!send_commands(&ssd1306_i2c_device, memory_mode,
            sizeof(memory_mode))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, segment_remap,
            sizeof(segment_remap))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, com_scan_direction,
            sizeof(com_scan_direction))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, com_pins,
            sizeof(com_pins))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, contrast,
            sizeof(contrast))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, precharge,
            sizeof(precharge))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, vcom_deselect,
            sizeof(vcom_deselect))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, deactivate_scroll,
            sizeof(deactivate_scroll))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, normal_display,
            sizeof(normal_display))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, column_range,
            sizeof(column_range))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, page_range,
            sizeof(page_range))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, display_on,
            sizeof(display_on))) return NULL;
    g_ssd1306_i2c_init_stage = 2;

    /* A5, GDDRAM'i dikkate almadan tum pikselleri yakar. Panel ve charge
     * pump calisiyorsa ekran kisa sure tamamen aydinlanmalidir. */
    static uint8_t all_pixels_on[] = {0xA5};
    if(!send_commands(&ssd1306_i2c_device, all_pixels_on,
            sizeof(all_pixels_on))) return NULL;
    g_ssd1306_i2c_init_stage = 3;
    osDelay(400);

    /* A4 ile tekrar framebuffer (GDDRAM) icerigini gosteren moda don. */
    static uint8_t resume_ram[] = {0xA4, 0xAF};
    if(!send_commands(&ssd1306_i2c_device, resume_ram,
            sizeof(resume_ram))) return NULL;
    g_ssd1306_i2c_init_stage = 4;

    /* Tani modu: Piyasadaki bazi dort pinli 1.3 inc moduller SSD1306 diye
     * satilsa da SH1106 denetleyicisi kullanir. SH1106'da dahili DC-DC
     * donusturucuyu acan komut AD 8B'dir. Ardindan AF ekrani, A5 ise
     * GDDRAM'den bagimsiz olarak butun pikselleri acik tutar.
     *
     * Bu test bilerek A4'e geri DONMEZ. Ekran tamamen aydinlanirsa modul
     * SH1106 uyumludur. Komutlar ACK aldigi halde ekran karanlik kalirsa
     * panel beslemesi/charge-pump/modul donanimi kuvvetli suphelidir. */
    static uint8_t sh1106_display_off[] = {0xAE};
    static uint8_t sh1106_dc_dc_on[] = {0xAD, 0x8B};
    static uint8_t sh1106_display_on[] = {0xAF};
    static uint8_t sh1106_all_pixels_on[] = {0xA5};

    if(!send_commands(&ssd1306_i2c_device, sh1106_display_off,
            sizeof(sh1106_display_off))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, sh1106_dc_dc_on,
            sizeof(sh1106_dc_dc_on))) return NULL;
    osDelay(10);
    if(!send_commands(&ssd1306_i2c_device, sh1106_display_on,
            sizeof(sh1106_display_on))) return NULL;
    if(!send_commands(&ssd1306_i2c_device, sh1106_all_pixels_on,
            sizeof(sh1106_all_pixels_on))) return NULL;

    g_ssd1306_i2c_sh1106_test_active = true;
    g_ssd1306_i2c_init_stage = 5;

    ssd1306_i2c_in_use = true;
    return &ssd1306_i2c_device;
}

void ssd1306_i2c_clear(ssd1306_i2c_handle_t display){
    if(display != NULL) memset(display->framebuffer, 0,
            sizeof(display->framebuffer));
}

void ssd1306_i2c_draw_text(ssd1306_i2c_handle_t display, uint8_t x,
        uint8_t page, const char* text){
    if(display == NULL || text == NULL || page >= 8) return;
    uint16_t index = (uint16_t)page * SSD1306_WIDTH + x;
    while(*text != '\0' && index + 5 < (uint16_t)(page + 1) * SSD1306_WIDTH){
        const uint8_t* glyph = ssd1306_font_glyph(*text++);
        for(uint8_t column = 0; column < 5; column++){
            display->framebuffer[index++] = glyph[column];
        }
        display->framebuffer[index++] = 0;
    }
}

bool ssd1306_i2c_build_refresh_job(ssd1306_i2c_handle_t display,
        osThreadId_t notify_task, uint32_t success_flag, uint32_t error_flag,
        i2c_job_t* job){
    if(display == NULL || notify_task == NULL || job == NULL ||
            success_flag == 0 || error_flag == 0 ||
            (success_flag & error_flag) != 0) return false;
    memset(job, 0, sizeof(*job));
    job->operation = I2C_JOB_WRITE_DMA;
    job->i2c_handle = display->config.i2c_handle;
    job->dma_handle = display->config.dma_handle;
    job->dma_stream = display->config.dma_stream;
    job->dev_addr = display->config.i2c_address;
    job->reg_addr = SSD1306_CONTROL_DATA;
    job->txdata = display->framebuffer;
    job->size = sizeof(display->framebuffer);
    job->notify_task = notify_task;
    job->notify_flag = success_flag;
    job->error_flag = error_flag;
    return true;
}
