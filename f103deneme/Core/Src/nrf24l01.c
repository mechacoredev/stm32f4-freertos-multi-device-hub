#include "nrf24l01.h"
#include <string.h>

enum{
    NRF_REG_CONFIG = 0x00,
    NRF_REG_EN_AA = 0x01,
    NRF_REG_EN_RXADDR = 0x02,
    NRF_REG_SETUP_AW = 0x03,
    NRF_REG_SETUP_RETR = 0x04,
    NRF_REG_RF_CH = 0x05,
    NRF_REG_RF_SETUP = 0x06,
    NRF_REG_STATUS = 0x07,
    NRF_REG_RX_ADDR_P0 = 0x0A,
    NRF_REG_TX_ADDR = 0x10,
    NRF_REG_RX_PW_P0 = 0x11,
    NRF_REG_FIFO_STATUS = 0x17,
    NRF_REG_DYNPD = 0x1C,
    NRF_REG_FEATURE = 0x1D
};

#define NRF_CMD_R_REGISTER     0x00
#define NRF_CMD_W_REGISTER     0x20
#define NRF_CMD_R_RX_PAYLOAD   0x61
#define NRF_CMD_W_TX_PAYLOAD   0xA0
#define NRF_CMD_FLUSH_TX       0xE1
#define NRF_CMD_FLUSH_RX       0xE2
#define NRF_CMD_R_RX_PL_WID    0x60
#define NRF_CMD_ACTIVATE       0x50
#define NRF_CMD_NOP            0xFF

#define NRF_STATUS_RX_DR       0x40
#define NRF_STATUS_TX_DS       0x20
#define NRF_STATUS_MAX_RT      0x10

static nrf24l01_config_t nrf_config;
static bool nrf_initialized;
volatile nrf24l01_debug_t g_nrf24l01_debug;

static void csn_low(void){
    HAL_GPIO_WritePin(nrf_config.csn_port, nrf_config.csn_pin, GPIO_PIN_RESET);
}

static void csn_high(void){
    HAL_GPIO_WritePin(nrf_config.csn_port, nrf_config.csn_pin, GPIO_PIN_SET);
}

static void ce_low(void){
    HAL_GPIO_WritePin(nrf_config.ce_port, nrf_config.ce_pin, GPIO_PIN_RESET);
}

static void ce_high(void){
    HAL_GPIO_WritePin(nrf_config.ce_port, nrf_config.ce_pin, GPIO_PIN_SET);
}

static bool spi_transfer(const uint8_t* tx, uint8_t* rx, uint16_t size){
    csn_low();
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(nrf_config.spi,
            (uint8_t*)tx, rx, size, 100);
    csn_high();
    if(result == HAL_OK) return true;
    g_nrf24l01_debug.last_status = NRF24L01_SPI_ERROR;
    return false;
}

static bool command(uint8_t command_byte){
    uint8_t rx = 0;
    return spi_transfer(&command_byte, &rx, 1);
}

static bool write_register(uint8_t reg, uint8_t value){
    uint8_t tx[2] = {(uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1F)), value};
    uint8_t rx[2] = {0};
    if(!spi_transfer(tx, rx, 2)) return false;
    g_nrf24l01_debug.last_status_register = rx[0];
    return true;
}

static bool read_register(uint8_t reg, uint8_t* value){
    uint8_t tx[2] = {(uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1F)), NRF_CMD_NOP};
    uint8_t rx[2] = {0};
    if(value == NULL || !spi_transfer(tx, rx, 2)) return false;
    g_nrf24l01_debug.last_status_register = rx[0];
    *value = rx[1];
    return true;
}

static bool write_register_burst(uint8_t reg, const uint8_t* data,
        uint8_t size){
    if(data == NULL || size == 0 || size > 5) return false;
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};
    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, size);
    return spi_transfer(tx, rx, size + 1);
}

static bool enable_dynamic_payload(void){
    if(!write_register(NRF_REG_FEATURE, 0x04)) return false;
    uint8_t feature = 0;
    if(!read_register(NRF_REG_FEATURE, &feature)) return false;
    if((feature & 0x04) == 0){
        uint8_t tx[2] = {NRF_CMD_ACTIVATE, 0x73};
        uint8_t rx[2] = {0};
        if(!spi_transfer(tx, rx, 2)) return false;
        if(!write_register(NRF_REG_FEATURE, 0x04)) return false;
    }
    if(!write_register(NRF_REG_DYNPD, 0x01)) return false;
    if(!read_register(NRF_REG_FEATURE, &feature)) return false;
    uint8_t dynpd = 0;
    if(!read_register(NRF_REG_DYNPD, &dynpd)) return false;
    return (feature & 0x04) != 0 && (dynpd & 0x01) != 0;
}

static void clear_interrupts(uint8_t mask){
    write_register(NRF_REG_STATUS, mask & 0x70);
}

static uint8_t read_status(void){
    uint8_t tx = NRF_CMD_NOP;
    uint8_t rx = 0;
    if(!spi_transfer(&tx, &rx, 1)) return 0;
    g_nrf24l01_debug.last_status_register = rx;
    return rx;
}

bool nrf24l01_init(const nrf24l01_config_t* config){
    nrf_initialized = false;
    uint32_t attempt_count = g_nrf24l01_debug.reinit_attempt_count + 1;
    uint32_t success_count = g_nrf24l01_debug.reinit_success_count;
    uint32_t failure_count = g_nrf24l01_debug.reinit_failure_count;
    if(config == NULL || config->spi == NULL || config->csn_port == NULL ||
            config->ce_port == NULL){
        g_nrf24l01_debug.last_status = NRF24L01_INVALID_ARGUMENT;
        g_nrf24l01_debug.reinit_attempt_count = attempt_count;
        g_nrf24l01_debug.reinit_failure_count = failure_count + 1;
        return false;
    }

    memset((void*)&g_nrf24l01_debug, 0, sizeof(g_nrf24l01_debug));
    g_nrf24l01_debug.reinit_attempt_count = attempt_count;
    g_nrf24l01_debug.reinit_success_count = success_count;
    g_nrf24l01_debug.reinit_failure_count = failure_count;
    nrf_config = *config;
    csn_high();
    ce_low();
    HAL_Delay(5);

    static const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    bool configured =
            write_register(NRF_REG_CONFIG, 0x0A) &&
            write_register(NRF_REG_EN_AA, 0x01) &&
            write_register(NRF_REG_EN_RXADDR, 0x01) &&
            write_register(NRF_REG_SETUP_AW, 0x03) &&
            write_register(NRF_REG_SETUP_RETR, 0x3F) &&
            write_register(NRF_REG_RF_CH, 76) &&
            write_register(NRF_REG_RF_SETUP, 0x07) &&
            write_register(NRF_REG_RX_PW_P0, 0) &&
            enable_dynamic_payload() &&
            write_register_burst(NRF_REG_RX_ADDR_P0, address, 5) &&
            write_register_burst(NRF_REG_TX_ADDR, address, 5) &&
            command(NRF_CMD_FLUSH_TX) && command(NRF_CMD_FLUSH_RX);

    uint8_t address_width = 0;
    if(!configured || !read_register(NRF_REG_SETUP_AW, &address_width) ||
            address_width != 0x03){
        g_nrf24l01_debug.last_status = NRF24L01_DEVICE_NOT_FOUND;
        g_nrf24l01_debug.reinit_failure_count++;
        return false;
    }

    clear_interrupts(0x70);
    nrf_initialized = true;
    g_nrf24l01_debug.initialized = true;
    g_nrf24l01_debug.last_status = NRF24L01_OK;
    g_nrf24l01_debug.reinit_success_count++;
    nrf24l01_start_listening();
    return true;
}

void nrf24l01_start_listening(void){
    if(!nrf_initialized) return;
    uint8_t config = 0;
    if(!read_register(NRF_REG_CONFIG, &config)) return;
    write_register(NRF_REG_CONFIG, config | 0x03);
    ce_high();
}

void nrf24l01_irq_callback(void){
    g_nrf24l01_debug.irq_pending = true;
    g_nrf24l01_debug.irq_count++;
}

nrf24l01_status_t nrf24l01_send(const uint8_t* payload, uint8_t length,
        uint32_t timeout_ms){
    if(!nrf_initialized || payload == NULL || length == 0 ||
            length > NRF24L01_MAX_PAYLOAD_SIZE){
        return NRF24L01_INVALID_ARGUMENT;
    }

    ce_low();
    uint8_t config = 0;
    if(!read_register(NRF_REG_CONFIG, &config) ||
            !write_register(NRF_REG_CONFIG, (config | 0x02) & (uint8_t)~0x01)){
        return NRF24L01_SPI_ERROR;
    }
    clear_interrupts(NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    uint8_t tx[NRF24L01_MAX_PAYLOAD_SIZE + 1] = {0};
    uint8_t rx[NRF24L01_MAX_PAYLOAD_SIZE + 1] = {0};
    tx[0] = NRF_CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], payload, length);
    if(!spi_transfer(tx, rx, length + 1)) return NRF24L01_SPI_ERROR;

    ce_high();
    HAL_Delay(1);
    ce_low();

    uint32_t start = HAL_GetTick();
    nrf24l01_status_t result = NRF24L01_TIMEOUT;
    while((HAL_GetTick() - start) < timeout_ms){
        uint8_t status = read_status();
        if((status & NRF_STATUS_TX_DS) != 0){
            clear_interrupts(NRF_STATUS_TX_DS);
            result = NRF24L01_OK;
            break;
        }
        if((status & NRF_STATUS_MAX_RT) != 0){
            clear_interrupts(NRF_STATUS_MAX_RT);
            command(NRF_CMD_FLUSH_TX);
            result = NRF24L01_MAX_RETRANSMIT;
            break;
        }
    }

    if(result == NRF24L01_OK){
        memcpy((void*)g_nrf24l01_debug.last_tx_payload, payload, length);
        g_nrf24l01_debug.last_payload_length = length;
        g_nrf24l01_debug.tx_success_count++;
    }else{
        g_nrf24l01_debug.tx_error_count++;
    }
    g_nrf24l01_debug.last_status = result;
    nrf24l01_start_listening();
    return result;
}

nrf24l01_status_t nrf24l01_receive(uint8_t* payload, uint8_t* length){
    if(!nrf_initialized || payload == NULL || length == NULL){
        return NRF24L01_INVALID_ARGUMENT;
    }

    uint8_t fifo = 0;
    if(!read_register(NRF_REG_FIFO_STATUS, &fifo)) return NRF24L01_SPI_ERROR;
    if((fifo & 0x01) != 0){
        g_nrf24l01_debug.irq_pending = false;
        return NRF24L01_NO_DATA;
    }

    uint8_t width_tx[2] = {NRF_CMD_R_RX_PL_WID, NRF_CMD_NOP};
    uint8_t width_rx[2] = {0};
    if(!spi_transfer(width_tx, width_rx, 2)) return NRF24L01_SPI_ERROR;
    uint8_t payload_length = width_rx[1];
    if(payload_length == 0 || payload_length > NRF24L01_MAX_PAYLOAD_SIZE){
        command(NRF_CMD_FLUSH_RX);
        clear_interrupts(NRF_STATUS_RX_DR);
        g_nrf24l01_debug.rx_error_count++;
        return NRF24L01_SPI_ERROR;
    }

    uint8_t tx[NRF24L01_MAX_PAYLOAD_SIZE + 1] = {0};
    uint8_t rx[NRF24L01_MAX_PAYLOAD_SIZE + 1] = {0};
    tx[0] = NRF_CMD_R_RX_PAYLOAD;
    memset(&tx[1], NRF_CMD_NOP, payload_length);
    if(!spi_transfer(tx, rx, payload_length + 1)) return NRF24L01_SPI_ERROR;
    memcpy(payload, &rx[1], payload_length);
    *length = payload_length;
    memcpy((void*)g_nrf24l01_debug.last_rx_payload, payload, payload_length);
    g_nrf24l01_debug.last_payload_length = payload_length;
    g_nrf24l01_debug.rx_count++;
    g_nrf24l01_debug.irq_pending = false;
    g_nrf24l01_debug.last_status = NRF24L01_OK;
    clear_interrupts(NRF_STATUS_RX_DR);
    return NRF24L01_OK;
}
