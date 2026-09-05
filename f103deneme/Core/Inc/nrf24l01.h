#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define NRF24L01_MAX_PAYLOAD_SIZE 32

typedef enum{
    NRF24L01_OK = 0,
    NRF24L01_INVALID_ARGUMENT,
    NRF24L01_SPI_ERROR,
    NRF24L01_DEVICE_NOT_FOUND,
    NRF24L01_TIMEOUT,
    NRF24L01_MAX_RETRANSMIT,
    NRF24L01_NO_DATA
}nrf24l01_status_t;

typedef struct{
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* csn_port;
    uint16_t csn_pin;
    GPIO_TypeDef* ce_port;
    uint16_t ce_pin;
}nrf24l01_config_t;

typedef struct{
    volatile bool initialized;
    volatile bool irq_pending;
    volatile uint8_t last_status_register;
    volatile uint8_t last_payload_length;
    volatile uint32_t irq_count;
    volatile uint32_t tx_success_count;
    volatile uint32_t tx_error_count;
    volatile uint32_t rx_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t reinit_attempt_count;
    volatile uint32_t reinit_success_count;
    volatile uint32_t reinit_failure_count;
    volatile nrf24l01_status_t last_status;
    volatile uint8_t last_tx_payload[NRF24L01_MAX_PAYLOAD_SIZE];
    volatile uint8_t last_rx_payload[NRF24L01_MAX_PAYLOAD_SIZE];
}nrf24l01_debug_t;

extern volatile nrf24l01_debug_t g_nrf24l01_debug;

bool nrf24l01_init(const nrf24l01_config_t* config);
void nrf24l01_start_listening(void);
void nrf24l01_irq_callback(void);
nrf24l01_status_t nrf24l01_send(const uint8_t* payload, uint8_t length,
        uint32_t timeout_ms);
nrf24l01_status_t nrf24l01_receive(uint8_t* payload, uint8_t* length);

#endif /* INC_NRF24L01_H_ */
