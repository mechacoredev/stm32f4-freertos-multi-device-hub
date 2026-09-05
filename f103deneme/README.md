# STM32F103 CAN + nRF24L01 companion node

This firmware is the STM32F103C8T6 companion for the F407 multi-device hub.

## Connections

### CAN1

- PA11: CAN RX to SN65HVD230 CRX/RXD
- PA12: CAN TX to SN65HVD230 CTX/TXD
- CAN nominal bitrate: 500 kbit/s

### nRF24L01 on SPI2

- PB13: SCK
- PB14: MISO
- PB15: MOSI
- PA8: IRQ
- PA9: CSN
- PA10: CE
- Supply: 3.3 V only, common ground

### Communication state LEDs

- PB0: CAN receive state LED, active high
- PB1: nRF24L01 receive state LED, active high

The radio uses channel 76, 1 Mbit/s, dynamic payloads and address
`E7:E7:E7:E7:E7`, matching the F407 firmware.

## Runtime behavior

- F407 sends one CAN and one nRF24L01 state command every 500 ms.
- F103 applies each command and immediately returns the inverse level over the
  same transport. This request-response timing prevents the two radios from
  entering transmit mode at the same instant.
- Incoming F407 `HIGH`/`LOW` CAN commands directly set/reset PB0.
- Incoming F407 `HIGH`/`LOW` nRF commands directly set/reset PB1.
- A valid frame is expected every 500 ms. A 750 ms deadline includes scheduler
  and radio turnaround margin while still detecting the first missed interval.
- A silent CAN link is restarted once per second; automatic bus-off management
  is enabled.
- A silent or disconnected nRF24L01 is reinitialized once per second. Reconnect
  does not trap the application in a permanent failure loop.

Useful Live Expressions:

- `g_nrf24l01_debug`
- `g_f103_nrf_rx_data`
- `g_f103_nrf_rx_count`
- `g_f103_nrf_tx_count`
- `g_f103_nrf_tx_error_count`
- `g_can1_rx_data`
- `g_can1_rx_count`
- `g_can1_tx_ok_count`
- `g_f103_can_rx_last_tick`
- `g_f103_nrf_rx_last_tick`
- `g_f103_can_rx_timeout`
- `g_f103_nrf_rx_timeout`
