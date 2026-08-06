# LL Multi Device Pin Map

Target board: STM32F407G-DISC1 / MB997, STM32F407VGT6 (LQFP100).

The P1/P2 positions below follow the official MB997 board schematic. All
external modules use 3.3 V logic and must share GND with the Discovery board.

## I2C1 — BME280 and MPU6050

| Signal | MCU pin | Header | Configuration |
|---|---:|---:|---|
| I2C1 SCL | PB6 | P2-23 | AF4, open-drain, pull-up |
| I2C1 SDA | PB7 | P2-24 | AF4, open-drain, pull-up |
| MPU6050 INT | PB1 | P2-21 | EXTI1, rising edge, pull-down |

## SPI1 — RC522 and nRF24L01 RX

| Signal | MCU pin | Header | Configuration |
|---|---:|---:|---|
| SPI1 SCK | PB3 | P2-28 | AF5, push-pull |
| SPI1 MISO | PB4 | P2-25 | AF5 |
| SPI1 MOSI | PB5 | P2-26 | AF5, push-pull |
| RC522 CS | PD6 | P2-30 | Output, initial high |
| RC522 RST | PD7 | P2-27 | Output, initial high |
| RC522 IRQ | PD3 | P2-31 | EXTI3, falling edge, pull-up |
| nRF RX CSN | PD1 | P2-33 | Output, initial high |
| nRF RX CE | PD0 | P2-36 | Output, initial low |
| nRF RX IRQ | PD2 | P2-34 | EXTI2, falling edge, pull-up |

## SPI2 — ILI9341 displays

| Signal | MCU pin | Header | Configuration |
|---|---:|---:|---|
| SPI2 SCK | PB13 | P1-37 | AF5, push-pull |
| SPI2 MISO | PB14 | P1-38 | AF5 |
| SPI2 MOSI | PB15 | P1-39 | AF5, push-pull |
| TFT1 DC | PE10 | P1-28 | Output, initial low |
| TFT1 RST | PE11 | P1-29 | Output, initial high |
| TFT1 CS | PE12 | P1-30 | Output, initial high |
| TFT2 DC | PE13 | P1-31 | Output, initial low |
| TFT2 RST | PE14 | P1-32 | Output, initial high |
| TFT2 CS | PE15 | P1-33 | Output, initial high |

## SPI3 — nRF24L01 TX

| Signal | MCU pin | Header | Configuration |
|---|---:|---:|---|
| SPI3 SCK | PC10 | P2-37 | AF6, push-pull |
| SPI3 MISO | PC11 | P2-38 | AF6 |
| SPI3 MOSI | PC12 | P2-35 | AF6, push-pull |
| nRF TX CSN | PC8 | P2-45 | Output, initial high |
| nRF TX CE | PC9 | P2-46 | Output, initial low |
| nRF TX IRQ | PC6 | P2-47 | EXTI6, falling edge, pull-up |

## Enabled interrupt groups

| IRQ | Device signal |
|---|---|
| EXTI1_IRQn | MPU6050 INT (PB1) |
| EXTI2_IRQn | nRF24L01 RX IRQ (PD2) |
| EXTI3_IRQn | RC522 IRQ (PD3) |
| EXTI9_5_IRQn | nRF24L01 TX IRQ (PC6) |

All EXTI and DMA interrupt priorities are 5, which is compatible with the
FreeRTOS ISR-safe notification calls used by this project.
