# Pin Map — Current Full Configuration

Target board: STM32F407G-DISC1 / STM32F407VGTx.

This file reflects the current
`stm32f4-freertos-multi-device-hub-main/LL_multi_device.ioc` configuration. It
supersedes the older map that placed the ILI9341 displays on SPI2.

## I2C1

| Signal | STM32F407 pin | Connected devices |
|---|---|---|
| I2C1_SCL | PB6 | BME280 SCL, MPU6500 SCL |
| I2C1_SDA | PB7 | BME280 SDA, MPU6500 SDA |
| MPU6500 interrupt | PC7 | MPU6500 INT |

Both SDA and SCL require pull-up resistors to 3.3 V. Some breakout boards already include them; measure the effective resistance before adding more.

## I2C2

| Signal | STM32F407 pin | Connected devices |
|---|---|---|
| I2C2_SCL | PB10 | ADXL345 SCL |
| I2C2_SDA | PB11 | ADXL345 SDA |

ADXL345 ALT ADDRESS low selects 7-bit address `0x53`; the driver also tries `0x1D`.

## SPI1 — RC522 and the single bidirectional nRF24L01

Shared bus:

| Signal | STM32F407 pin |
|---|---|
| SPI1_SCK | PB3 |
| SPI1_MISO | PB4 |
| SPI1_MOSI | PB5 |

Device control:

| Device | CS/CSN | CE | IRQ / other |
|---|---|---|---|
| RC522 | PD6 | — | RST PD7, IRQ PD3 |
| nRF24L01 RX/TX | PD1 | PD0 | PD2 |

Only one CS/CSN may be active at a time.

## SPI3 — two ILI9341 displays and one SPI SSD1306

Shared bus:

| Signal | STM32F407 pin |
|---|---|
| SPI3_SCK | PC10 |
| SPI3_MISO | PC11 |
| SPI3_MOSI | PC12 |

Device control:

| Device | CS | DC | RST |
|---|---|---|---|
| SSD1306 SPI | PC8 | PC9 | PC6 |
| ILI9341 TFT1 | PE12 | PE10 | PE11 |
| ILI9341 TFT2 | PE15 | PE13 | PE14 |

The TFTs are **not** on PB13/PB14/PB15 in this revision. Those pins conflicted with CAN2.

## CAN2 and SN65HVD230

STM32F407 side:

| Signal | STM32F407 pin | SN65HVD230 pin |
|---|---|---|
| CAN2_TX | PB13 | CTX / TXD |
| CAN2_RX | PB12 | CRX / RXD |
| 3.3 V | 3V3 | VCC |
| Ground | GND | GND |

Physical bus:

| F407 transceiver | F103 transceiver |
|---|---|
| CANH | CANH |
| CANL | CANL |
| GND | GND |

Use one 120-ohm termination resistor across CANH/CANL at each physical end of the bus. Do not add termination at every node.

CAN2 is configured for 500 kbit/s. The F103 node must use matching nominal bit timing.

### F407 communication LEDs

| Function | STM32F407 pin | Board LED |
|---|---|---|
| CAN receive state | PD12 | Green, active high |
| nRF24L01 receive state | PD13 | Orange, active high |

An incoming `HIGH` sets the corresponding output and `LOW` resets it.

### STM32F103 companion nRF24L01

| Signal | STM32F103 pin |
|---|---|
| SPI2_SCK | PB13 |
| SPI2_MISO | PB14 |
| SPI2_MOSI | PB15 |
| nRF24L01 IRQ | PA8 |
| nRF24L01 CSN | PA9 |
| nRF24L01 CE | PA10 |
| CAN state LED | PB0 |
| nRF24L01 state LED | PB1 |

The F103 and F407 radios use channel 76, 1 Mbit/s, dynamic payloads and the same five-byte address. Both radios require 3.3 V power and a common ground.

## DMA mapping

| Peripheral direction | DMA mapping |
|---|---|
| I2C1 RX | DMA1 Stream 0 |
| I2C2 RX | DMA1 Stream 3 |
| I2C2 TX | DMA1 Stream 7 |
| SPI1 RX | DMA2 Stream 0 |
| SPI1 TX | DMA2 Stream 3 |
| SPI3 RX | DMA1 Stream 2 |
| SPI3 TX | DMA1 Stream 5 |

I2C2 has separate RX and TX streams available to the dispatcher. The video
configuration uses ADXL345 on this bus.

## Enabled interrupts

All RTOS-aware peripheral IRQs are configured at preemption priority 5, subpriority 0.

- EXTI9_5 / line 7: MPU6500 interrupt on PC7
- EXTI2: the single nRF24L01 RX/TX interrupt on PD2
- EXTI3: RC522 interrupt on PD3
- I2C1 event and error
- I2C2 event and error
- SPI1
- SPI3
- DMA1 Stream 0, 2, 3, 5 and 7
- DMA2 Stream 0 and 3
- CAN2 TX
- CAN2 RX0
- CAN2 RX1
- CAN2 SCE

## Power and grounding checklist

- Connect all module grounds, both MCU grounds and both transceiver grounds.
- Use 3.3 V logic for STM32F407 GPIO.
- Check each breakout board’s regulator/level-shifter arrangement before applying 5 V.
- Give nRF24L01 modules local decoupling close to VCC/GND.
- Keep SPI and I2C jumpers short during initial validation.
- Power off before moving signal wires.
