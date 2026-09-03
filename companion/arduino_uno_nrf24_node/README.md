# Arduino Uno NRF24L01 companion node

Install the Arduino `RF24` library and upload `arduino_uno_nrf24_node.ino`.

| Signal | Arduino Uno |
|---|---|
| NRF CE | D9 |
| NRF CSN | D10 |
| NRF MOSI | D11 |
| NRF MISO | D12 |
| NRF SCK | D13 |
| External LED through a series resistor | D4 |

Use 3.3 V for the NRF and place local decoupling close to it. The sketch
matches the F407 settings: channel 76, 1 Mbit/s, CRC16, dynamic payloads,
auto acknowledgement and address `E7:E7:E7:E7:E7`.
