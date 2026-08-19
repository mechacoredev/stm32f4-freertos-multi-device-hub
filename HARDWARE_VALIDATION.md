# Hardware Validation Checklist

Use this checklist in order. A clean build is only the starting point; each stage must be observed on hardware before the next one is trusted.

## 1. Pre-power inspection

- [ ] Confirm the wiring against [PIN_MAP.md](PIN_MAP.md).
- [ ] Confirm common ground between all modules, STM32F407, STM32F103 and both CAN transceivers.
- [ ] Check for shorts between 3.3 V, 5 V and ground.
- [ ] Confirm I2C pull-ups and idle-high SDA/SCL.
- [ ] Confirm every SPI device has a unique CS/CSN.
- [ ] Confirm SPI3 TFT/nRF TX wiring; do not use the obsolete SPI2 TFT pin map.
- [ ] Confirm CANH-to-CANH and CANL-to-CANL.
- [ ] Confirm exactly two 120-ohm CAN termination resistors at the physical ends.
- [ ] Confirm both CAN nodes use 500 kbit/s timing.

## 2. Baseline startup

Start with `is_mcu_reset_allowed = false` so diagnostics remain visible instead of immediately rebooting.

- [ ] Flash the STM32F103 CAN test firmware.
- [ ] Start the STM32F407 Debug build.
- [ ] Confirm every enabled task reaches its normal loop.
- [ ] Confirm no HardFault, assert or unexpected reset.
- [ ] Confirm `g_system_bus_health` entries settle to `SYSTEM_BUS_STATE_OK`.
- [ ] Record startup time and any initial recovery attempt.

## 3. I2C1 validation

Observe:

- `bme280_display_data`
- `mpu6050_display_data`
- the I2C1 entry in `g_system_bus_health`

Tests:

- [ ] BME280 temperature, pressure and humidity update continuously.
- [ ] MPU6500 all three accelerometer and all three gyroscope axes update.
- [ ] Both sources appear in health diagnostics over time.
- [ ] Data ages remain bounded during normal operation.
- [ ] No recurring AF, bus error, arbitration loss or timeout.

Fault injection:

1. Record the normal counters.
2. Disconnect SCL or one sensor connection briefly.
3. Confirm the bus becomes suspect/failed.
4. Reconnect the wire.
5. Confirm a recovery attempt occurs.
6. Count recovery as successful only after valid sensor data updates again.
7. If recovery does not restore data, confirm failure count increases.
8. With reset permission still false, confirm reset is reported as suppressed after the deadline.
9. Repeat once with reset permission true only after the local recovery behavior is understood.

## 4. SPI1 validation

Devices: RC522 and nRF24L01 RX.

- [ ] RC522 operations complete and its display/debug data updates.
- [ ] nRF24L01 RX receives packets.
- [ ] CS/CSN lines never overlap.
- [ ] DMA/IRQ success counts increase without error growth.
- [ ] Disconnect and reconnect each device separately.
- [ ] Confirm SPI recovery is followed by device-level validation.
- [ ] Do not count a peripheral reset alone as a successful recovery.

## 5. SPI3 shared-bus validation

Devices: nRF24L01 TX, ILI9341 TFT1 and ILI9341 TFT2.

- [ ] nRF TX packets continue while both displays refresh.
- [ ] TFT1 and TFT2 show their intended content.
- [ ] `ili9341_dma_success_count` increases.
- [ ] `ili9341_dma_error_count` remains stable.
- [ ] No display corruption appears during nRF traffic.
- [ ] Logic-analyzer capture confirms only one CS/CSN is low at a time.
- [ ] Disconnect one SPI3 device and confirm the other clients remain diagnosable.

## 6. CAN2 validation

Observe:

- `g_can2_debug`
- `g_can2_last_tx_frame`
- `g_can2_last_rx_frame`
- the CAN2 entry in `g_system_bus_health`

Baseline:

- [ ] F407 TX request and TX complete counts increase.
- [ ] F103 receives F407 frames.
- [ ] F407 RX IRQ/received/processed counts increase from F103 frames.
- [ ] Last TX/RX IDs, DLC and payloads match expectations.
- [ ] No warning, error-passive or bus-off counters increase.

Fault injection:

- [ ] Disconnect or power down the F103 node.
- [ ] Confirm missing ACK/progress is detected within the configured 3-second window.
- [ ] Confirm CAN recovery attempts are recorded.
- [ ] Restore the node and confirm RX/TX progress resumes.
- [ ] Confirm recovery success is recorded only after real CAN progress.
- [ ] Temporarily remove one termination resistor and record error behavior; restore it immediately after the test.

## 7. Full simultaneous-load test

Current result: **Passed for 15 minutes** with the complete connected configuration. All enabled devices operated as expected during that observation window. The individual evidence items below should still be retained for repeatable future test records.

- [ ] Enable all sensor, radio, display and CAN tasks.
- [ ] Run I2C1, SPI1, SPI3 and CAN2 concurrently.
- [ ] Confirm every source update count increases.
- [ ] Confirm maximum data ages stay within their intended periods.
- [ ] Confirm RTOS tasks continue producing health heartbeats.
- [ ] Confirm no queue/ring overflow.
- [ ] Confirm no DMA stream remains permanently active.
- [ ] Confirm no unexpected recovery loop.
- [ ] Capture a logic-analyzer trace for I2C and both SPI buses.
- [ ] Capture CAN traffic or at least both-node counters and last frames.

## 8. Endurance

The final duration is a project decision; one hour is a useful development checkpoint, not a product-qualification test.

Current result: the 15-minute integration run passed. The one-hour and longer-duration checks below remain pending.

- [ ] Run the complete system for at least one hour without breakpoints.
- [ ] Record start/end counters and data ages.
- [ ] Record all recovery attempts and their verified outcomes.
- [ ] Verify no counter stops unexpectedly.
- [ ] Verify displays remain responsive.
- [ ] Verify CAN communication remains bidirectional.
- [ ] Repeat after a warm debugger restart and after a full power cycle.

## Result record

For every test, save:

- firmware commit/revision
- wiring revision
- power source
- test duration
- enabled tasks
- initial/final diagnostic snapshots
- logic-analyzer/CAN captures
- observed fault
- recovery action
- whether valid communication actually resumed

Do not mark the complete system validated until the simultaneous-load and fault-injection stages have both passed.
