# Hardware Validation Checklist

Do not mark an item complete from code inspection alone. Record the firmware
revision, wiring, power supply, observed result, and failure evidence.

## 1. Full-system baseline

- [ ] Build and flash the current Debug image.
- [ ] Confirm every `*_initialized` field in `g_runtime_debug` is true.
- [ ] Confirm `normal_monitoring_started == true`.
- [ ] Confirm `stale_subsystem_mask == 0`.
- [ ] Confirm all sensor values and display fields update.
- [ ] Run the complete system for at least one hour without an unexpected
      reset.

Record:

- `monitor_cycle_count`
- `watchdog_feed_count`
- `missing_heartbeat_mask`
- `minimum_free_stack_words[]`
- `free_heap_bytes`
- `minimum_ever_free_heap_bytes`

## 2. I2C protocol and DMA

- [ ] Capture START, write address, register address, repeated START, read
      address, payload, and STOP using a logic analyzer.
- [ ] Verify the BME280 payload length.
- [ ] Verify the MPU6050/MPU6500 14-byte payload.
- [ ] Confirm the event ISR only advances bounded states.
- [ ] Confirm the DMA completion path wakes the dispatcher.
- [ ] Confirm failed transfers do not increment valid-data counters.

## 3. I2C fault injection

- [ ] Temporarily disconnect BME280 SDA and reconnect it.
- [ ] Temporarily disconnect MPU6050/MPU6500 SDA and reconnect it.
- [ ] Hold SDA low and confirm BUSY detection.
- [ ] Confirm peripheral software reset is attempted.
- [ ] Confirm up to nine SCL recovery pulses and an explicit STOP are generated.
- [ ] Confirm `physical_recovery_count` changes.
- [ ] Confirm `physical_recovery_success_count` changes after a recoverable
      fault.
- [ ] Confirm a persistent fault sets the relevant stale bit and stops watchdog
      feeding.
- [ ] Confirm a watchdog reset occurs and the next boot reports
      `last_reset_was_watchdog == true`.

## 4. SPI and device progress

- [ ] Verify SPI chip select falls before the first clock and rises after the
      final bit.
- [ ] Confirm both RX and TX DMA streams complete for full-duplex jobs.
- [ ] Confirm RC522 Version register checks continue when no RFID card is
      present.
- [ ] Disconnect RC522 and verify stale bit 4.
- [ ] Disconnect nRF24L01 TX and verify stale bit 8; bit 16 may also appear
      because RX stops receiving packets.
- [ ] Disconnect nRF24L01 RX and verify the expected TX/RX dependency failure.
- [ ] Confirm temporary radio disconnection can recover after reconnection.
- [ ] Confirm persistent radio failure eventually causes a watchdog reset.

ILI9341 note: a write-only SPI transfer has no panel acknowledgement. Verify
the DMA pipeline and task progress, but do not claim that this proves physical
panel presence.

## 5. RTOS resource tests

- [ ] Record the minimum free stack words for every task after at least one
      hour.
- [ ] Confirm no queue stays permanently full.
- [ ] Confirm no mutex remains owned after transfer timeout or abort.
- [ ] Confirm the high-priority health task advances once per second under
      heavy I2C and SPI load.
- [ ] Trigger the stack-overflow hook in a temporary test build.
- [ ] Trigger the malloc-failed hook in a temporary test build.
- [ ] Restore safe settings after destructive tests.

## 6. Reset and startup tests

- [ ] Perform at least 20 cold power cycles.
- [ ] Perform repeated watchdog resets with all devices connected.
- [ ] Introduce a fault immediately after a watchdog reset and verify the
      bounded startup grace behavior.
- [ ] Confirm the debugger freezes IWDG while the core is suspended.
- [ ] Confirm standalone operation resets without debugger intervention.

## Test record template

```text
Date:
Firmware revision:
Board and wiring:
Power supply:
Logic analyzer and sample rate:
Test case:
Expected result:
Observed result:
Pass or fail:
Diagnostic values:
Screenshot or trace:
Notes:
```
