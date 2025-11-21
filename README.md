# ESPHome TFmini Plus External Component

ESPHome external component to read the TFmini Plus LiDAR over UART. Handles temporary disconnects gracefully, exposes core measurements, and adds sleep/wake services for power saving. Based on [TFMini-Plus](https://github.com/budryerson/TFMini-Plus) library.

## Features

- Distance (cm), signal strength, and TFMini Plus temperature sensors.
- Status text sensor (reports `OK`/`OFFLINE`/`SLEEPING` and [sensor error states](#status-codes-text-sensor)).
- Offline handling:
  - Marks sensors unavailable when no frames arrive.
  - Retries once per 60 seconds if the device is missing at boot or drops out.
  - Errors (checksum/timeout) are counted and logged at most once per minute.
  - Device is declared offline only if no valid frame arrives within 1 second.
- Optional soft reset at boot and configurable frame rate; can persist settings on the device.
- Sleep/wake API services to stop/start measurements without rebooting (frame rate set to 0 on sleep).

## Usage (ESPHome YAML)

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/nilvanis/esphome_tfmini_plus
    components: [tfmini_plus]

# Highly recommended to disable DEBUG logs
# in case of high rate update_interval
logger:
  level: INFO


uart:
  id: tfmmini
  rx_pin: 26
  tx_pin: 27
  baud_rate: 115200

api:
  custom_services: true     # needed for sleep/wake services

sensor:
  - platform: tfmini_plus
    id: tfmini
    uart_id: tfmmini
    update_interval: 50ms      # how often sensor is polled for result
    frame_rate: 20             # Hz, how fast sensor polls
    # soft_reset: false        # optional soft reset at boot
    # save_settings: false     # persist any sent config commands
    distance:
      name: "TFMini Plus: Distance"
      unit_of_measurement: "cm"
      accuracy_decimals: 0
    signal_strength:
      name: "TFMini Plus: Signal"
      filters:
        throttle: 1s
    temperature:
      name: "TFMini Plus: Temperature"
      filters:
        throttle: 1s
    status:
      name: "TFMini Plus: Status"

...
# automations
    on_turn_on:
      - lambda: 'id(tfmini)->wake_service();'   # trun on LiDAR
    on_turn_off:
      - lambda: 'id(tfmini)->sleep_service();'  # turn off LiDAR
```

## Sleep/Wake Services

- `wake_service()`: sets frame rate to 0, marks sensors unavailable.
- `sleep_service()`: reapplies configured frame rate and resumes polling.

You can call the services from Home Assistant Developer Tools → Services or in automations:

- `esphome.<node_name>_tfmini_plus_sleep`
- `esphome.<node_name>_tfmini_plus_wake`

## Configuration Options

- `frame_rate` (default 100): Lidar polls/sec. Allowed values {`0`, `1`, `2`, `5`, `10`, `20`, `25`, `50`, `100`, `125`, `200`, `250`, `500`, `1000`}; 0 pauses output.
- `soft_reset` (default false): Issue `SOFT_RESET` at boot.
- `save_settings` (default false): Send `SAVE_SETTINGS` after configuration commands.
- `update_interval` (default 100ms): Polling interval for reading frames from the UART buffer.
- `status` sensor: Optional; if omitted, no text_sensor dependency is linked.

## Behavior Notes

- If the device is offline (no valid frame), sensors publish `unavailable` (NaN) and status shows OFFLINE.
- Retry cadence while offline: once every 60 seconds.
- Error handling: checksum/timeouts increment an error counter (logged at most once per minute); device moves to OFFLINE if no valid frame arrives within 1 second. Status text sensor (if configured) reports: `READY`, `SERIAL`, `HEADER`, `CHECKSUM`, `TIMEOUT`, `PASS`, `FAIL`, `I2CREAD`, `I2CWRITE`, `I2CLENGTH`, `WEAK`, `STRONG`, `FLOOD`, `MEASURE`, `OFFLINE`, `SLEEPING`, `OTHER`.
- Measurement error codes (weak signal, saturation, ambient flood) set status to `WEAK`/`STRONG`/`FLOOD` and mark sensors unavailable for that update.

## Status Codes (text sensor)

- `READY`: Normal operation; last frame valid.
- `OFFLINE`: No valid frames; device not responding.
- `SLEEPING`: Device put into frame-rate 0 sleep via service.
- `HEADER`: Expected frame header (0x59 0x59) not found before timeout; no data or line noise.
- `CHECKSUM`: Frame or reply checksum mismatch; corrupted data.
- `TIMEOUT`: Timed out waiting for a frame or command reply.
- `WEAK` (`WEAK_SIGNAL`): Distance -1 with low strength (≤100); target too weak/dim.
- `STRONG` (`STRONG_SIGNAL`): Strength -1; receiver saturated.
- `FLOOD` (`FLOOD_LIGHT`): Distance -4; ambient light saturation.
- `PASS`: Last command reported success.
- `FAIL`: Last command reported failure.
- `I2CREAD` / `I2CWRITE` / `I2CLENGTH`: Reserved mirror codes from the reference lib; not expected in UART mode.
- `MEASURE`: Reserved code; not normally used here.
- `OTHER`: Fallback for unexpected conditions.
