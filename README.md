# TFmini Plus for ESPHome

![Benewake TFMini-Plus](tfmini-plus.png)

ESPHome external component adding plug-and-play support for the [Benewake TFmini-Plus LiDAR](https://en.benewake.com/TFminiPlus/index.html).\
Based on [TFMini-Plus](https://github.com/budryerson/TFMini-Plus) library.

## Features

- Exposes sensors:
  - distance
  - signal strength
  - device* temperature
  - device status
- Device control:
  - sleep/wake via ESPHome API services
  - frame rate setting
  - soft reset on device bootup setting
  - save settings on the device setting

> *device: the Benewake TFMini Plus sensor

## How to use

1) Add the external component and UART:

    ```yaml
    external_components:
      - source: github://nilvanis/esphome-tfmini-plus@main
        components: [tfmini_plus]
    ```

2) Enable custom API services:

    ```yaml
    api:
      custom_services: true   # enables sleep/wake services
    ```

3) Configure UART bus, where you connected the sensor:

    ```yaml
    uart:
      id: tfmmini
      rx_pin: 26
      tx_pin: 27
      baud_rate: 115200
    ```

4) Add the sensors:

    ```yaml
    sensor:
      - platform: tfmini_plus
        id: tfmini
        uart_id: tfmmini
        update_interval: 50ms   # how often ESPHome reads
        frame_rate: 20          # Hz output from the sensor to UART (0 = sleep)
        distance:
          name: "TFMini Plus Distance"
          unit_of_measurement: "cm"
          accuracy_decimals: 0
        signal_strength:
          name: "TFMini Plus Signal"
            filters:
              throttle: 1s
        temperature:
          name: "TFMini Plus Temperature"
            filters:
              throttle: 60s
        status:
          name: "TFMini Plus Status"
    ```

    > [!TIP]
    > Pair `frame_rate` with `update_interval` for better results.\
    > e.g., 20 Hz ↔ 50 ms, because:\
    > 20Hz = 20/s\
    > 50ms = 20s (1s is 1000ms; 1000ms/50ms = 20)

5) Optional wake/sleep automations (example with a LED strip):

    ```yaml
    light:
      - platform: neopixelbus
        # ... your light config ...
        on_turn_on:
          - lambda: 'id(tfmini)->wake_service();'
        on_turn_off:
          - lambda: 'id(tfmini)->sleep_service();'
    ```

    > [!TIP]
    > Services can be also called from Home Assistant via API:
    > - `esphome.<node_name>_tfmini_plus_wake`
    > - `esphome.<node_name>_tfmini_plus_sleep`

6) Performance considerations:

    It is highly recommended to disable DEBUG logs in case of high rate `update_interval` (faster than 1000ms)

    ```yaml
    logger:
      level: INFO
    ```

    You can also remove temperature & signal strength sensors if you don't use it.

## Details

- Distance is reported in cm
- Status is a text sensor that reflects device state and errors.
- Sleep sets frame rate to 0; all sensors go unavailable during SLEEP. Wake sets configured frame rate and brings back the sensors.
- Component checks communication with the device. In case of failure or continuus errors, sensors become unavailable. Retries every 60s.

## Status sensor states

- `READY`: Running normally.
- `SLEEP`: Put to sleep via API service (frame rate 0).
- `OFFLINE`: No valid frames for a while.
- `CHECKSUM` / `HEADER` / `TIMEOUT`: Frame/read issues; recovers when a good frame arrives.
- `WEAK`: Weak laser signal received.
- `STRONG`: Saturated laser signal received.
- `FLOOD`: Ambient light saturation.
- `PASS` / `FAIL`: Reply from a command (config/save/reset).
- `SERIAL`, `I2CREAD`, `I2CWRITE`, `I2CLENGTH`, `MEASURE`, `OTHER`: Rare/diagnostic codes.

## Technical notes

- Change filters (to reduce spam): distance publishes on ≥0.1 cm change; signal ≥1; temperature ≥0.05 °C. Unavailable publishes are forced periodically in sleep to punch through throttling filters.
- Errors (checksum/timeout) are counted and logged (at most once per minute). The device is marked OFFLINE if no valid frame arrives within 1 second (5 seconds right after wake). Distance/signal/temperature publish NaN when unavailable.
