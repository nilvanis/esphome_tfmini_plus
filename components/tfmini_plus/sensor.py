import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import api, sensor, text_sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_RULER,
    ICON_SIGNAL,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart", "text_sensor"]
AUTO_LOAD = ["text_sensor"]

tfmini_plus_ns = cg.esphome_ns.namespace("tfmini_plus")
CustomAPIDevice = api.api_ns.class_("CustomAPIDevice")
TFMiniPlusComponent = tfmini_plus_ns.class_("TFMiniPlusComponent", cg.PollingComponent, uart.UARTDevice, CustomAPIDevice)

CONF_DISTANCE = "distance"
CONF_SIGNAL_STRENGTH = "signal_strength"
CONF_TEMPERATURE = "temperature"
CONF_STATUS = "status"
CONF_FRAME_RATE = "frame_rate"
CONF_SOFT_RESET = "soft_reset"
CONF_SAVE_SETTINGS = "save_settings"


def _validate_frame_rate(value):
    value = cv.int_(value)
    valid_rates = [0, 1, 2, 5, 10, 20, 25, 50, 100, 125, 200, 250, 500, 1000]
    if value not in valid_rates or (value != 0 and 1000 % value != 0):
        raise cv.Invalid(
            f"Frame rate must be one of {valid_rates}. Use 0 to pause measurements."
        )
    return value


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TFMiniPlusComponent),
            cv.Optional(CONF_FRAME_RATE, default=100): _validate_frame_rate,
            cv.Optional(CONF_SOFT_RESET, default=False): cv.boolean,
            cv.Optional(CONF_SAVE_SETTINGS, default=False): cv.boolean,
            cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement="cm",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DISTANCE,
                icon="mdi:arrow-expand-horizontal",
            ),
            cv.Optional(CONF_SIGNAL_STRENGTH): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon=ICON_SIGNAL,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                icon=ICON_THERMOMETER,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:text",
            ),
        }
    )
    .extend(cv.polling_component_schema("100ms"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_frame_rate(config[CONF_FRAME_RATE]))
    cg.add(var.set_soft_reset(config[CONF_SOFT_RESET]))
    cg.add(var.set_save_settings(config[CONF_SAVE_SETTINGS]))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_SIGNAL_STRENGTH in config:
        sens = await sensor.new_sensor(config[CONF_SIGNAL_STRENGTH])
        cg.add(var.set_signal_strength_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_STATUS in config:
        cg.add_define("USE_TFMINI_PLUS_STATUS_SENSOR")
        ts = await text_sensor.new_text_sensor(config[CONF_STATUS])
        cg.add(var.set_status_sensor(ts))
