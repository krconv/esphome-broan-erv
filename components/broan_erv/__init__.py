import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, select, sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
    ICON_FAN,
    ICON_GAUGE,
    ICON_FLASH,
    DEVICE_CLASS_POWER,
    STATE_CLASS_MEASUREMENT,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_WATT,
)
from esphome import pins

# Custom unit not in ESPHome const
UNIT_CUBIC_FEET_PER_MINUTE = "CFM"

AUTO_LOAD = ["select", "sensor"]
DEPENDENCIES = ["uart"]

CONF_SRC_ADDRESS = "src_address"
CONF_DST_ADDRESS = "dst_address"
CONF_POLL_INTERVAL = "poll_interval"
CONF_RE_DE_PIN = "re_de_pin"
CONF_LISTEN_ONLY = "listen_only"
CONF_POLL_REGISTERS = "poll_registers"
CONF_MODE = "mode"
CONF_SUPPLY_FAN_RPM = "supply_fan_rpm"
CONF_EXHAUST_FAN_RPM = "exhaust_fan_rpm"
CONF_SUPPLY_FAN_CFM = "supply_fan_cfm"
CONF_EXHAUST_FAN_CFM = "exhaust_fan_cfm"
CONF_POWER = "power"
CONF_OUTSIDE_TEMPERATURE = "outside_temperature"
CONF_RETURN_TEMPERATURE = "return_temperature"
CONF_SUPPLY_TEMPERATURE = "supply_temperature"

broan_erv_ns = cg.esphome_ns.namespace("broan_erv")
BroanERVComponent = broan_erv_ns.class_(
    "BroanERVComponent", cg.Component, uart.UARTDevice
)
BroanERVModeSelect = broan_erv_ns.class_(
    "BroanERVModeSelect", select.Select, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BroanERVComponent),
        cv.Optional(CONF_SRC_ADDRESS, default=0x11): cv.hex_uint8_t,
        cv.Optional(CONF_DST_ADDRESS, default=0x10): cv.hex_uint8_t,
        cv.Optional(CONF_POLL_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_RE_DE_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_LISTEN_ONLY, default=False): cv.boolean,
        cv.Optional(CONF_POLL_REGISTERS, default=[]): cv.ensure_list(cv.hex_uint16_t),
        cv.Optional(CONF_MODE): select.select_schema(
            BroanERVModeSelect,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_FAN,
        ),
        cv.Optional(CONF_SUPPLY_FAN_RPM): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
            icon=ICON_FAN,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EXHAUST_FAN_RPM): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
            icon=ICON_FAN,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SUPPLY_FAN_CFM): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_FEET_PER_MINUTE,
            icon=ICON_GAUGE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_EXHAUST_FAN_CFM): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_FEET_PER_MINUTE,
            icon=ICON_GAUGE,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            icon=ICON_FLASH,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_src_address(config[CONF_SRC_ADDRESS]))
    cg.add(var.set_dst_address(config[CONF_DST_ADDRESS]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_listen_only(config[CONF_LISTEN_ONLY]))

    for reg in config[CONF_POLL_REGISTERS]:
        cg.add(var.add_poll_register(reg))

    if rede_pin := config.get(CONF_RE_DE_PIN):
        pin = await cg.gpio_pin_expression(rede_pin)
        cg.add(var.set_re_de_pin(pin))

    if mode_config := config.get(CONF_MODE):
        sel = cg.new_Pvariable(mode_config[CONF_ID])
        await cg.register_component(sel, mode_config)
        await select.register_select(
            sel,
            mode_config,
            options=[
                "Off",
                "Minimum",
                "Medium",
                "Maximum",
                "Auto",
            ],
        )
        cg.add(sel.set_parent(var))
        cg.add(var.set_mode_select(sel))

    # Register optional sensors from config
    if supply_fan_rpm_config := config.get(CONF_SUPPLY_FAN_RPM):
        sens = await sensor.new_sensor(supply_fan_rpm_config)
        cg.add(var.set_supply_fan_rpm_sensor(sens))

    if exhaust_fan_rpm_config := config.get(CONF_EXHAUST_FAN_RPM):
        sens = await sensor.new_sensor(exhaust_fan_rpm_config)
        cg.add(var.set_exhaust_fan_rpm_sensor(sens))

    if supply_fan_cfm_config := config.get(CONF_SUPPLY_FAN_CFM):
        sens = await sensor.new_sensor(supply_fan_cfm_config)
        cg.add(var.set_supply_fan_cfm_sensor(sens))

    if exhaust_fan_cfm_config := config.get(CONF_EXHAUST_FAN_CFM):
        sens = await sensor.new_sensor(exhaust_fan_cfm_config)
        cg.add(var.set_exhaust_fan_cfm_sensor(sens))

    if power_config := config.get(CONF_POWER):
        sens = await sensor.new_sensor(power_config)
        cg.add(var.set_power_sensor(sens))