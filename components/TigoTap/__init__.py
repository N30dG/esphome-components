import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

from esphome.components import sensor

DEPENDENCIES = ["uart"]

CONF_TAP_ID = "tap-ID"



tigo_tap_ns = cg.esphome_ns.namespace("tigo_tap")
TigoTap = tigo_tap_ns.class_("TigoTap", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (cv.Schema({
        cv.GenerateID(): cv.declare_id(TigoTap)
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)