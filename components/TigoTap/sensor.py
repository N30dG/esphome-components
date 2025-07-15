import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, UNIT_EMPTY, ICON_EMPTY
from . import TigoTap, CONF_TAP_ID, tigo_tap_ns

DEPENDENCIES = ["TigoTap"]

CONF_PV_BARCODE = "barcode"
CONF_PV_SENSOR_TYPE = "type"
CONF_PV_SENSOR_TIMEOUT = "sensor_timeout"

Sensor = tigo_tap_ns.class_("Sensor", cg.PollingComponent)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        Sensor,
        unit_of_measurement=UNIT_EMPTY,
        icon=ICON_EMPTY,
        accuracy_decimals=3,
    )
    .extend(
        {
            cv.GenerateID(CONF_TAP_ID): cv.use_id(TigoTap),
            cv.Required(CONF_PV_SENSOR_TYPE): cv.string,
            cv.Required(CONF_PV_BARCODE): cv.string,
            cv.Optional(CONF_PV_SENSOR_TIMEOUT, default=60): cv.int_range(0, 255),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_TAP_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    cg.add(var.setSensorTimeout(config[CONF_PV_SENSOR_TIMEOUT]))
    cg.add(var.setType(config[CONF_PV_SENSOR_TYPE]))
    cg.add(paren.register_sensor(var, config[CONF_PV_BARCODE]))