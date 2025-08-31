import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import TCPServerComponent

DEPENDENCIES = ["tcp_server"]

CONF_TCP_SERVER_ID = "tcp_server_id"
CONF_CONNECTED = "connected"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TCP_SERVER_ID): cv.use_id(TCPServerComponent),
    cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TCP_SERVER_ID])

    if count_config := config.get(CONF_CONNECTED):
        sens = await binary_sensor.new_binary_sensor(count_config)
        cg.add(parent.set_count_sensor(sens))
