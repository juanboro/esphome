# Heavily based on/taken from the wake_on_lan component (more generic form of it), and a little on the udp component

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT
from esphome.core import CORE

DEPENDENCIES = ["network"]
CONF_ADDRESSES = "addresses"


def AUTO_LOAD():
    if CORE.is_esp8266 or CORE.is_rp2040:
        return []
    return ["socket"]


udp_broadcast_ns = cg.esphome_ns.namespace("udp_broadcast")
UDPBroadcastComponent = udp_broadcast_ns.class_("UDPBroadcastComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UDPBroadcastComponent),
        cv.Optional(
            CONF_ADDRESSES,
            default=[
                "255.255.255.255",
            ],
        ): cv.ensure_list(
            cv.ipv4address,
        ),
        cv.Optional(CONF_PORT, default=5007): cv.port,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    for address in config[CONF_ADDRESSES]:
        cg.add(var.add_address(str(address)))

    cg.add(var.set_port(config[CONF_PORT]))

    await cg.register_component(var, config)
