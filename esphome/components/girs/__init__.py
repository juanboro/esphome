import esphome.codegen as cg
from esphome.components import remote_base, tcp_server
import esphome.config_validation as cv
from esphome.const import CONF_ID

girs_ns = cg.esphome_ns.namespace("girs")
GirsComponent = girs_ns.class_(
    "GirsComponent", cg.Component, remote_base.RemoteReceiverListener
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GirsComponent),
        cv.Optional(remote_base.CONF_RECEIVER_ID): cv.use_id(
            remote_base.RemoteReceiverBase
        ),
        cv.Optional(remote_base.CONF_TRANSMITTER_ID): cv.use_id(
            remote_base.RemoteTransmitterBase
        ),
        cv.Optional(tcp_server.CONF_TCP_SERVER_ID): cv.use_id(
            tcp_server.TCPServerComponent
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if tcp_server.CONF_TCP_SERVER_ID in config:
        server = await cg.get_variable(config[tcp_server.CONF_TCP_SERVER_ID])
        cg.add(var.set_tcp_server(server))

    await cg.register_component(var, config)

    if remote_base.CONF_RECEIVER_ID in config:
        await remote_base.register_listener(var, config)
        cg.add(var.set_can_rx(True))

    if remote_base.CONF_TRANSMITTER_ID in config:
        transmitter_ = await cg.get_variable(config[remote_base.CONF_TRANSMITTER_ID])
        cg.add(var.set_transmitter(transmitter_))
        cg.add(var.set_can_tx(True))
        await remote_base.register_transmittable(var, config)
