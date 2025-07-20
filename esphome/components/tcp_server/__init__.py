from esphome import automation
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_ON_CONNECT,
    CONF_ON_DISCONNECT,
    CONF_ON_MESSAGE,
    CONF_PORT,
    CONF_TRIGGER_ID,
)
from esphome.core import CORE

DEPENDENCIES = ["network"]


def AUTO_LOAD():
    if CORE.using_esp_idf:
        return ["socket", "uart"]
    return ["async_tcp", "uart"]


MULTI_CONF = True

CONF_TCP_SERVER_ID = "tcp_server_id"

tcp_server_ns = cg.esphome_ns.namespace("tcp_server")

TCPServerComponent = tcp_server_ns.class_("TCPServerComponent", cg.Component)
TCPServerTrigger = tcp_server_ns.class_(
    "TCPServerTrigger",
    automation.Trigger.template(cg.const_char_ptr, cg.size_t, cg.const_char_ptr),
)
TCPServerOnConnectTrigger = tcp_server_ns.class_(
    "TCPServerOnConnectTrigger",
    automation.Trigger.template(cg.const_char_ptr),
)
TCPServerOnDisconnectTrigger = tcp_server_ns.class_(
    "TCPServerOnDisconnectTrigger",
    automation.Trigger.template(cg.const_char_ptr),
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TCPServerComponent),
        cv.Optional(CONF_ON_MESSAGE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(TCPServerTrigger),
            }
        ),
        cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    TCPServerOnConnectTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    TCPServerOnDisconnectTrigger
                ),
            }
        ),
        cv.Optional(CONF_PORT): cv.port,
        cv.Optional(uart.CONF_UART_ID): cv.use_id(uart.UARTComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if CONF_PORT in config:
        cg.add(var.set_port(config[CONF_PORT]))

    await cg.register_component(var, config)

    for conf in config.get(CONF_ON_MESSAGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_onmessage_trigger(trigger))
        await automation.build_automation(
            trigger,
            [
                (cg.const_char_ptr, "client_id"),
                (cg.const_char_ptr, "msg"),
                (cg.size_t, "len"),
            ],
            conf,
        )

    for conf in config.get(CONF_ON_CONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_onconnect_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.const_char_ptr, "client_id")], conf
        )

    for conf in config.get(CONF_ON_DISCONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ondisconnect_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.const_char_ptr, "client_id")], conf
        )

    if uart.CONF_UART_ID in config:
        await uart.register_uart_device(var, config)
