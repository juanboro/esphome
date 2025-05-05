# esphome girs
Custom Component providing a GIRS (General InfraRed Server) for esphome

GIRS Server for ESPHome
=======================
This provides a simple GIRS component for esphome that allows transmitting and receiving IR (or RF) remote data from/to Esphome via TCP.  I use it with [IrScrutinizer](https://www.harctoolbox.org/IrScrutinizer.html) and it should hopefully work with the linux LIRC driver as well.  It provides only minimal Girs functionality - as most of what also is in Girs already has different (usually better) ways to do it with Esphome and/or Homeassistant.

TCP connectivity is provided via [this](https://github.com/juanboro/esphome/tree/mymaster/juanboro/tcp_server) compoent.  It also supports connecting via UART (serial port)

Usage
-----
```yaml
external_components:
  - source: github://juanboro/esphome
    components: [ tcp_server girs ]

tcp_server:
  - id: tcpgirs_id
    port: 33333

girs:
  tcp_server_id: tcpgirs_id
  receiver_id: remote_receiver_id
  transmitter_id: remote_transmitter_id

remote_receiver:
  id: remote_receiver_id
  # specific config as necesary

remote_transmitter:
  id: remote_transmitter_id
  # specific config as necesary

```

You can provde either receiver_id, transmitter_id or both, or neither.

Gist of using UART:
-------------------
```yaml
uart:
  id: girs_uart_id
  tx_pin: 1
  rx_pin: 3
  baud_rate: 115200

girs:
  id: girs_id
  receiver_id: remote_receiver_id
  transmitter_id: remote_transmitter_id
  uart_id: girs_uart_id
```

## final notes
This is still a work in progress.  There are surely some bugs, and I still have some additional parameters to expose to the yaml.  

Also per: [this](https://developers.esphome.io/contributing/code/) -- I have no idea how to *properly* understand/do what is wanted by this bullet: 
- Components specifically should not directly access other components -- for example, to publish to MQTT topics.

Guidance from ESPHome experts would be appreciated.

