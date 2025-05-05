# esphome tcp_server
Custom Component providing a tcp server for esphome

TCP Server for ESPHome
======================

Custom component for ESPHome to allow sending/receiving tcp messages. 

This component creates a TCP server listening on port 9000 (by default).  It allows you to define custom 'on_message' automations to forward messages via lambdas as desired, and to respond via a lambda call to the component.

This component should work on ESP-32 (both platforms) and ESP8266 with IPV4.  It has not (yet) been tested with IPV6 or the RP2040/BK*/RTL* controllers.

Component source is [here](https://github.com/juanboro/esphome/tree/mymaster/esphome/components/tcp_server)

Usage
-----

```yaml
external_components:
  - source: github://juanboro/esphome
    components: [ tcp_server ]
    
tcp_server:
```

It's not that interesting unless you at least define one or more of the automations: on_connect, on_message, or on_disconnect.  Each automation provides a "client_id" string (the address of the connecting cient).  The on_message automation also provides a "msg" string with messages received from the client.

Example echo server:
```yaml
tcp_server:
  - id: echotcp
    port: 9000

    on_message: 
      then:
        - lambda: |-
            id(echotcp).write(msg,client_id);  

            // you can broadcast to all connected clients like this:
            //id(echotcp).write(msg);  

    on_connect:
      then:
        - logger.log:
            format: "TCP Echo Now Connected to %s"
            args: ['client_id.c_str()']
            level: INFO
```

You can optionally create a `binary_sensor` to indicate whether a client is currently connected.

```yaml
tcp_server:
  id: tcp

binary_sensor:
  - platform: tcp_server
    tcp_server_id: tcp
    connected:
      name: "TCP Server Connected"

```

You can also optionally stream to/from an UART...
```yaml
uart:
  id: tcp_uart_id
  tx_pin: 1
  rx_pin: 3
  baud_rate: 9600

tcp_server:
  - id: serialtcp
    port: 8888
    uart_id: tcp_uart_id
```

## errata
This does not work correctly when using TCP logging on the ESP32 ESP-IDF platform.

## final notes
This is heavily based on this (and everything it is based on): [esphome-stream-server-v2](https://github.com/tube0013/esphome-stream-server-v2.git).

ESPHome Mysteries
-----------------
Given the fluid nature of IDF platforms, ESPHome, etc - this is likely to break as things are updated. The state of sockets isn't exactly clearly docmented as best I can tell for ESPHome.  This tries to use [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) when it is available - except it isn't available in ESPHome for the ESP-IDF platform (not clear why it can't be since I believe Arduino is now available to ESP-IDF).  

Also per: [this](https://developers.esphome.io/contributing/code/) -- I have no idea how to *properly* understand/do what is wanted by this bullet: 
- Components specifically should not directly access other components -- for example, to publish to MQTT topics.

Guidance from ESPHome experts would be appreciated.

