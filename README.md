# ESP32 Based Mesh Network Solution for Managing Practical Lessons


## Introduction

This project is done as a Informatics Bsc thesis

The no_router example code was configured to implement the following features
- Use of built-in button on the boards and sending a message to the root node with every press
- Read 1-wire temperature values (from [DS18B20 shield](https://grobotronics.com/wemos-d1-mini-temperature-shield-ds18b20.html?sl=en))
- Timer task for sending temperature values to the root node
- Control the built-in RGB LED by indicating the conection status and device's layer on the mesh network
- Reading and writing to USB from the UART0 channel (through the built-in USB interface)
- Broadcast messages upstream to root node for sending payloads to USB interface
- Broadcasting messages coming from USB interface to all child nodes

## Node.js application as a control panel

[ESP32 mesh network control panel
](https://github.com/martenainult/smort-panel)

## Devices used
The project was implemented and tested with the following boards:

- Firebeetle 2 ESP32-E 4Mb : [Docs](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654)
- Az-delivery ESP32 DevKitC V2 [Docs](https://cdn.shopify.com/s/files/1/1509/1638/files/8_BA_DevKitV2_4a42c67f-225d-4769-b4cf-c5273b9cf84b.pdf?v=1649144286)
- Wemos D1 S2 Mini ESP32-S2 [Docs](https://www.wemos.cc/en/latest/s2/s2_mini.html)



## Configure

To run this example, you need at least two development boards, one configured as a root node, and the others as a non-root node. The root node is selected manually through config and flashed separately to the root device

You need to go to the submenu `Example Configuration` and configure one device as a root node, and the others as non-root nodes with `idf.py menuconfig`(CMake). 

- Root node: There is only one root node in an ESP-Mesh-Lite network. `MESH-LITE` networks can be differentiated by their `MESH_LITE_ID` and channels.
- Non-root node: Includes leaf nodes and intermediate nodes, which automatically select their parent nodes according to the network conditions.
	- Leaf node: A leaf node cannot also be an intermediate node, which means leaf node cannot has any child nodes.



You can also go to the submenu `Component config -> ESP Wi-Fi Mesh Lite`, and configure the ESP-Mesh-Lite related parameters such as the max number of layers allowed and the `MESH_LITE_ID`.


## Run

###  General steps in app_main
1. Initialize Wi-Fi, and start ESP-Mesh-Lite according to configuration;
2. Initialize RGB LED of WS2812 
2. Start the "`print_system_info`" timer task for outputting network data every 2 seconds

### Root  node specific steps in app_main 
1. Initialization of UART connection on UART0
2. `mesh_lite_config.join_mesh_without_configured_wifi = false;`
3. Start the "`uart_rx_task`" task that continuously reads data coming from USB

### Child node specific steps in app_main
1. Initialization of Built-in button (set with the `GPIO_BUTTON_1`)
2. Initialization of DS18B20 by 1-wire (set with the `GPIO_TEMP_1`)
2. `mesh_lite_config.join_mesh_without_configured_wifi = true;`


## Configuration files

The following items are added to the `sdkconfig.defaults` for optimization 

~~~
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHFREQ="80m"

CONFIG_APPTRACE_DEST_UART0=
CONFIG_ETH_USE_SPI_ETHERNET=

CONFIG_ESP_HTTP_CLIENT_ENABLE_HTTPS=
CONFIG_XTAL_FREQ_40=y
CONFIG_XTAL_FREQ_AUTO=
CONFIG_XTAL_FREQ=40

CONFIG_LWIP_TCPIP_TASK_PRIO=3

CONFIG_LWIP_TCP_SND_BUF_DEFAULT=14360
CONFIG_LWIP_TCP_WND_DEFAULT=14360
CONFIG_LWIP_TCP_RECVMBOX_SIZE=12

CONFIG_BRIDGE_SOFTAP_MAX_CONNECT_NUMBER=6
CONFIG_JOIN_MESH_IGNORE_ROUTER_STATUS=y

CONFIG_TCP_SND_BUF_DEFAULT=14360
CONFIG_TCP_WND_DEFAULT=14360
CONFIG_TCP_RECVMBOX_SIZE=12

CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT=
~~~
