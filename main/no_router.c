/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/socket.h>
/* 
#include "onewire_bus.h"
#include "ds18b20.h"
 */

#include "driver/gpio.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif

#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "iot_button.h"
#include "led_strip.h"
#include <cJSON.h>
#include "driver/uart.h"
#include "soc/uart_struct.h"

static const char *TAG = "no_router";

// GPIO definitions
#if CONFIG_IDF_TARGET_ESP32S2 
    #define GPIO_BUTTON_1 0 // S2
    #define GPIO_TEMP_1 33 // S2 - LOLIN 

#elif CONFIG_IDF_TARGET_ESP32
    #define GPIO_BUTTON_1 27 // ESP32-E
    // #define GPIO_BUTTON_1 0 // ESP32
    #define GPIO_TEMP_1 4 // ESP32 & ESP32-E

#endif

// UART objects
QueueHandle_t uart_queue;
uint8_t *uart_data;
const int uart_buffer_size = (1024 * 2);
static led_strip_handle_t SOC_RGB;
const uart_port_t uart_num = UART_NUM_0;

// Answer states
static const char *answer_options[] = {"A", "B", "C", "D"};
static int i_answer = 0;
static uint8_t len_options = sizeof(answer_options) / sizeof(answer_options[0]);;

uint8_t hope = 0;

// Temperature handles
/* ds18b20_device_handle_t ds18b20_obj; */
float temperature; 

// -->


// FUNCTIONS
void set_rgb_value(uint8_t level_in) {
    switch (level_in)
    {
        case 0:
            // set pink on layer 0
            /*
                - strip         LED strip
                - index         index of pixel to set
                - hue           hue part of color (0 - 360)
                - saturation    saturation part of color (0 - 255)
                - value         value part of color (0 - 255)
            */
            led_strip_set_pixel_hsv(SOC_RGB, 0, 315, 255, 50);
            break;    
        case 1:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 120, 255, 50);
            break;

        case 2:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 45, 255, 50);
            break;    
        
        case 3:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 180, 255, 50);
            break;    

        case 4:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 225, 255, 50);
            break;

        case 5:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 270, 255, 50);
            break;
        case 6:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 350, 255, 50);
            break;
        default:
            led_strip_set_pixel_hsv(SOC_RGB, 0, 0, 255, 50);
            break;  
    }
    led_strip_refresh(SOC_RGB);
};

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    static const char *local_tag = "print_system_info_timercb";
    uint8_t primary                 = 0;
    uint8_t sta_mac[6]              = {0};
    wifi_ap_record_t ap_info        = {0};
    wifi_second_chan_t second       = 0;
    wifi_sta_list_t wifi_sta_list   = {0x0};
    uint8_t device_mesh_layer       = esp_mesh_lite_get_level();;

    set_rgb_value(device_mesh_layer);

    if (device_mesh_layer > 1) {
        esp_wifi_sta_get_ap_info(&ap_info);
    } else if (device_mesh_layer == 0) {
        ESP_LOGI(TAG, "No station available. Try %d", hope);
        if (hope > 3) {
            hope = 0;
            esp_mesh_lite_start();
            return;
        }
        hope = hope + 1;
    };
    
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, free heap: %"PRIu32"", primary,
             esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

void send_ping_to_nodes(char * eventType) {
    static const char *local_tag = "send_ping_to_nodes";

    cJSON * event_json = cJSON_CreateObject();
    if (event_json) {
        cJSON_AddStringToObject(event_json, "ping", eventType); 
        
    } else {
	    ESP_LOGI(local_tag,"event_json missing");
        return;
    }

    esp_mesh_lite_try_sending_msg("ping_info", "ping_info_ack", 5, event_json, &esp_mesh_lite_send_broadcast_msg_to_child);

}
void send_json_to_root(char * eventType, cJSON * event_json) {
    static const char *local_tag = "send_json_to_root";

    uint8_t sta_mac[6]              = {0};
    wifi_ap_record_t ap_info        = {0};
    char mac_str[MAC_MAX_LEN];
    char ap_mac_str[MAC_MAX_LEN];
    uint8_t device_mesh_layer = esp_mesh_lite_get_level();

    ESP_LOGI(local_tag, "%d", device_mesh_layer);
    set_rgb_value(device_mesh_layer);

    esp_wifi_get_mac(WIFI_IF_STA, sta_mac);
    esp_wifi_sta_get_ap_info(&ap_info);

    snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(sta_mac));
    snprintf(ap_mac_str, sizeof(ap_mac_str), MACSTR, MAC2STR(ap_mac_str));

    if (event_json) {
        cJSON_AddStringToObject(event_json, "event", eventType); 
        cJSON_AddStringToObject(event_json, "s_mac", mac_str);
        cJSON_AddNumberToObject(event_json, "lev", device_mesh_layer);
        if (device_mesh_layer != 0) {
            cJSON_AddStringToObject(event_json, "p_mac", ap_mac_str); 
        }
        
	    ESP_LOGD(local_tag,"s_json filled");
    } else {
	    ESP_LOGI(local_tag,"event_json missing");
        return;
    }

    esp_mesh_lite_try_sending_msg("event_info", "event_info_ack", 5, event_json, &esp_mesh_lite_send_msg_to_root);

}

void tempsensor_example(TimerHandle_t timer)
{
    static const char *local_tag = "tempsensor_example";
    
/* 
    ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20_obj));
    ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20_obj, &temperature));
    ESP_LOGI(TAG, "temperature read from DS18B20: %.2f C", temperature);

    uint8_t len_temp = 10;
    char temp[10];
    char temp_parsed[len_temp+1];

    sprintf(temp, "%f", temperature);
    strncpy(temp_parsed, temp, len_temp);

    ESP_LOGI(local_tag, "Temperature out celsius %s°C", temp_parsed);
 */


    cJSON * event_json = cJSON_CreateObject();
    cJSON_AddStringToObject(event_json, "temp", temp_parsed);
    send_json_to_root("sensor", event_json);

    cJSON_Delete(event_json);
}




static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}


// <-- BUTTONS INIT

static button_config_t initGpioButton_Short(int pinNumber) {
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS, // set the long press time
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS, // set the short press time
        // The default value is 150 ms, but we are going to use the minimum, 50 ms
        .gpio_button_config = {
            .gpio_num = pinNumber,
            .active_level = 0,
        },
    };
    return gpio_btn_cfg;
};

static void button_single_click_cb(void *arg,void *usr_data)
{
    char local_tag[] = "button_single_click_cb";
    ESP_LOGI(local_tag, "Button pressed");

    i_answer = (i_answer + 1) % (len_options);
    ESP_LOGI(local_tag, "i_answer: %d", i_answer);
    ESP_LOGI(local_tag, "len_options: %d", len_options);
    
    cJSON * s_json = cJSON_CreateObject(); 

    cJSON_AddStringToObject(s_json, "opt", answer_options[i_answer]); 

    send_json_to_root("answer", s_json);
    ESP_LOGI(local_tag, "Answer sent to root");

    cJSON_Delete(s_json);
};

// ---> 
void write_to_uart(char * data) 
{
    // MEMORY LEAK FROM HERE
    // add newline to string
    char *newstr = malloc(strlen(data) + 2);
    strcpy(newstr, data);
    strcat(newstr, "\n");
    ESP_LOGD("write_to_uart", "Writing to UART:%s", newstr);

    uart_write_bytes(uart_num, newstr, strlen(newstr));

    free(newstr);
}

// from the example of: https://github.com/espressif/esp-idf/blob/v5.1.4/examples/peripherals/uart/uart_async_rxtxtasks/main/uart_async_rxtxtasks_main.c
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(uart_buffer_size+1);
    while (1) {
        const int rxBytes = uart_read_bytes(uart_num, data, uart_buffer_size, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            send_ping_to_nodes("ping");
        }
    }
    free(data);
}

// <-- COMMUNICATION CALLBACKS

static cJSON* event_info_process(cJSON *payload, uint32_t seq)
{
    char local_tag[] = "event_info_process";
    ESP_LOGD(local_tag, "reached");

    char * rendered = cJSON_PrintUnformatted(payload);
	ESP_LOGD(local_tag,"%s\n", rendered);

    write_to_uart(rendered);

    // Memory leak solution source: https://github.com/DaveGamble/cJSON/issues/297
    cJSON_free(rendered);

    return NULL;
}

static cJSON* ping_info_ack_process(cJSON *payload, uint32_t seq)
{
    return NULL;
}

// SEND PING BROADCAST TO CHILDREN 

static cJSON* ping_info_process(cJSON *payload, uint32_t seq)
{
    char local_tag[] = "event_info_process";
    ESP_LOGD(local_tag, "reached");

    i_answer = (i_answer + 1) % (len_options);
    ESP_LOGI(local_tag, "i_answer: %d", i_answer);
    ESP_LOGI(local_tag, "len_options: %d", len_options);
    
    cJSON * s_json = cJSON_CreateObject(); 

    cJSON_AddStringToObject(s_json, "opt", answer_options[i_answer]); 

    send_json_to_root("ping", s_json);
    ESP_LOGI(local_tag, "Answer sent to root");

    cJSON_Delete(s_json);

    send_ping_to_nodes("ping");


    return NULL;
}

static cJSON* event_info_ack_process(cJSON *payload, uint32_t seq)
{
    ESP_LOGD("event_info_ack_process", "event callback received");

    return NULL;
}

/**
 * @brief Register a message listening callback event on the network
 */
static const esp_mesh_lite_msg_action_t node_event_action[] = {
    // Message to listen to, 
    {"event_info", "event_info_ack", event_info_process},
    {"event_info_ack", NULL, event_info_ack_process},

    {NULL, NULL, NULL} /* Must be NULL terminated */
};

static const esp_mesh_lite_msg_action_t node_ping_action[] = {
    // Message to listen to, 
    {"ping_info", "ping_info_ack", ping_info_process},
    {"ping_info_ack", NULL, ping_info_ack_process},

    {NULL, NULL, NULL} /* Must be NULL terminated */
};

// --> 

static void wifi_init(void)
{
    // Station
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x0, sizeof(wifi_config_t));
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
    strlcpy((char *)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = CONFIG_MESH_CHANNEL;
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[32];
    uint8_t softap_mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));

#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
    snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
    snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
    esp_mesh_lite_set_softap_ssid_to_nvs(softap_ssid);
    esp_mesh_lite_set_softap_psw_to_nvs(CONFIG_BRIDGE_SOFTAP_PASSWORD);
    esp_mesh_lite_set_softap_info(softap_ssid, CONFIG_BRIDGE_SOFTAP_PASSWORD);
}

void app_main()
{
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    mesh_lite_config.join_mesh_ignore_router_status = true;


// ROOT NODE exclusive code 
    #if CONFIG_MESH_ROOT
        mesh_lite_config.join_mesh_without_configured_wifi = false;

        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        };
        uart_data = (uint8_t *) malloc(uart_buffer_size );

        uart_param_config(uart_num, &uart_config);
        uart_set_pin(uart_num, 1, 3, 19, 18);
        ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size , uart_buffer_size, 10, &uart_queue, 0));

    // ALL OTHER NODE
    #else
        mesh_lite_config.join_mesh_without_configured_wifi = true;

        button_config_t gpio_btn_cfg_1 = initGpioButton_Short(GPIO_BUTTON_1);
        button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg_1);
        if(NULL == gpio_btn) {
            ESP_LOGE(TAG, "Button create failed");
        }
        iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb, (void *) BUTTON_SINGLE_CLICK);


    #endif

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 5, // The GPIO that connected to the LED strip's data line
        .max_leds = 1, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
    #else
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // whether to enable the DMA feature
    #endif
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &SOC_RGB));

    led_strip_clear(SOC_RGB);

    esp_mesh_lite_init(&mesh_lite_config);
    esp_mesh_lite_msg_action_list_register(node_event_action);
    esp_mesh_lite_msg_action_list_register(node_ping_action);

    app_wifi_set_softap_info();

    #if CONFIG_MESH_ROOT
        ESP_LOGI(TAG, "Root node");
        esp_mesh_lite_set_allowed_level(1);

    #else
        ESP_LOGI(TAG, "Child node");
    #endif

    esp_mesh_lite_start();

    #if CONFIG_MESH_ROOT
        ESP_LOGI(TAG, "Root node specific");
        
        // Only the root node will be receiving UART messages
        xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    #else

        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);

        //interrupt of rising edge
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        //change gpio interrupt type for one pin
        gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

        //create a queue to handle gpio event from isr
        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        //start gpio task
        xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

        //install gpio isr service
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

        //remove isr handler for gpio number.
        gpio_isr_handler_remove(GPIO_INPUT_IO_0);
        //hook isr handler for specific gpio pin again
        gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

        printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

        int cnt = 0;
        /* 
        onewire_bus_handle_t bus = NULL;
        onewire_bus_config_t bus_config = {
            .bus_gpio_num = GPIO_TEMP_1,
        };
        onewire_bus_rmt_config_t rmt_config_onewire = {
            .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
        };
        ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config_onewire, &bus));

        int ds18b20_device_num = 0;
        onewire_device_iter_handle_t iter = NULL;
        onewire_device_t next_onewire_device;
        esp_err_t search_result = ESP_OK;

        // create 1-wire device iterator, which is used for device search
        ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
        ESP_LOGI(TAG, "Device iterator created, start searching...");
        do {
            search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
            if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
                ds18b20_config_t ds_cfg = {};
                // check if the device is a DS18B20, if so, return the ds18b20 handle
                if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20_obj) == ESP_OK) {
                    ESP_LOGI(TAG, "Found a DS18B20, address: %016llX", next_onewire_device.address);
                } else {
                    ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
                }
            }
        } while (search_result != ESP_ERR_NOT_FOUND);
        ESP_ERROR_CHECK(onewire_del_device_iter(iter));
        ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

        float temperature; 
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20_obj));
        ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20_obj, &temperature));
        ESP_LOGI(TAG, "temperature read from DS18B20: %.2f C", temperature);
        
        ESP_LOGI(TAG, "setting up DS18B20 shield timer");
         */
        
        TimerHandle_t timer_tempsensor = xTimerCreate("tempsensor_example", 1000 / portTICK_PERIOD_MS, true, NULL, tempsensor_example);
        xTimerStart(timer_tempsensor, 0);
        
        
    #endif

    TimerHandle_t timer = xTimerCreate("print_system_info", 2000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
 
}