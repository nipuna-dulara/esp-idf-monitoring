#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"s
#include "driver/i2c.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ssd1306.h" // Library for DM0006 display (assuming it's SSD1306 based)

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define SHTC1_SENSOR_ADDR 0x70   /*!< SHTC1 I2C address */
#define SHTC1_CMD_MEASURE 0x7CA2 /*!< Measurement command for SHTC1 */

#define TEMP_THRESHOLD_HIGH 30.0 /*!< High temperature threshold */
#define TEMP_THRESHOLD_LOW 20.0  /*!< Low temperature threshold */
#define HUM_THRESHOLD_HIGH 60.0  /*!< High humidity threshold */
#define HUM_THRESHOLD_LOW 30.0   /*!< Low humidity threshold */

#define MODE_BUTTON_GPIO 34
#define OK_BUTTON_GPIO 35
#define UP_BUTTON_GPIO 32
#define DOWN_BUTTON_GPIO 33

#define WIFI_SSID "Dialog_4G_905"
#define WIFI_PASS "AmeeraRox123"

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

typedef enum
{
    MAIN_SCREEN,
    MENU_SCREEN,
    SET_TEMP_THRESHOLD_HIGH,
    SET_TEMP_THRESHOLD_LOW,
    SET_HUM_THRESHOLD_HIGH,
    SET_HUM_THRESHOLD_LOW
} screen_t;

screen_t current_screen = MAIN_SCREEN;
float temp_threshold_high = TEMP_THRESHOLD_HIGH;
float temp_threshold_low = TEMP_THRESHOLD_LOW;
float hum_threshold_high = HUM_THRESHOLD_HIGH;
float hum_threshold_low = HUM_THRESHOLD_LOW;

static const char *TAG = "SHTC1";
static const char *MQTT_TAG = "MQTT";

static esp_mqtt_client_handle_t mqtt_client;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
}

void update_display(ssd1306_t *dev)
{
    ssd1306_clear_screen(dev, false);
    char buffer[32];

    switch (current_screen)
    {
    case MAIN_SCREEN:
        snprintf(buffer, sizeof(buffer), "Temp: %.2f C", temp_threshold_high);
        ssd1306_draw_string(dev, 0, 0, buffer, 16, false);
        snprintf(buffer, sizeof(buffer), "Hum: %.2f %%", hum_threshold_high);
        ssd1306_draw_string(dev, 0, 16, buffer, 16, false);
        break;
    case MENU_SCREEN:
        ssd1306_draw_string(dev, 0, 0, "1. Set Temp High", 16, false);
        ssd1306_draw_string(dev, 0, 16, "2. Set Temp Low", 16, false);
        ssd1306_draw_string(dev, 0, 32, "3. Set Hum High", 16, false);
        ssd1306_draw_string(dev, 0, 48, "4. Set Hum Low", 16, false);
        break;
    case SET_TEMP_THRESHOLD_HIGH:
        snprintf(buffer, sizeof(buffer), "Temp High: %.2f", temp_threshold_high);
        ssd1306_draw_string(dev, 0, 0, buffer, 16, false);
        break;
    case SET_TEMP_THRESHOLD_LOW:
        snprintf(buffer, sizeof(buffer), "Temp Low: %.2f", temp_threshold_low);
        ssd1306_draw_string(dev, 0, 0, buffer, 16, false);
        break;
    case SET_HUM_THRESHOLD_HIGH:
        snprintf(buffer, sizeof(buffer), "Hum High: %.2f", hum_threshold_high);
        ssd1306_draw_string(dev, 0, 0, buffer, 16, false);
        break;
    case SET_HUM_THRESHOLD_LOW:
        snprintf(buffer, sizeof(buffer), "Hum Low: %.2f", hum_threshold_low);
        ssd1306_draw_string(dev, 0, 0, buffer, 16, false);
        break;
    }

    ssd1306_refresh_gram(dev);
}

void handle_buttons(ssd1306_t *dev)
{
    if (gpio_get_level(MODE_BUTTON_GPIO) == 1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS); // debounce
        if (gpio_get_level(MODE_BUTTON_GPIO) == 1)
        {
            current_screen = (current_screen + 1) % 6;
            update_display(dev);
        }
    }
    else if (gpio_get_level(OK_BUTTON_GPIO) == 1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS); // debounce
        if (gpio_get_level(OK_BUTTON_GPIO) == 1)
        {
            // Logic for OK button
            if (current_screen != MAIN_SCREEN)
            {
                current_screen = MAIN_SCREEN;
                update_display(dev);
            }
        }
    }
    else if (gpio_get_level(UP_BUTTON_GPIO) == 1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS); // debounce
        if (gpio_get_level(UP_BUTTON_GPIO) == 1)
        {
            // Logic for Up button
            switch (current_screen)
            {
            case SET_TEMP_THRESHOLD_HIGH:
                temp_threshold_high += 0.5;
                break;
            case SET_TEMP_THRESHOLD_LOW:
                temp_threshold_low += 0.5;
                break;
            case SET_HUM_THRESHOLD_HIGH:
                hum_threshold_high += 1.0;
                break;
            case SET_HUM_THRESHOLD_LOW:
                hum_threshold_low += 1.0;
                break;
            default:
                break;
            }
            update_display(dev);
        }
    }
    else if (gpio_get_level(DOWN_BUTTON_GPIO) == 1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS); // debounce
        if (gpio_get_level(DOWN_BUTTON_GPIO) == 1)
        {
            // Logic for Down button
            switch (current_screen)
            {
            case SET_TEMP_THRESHOLD_HIGH:
                temp_threshold_high -= 0.5;
                break;
            case SET_TEMP_THRESHOLD_LOW:
                temp_threshold_low -= 0.5;
                break;
            case SET_HUM_THRESHOLD_HIGH:
                hum_threshold_high -= 1.0;
                break;
            case SET_HUM_THRESHOLD_LOW:
                hum_threshold_low -= 1.0;
                break;
            default:
                break;
            }
            update_display(dev);
        }
    }
}

static void button_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MODE_BUTTON_GPIO) | (1ULL << OK_BUTTON_GPIO) | (1ULL << UP_BUTTON_GPIO) | (1ULL << DOWN_BUTTON_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t shtc1_read(uint16_t *temperature, uint16_t *humidity)
{
    uint8_t data[6];
    esp_err_t ret;

    // Send measurement command
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SHTC1_SENSOR_ADDR, (uint8_t *)&SHTC1_CMD_MEASURE, sizeof(SHTC1_CMD_MEASURE), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Wait for the sensor to complete the measurement
    vTaskDelay(20 / portTICK_RATE_MS);

    // Read the measurement data
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, SHTC1_SENSOR_ADDR, data, sizeof(data), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Convert the data
    *temperature = (data[0] << 8) | data[1];
    *humidity = (data[3] << 8) | data[4];

    return ESP_OK;
}

/**
 * @brief MQTT event handler
 */
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://mqtt_server_uri", // Replace with your MQTT server URI
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi
    wifi_init();

    // Wait for Wi-Fi connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    else
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    }
    ESP_ERROR_CHECK(i2c_master_init());
    button_init();
    mqtt_app_start();

    ssd1306_t dev;
    dev._address = 0x3C;
    dev._flip = true;
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);

    uint16_t temperature, humidity;
    while (1)
    {
        handle_buttons(&dev);

        if (current_screen == MAIN_SCREEN)
        {
            esp_err_t ret = shtc1_read(&temperature, &humidity);
            if (ret == ESP_OK)
            {

                float temp = -45 + 175 * ((float)temperature / 65535.0);
                float hum = 100 * ((float)humidity / 65535.0);

                char buffer[32];
                snprintf(buffer, sizeof(buffer), "Temp: %.2f C", temp);
                ssd1306_draw_string(&dev, 0, 0, buffer, 16, false);
                snprintf(buffer, sizeof(buffer), "Hum: %.2f %%", hum);
                ssd1306_draw_string(&dev, 0, 16, buffer, 16, false);

                bool warning = false;
                if (temp > temp_threshold_high || temp < temp_threshold_low)
                {
                    ssd1306_draw_string(&dev, 0, 32, "Temp Out of Range!", 16, true);
                    warning = true;
                }
                else
                {
                    ssd1306_draw_string(&dev, 0, 32, "                     ", 16, false);
                }
                if (hum > hum_threshold_high || hum < hum_threshold_low)
                {
                    ssd1306_draw_string(&dev, 0, 48, "Hum Out of Range!", 16, true);
                    warning = true;
                }
                else
                {
                    ssd1306_draw_string(&dev, 0, 48, "                   ", 16, false);
                }

                if (warning)
                {
                    char mqtt_msg[128];
                    snprintf(mqtt_msg, sizeof(mqtt_msg), "Warning: Temp=%.2fC, Hum=%.2f%%", temp, hum);
                    esp_mqtt_client_publish(mqtt_client, "/sensor/warning", mqtt_msg, 0, 1, 0);
                }

                ssd1306_refresh_gram(&dev);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read data from SHTC1 sensor");
            }
        }

        vTaskDelay(200 / portTICK_RATE_MS); // Delay for 200 ms
    }
}