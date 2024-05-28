/*
We can send 1kB UDP packets in about 1.7 msec per packet.
Thus, we can get very very low latency, and only need about 7% of the CPU,
and perhaps 10% of the sustained network throughput.

If we also listen for UDP packets from the clients, we can retransmit
packets that are lost.  The clients can send notices when they are missing
packets, using packet sequence numbers.  The server can keep a buffer of the
last 20 packets or so, and retransmit them when requested.  Probably it
should just back up and resume transmitting from the earliest missing packet.

Clients might well miss the same packets, so this might not cost much.
One question is how long a client should wait before requesting a missing packet.
Are UDP packets delivered over WiFi in order?

*/

#include <stdio.h>
#include <string>

#include "driver/gpio.h"

#include "esp_wifi.h"

#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// The sensor produces data at 1920*7*2*4/3 + overhead ~= 36kB/sec.
// Set up to buffer a comfortable 1.5 seconds of data.
QueueHandle_t data_queue = xQueueCreate(128, 512);

// Set these to your desired credentials.
const uint8_t ssid[] = "TheBelfry";
const uint8_t password[] = "GrandsireCaters";

void init_wifi()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = *ssid,
            .password = *password,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
void loop()
{
}

const char *udpAddress = "192.168.4.255";
const int udpPort = 3333;

void output_pin(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ull << pin,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
}

void enable_quic()
{
    output_pin(13);
    gpio_set_level(GPIO_NUM_13, 1);
}

void set_led(int on)
{
    gpio_set_level(GPIO_NUM_13, on);
}

extern "C" void app_main()
{
    // initArduino();
    printf("Hello, world!\n");

    enable_quic();
    output_pin(21);
    gpio_set_level(GPIO_NUM_21, 1); // neopixel power
    output_pin(33);
    gpio_set_level(GPIO_NUM_33, 1); // neopixel data

    set_led(1);

    ESP_ERROR_CHECK(nvs_flash_init());
    init_wifi();

    for (int cnt = 0; cnt < 10000; cnt++)
    {
        // With delay of 9, we get occasional dropouts.
        vTaskDelay(100 / portTICK_PERIOD_MS);
        set_led(cnt % 2);
        printf("%d\n", cnt);
    }
    while (1)
        ;
}
