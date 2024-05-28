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
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <string.h>
#include <lwip/sockets.h>
#include <esp32-hal.h>

// Set these to your desired credentials.
const uint8_t ssid[] = "TheBelfry";
const uint8_t password[] = "GrandsireCaters";

static const char *SOFT_AP_TAG = "wifi softAP";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(SOFT_AP_TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(SOFT_AP_TAG, "station " MACSTR " leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

void init_wifi()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "TheBelfry",
            .password = "GrandsireCaters",
            .ssid_len = sizeof("TheBelfry") - 1,
            .channel = 5,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .max_connection = 4,
            .pmf_cfg = {
                .required = true,
            },
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#endif
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("WiFi started\n");
}

int socket()
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        return sock;
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    return sock;
}

/**
 * Sends a message via UDP
 * @param id - target IP address. Could be a 1-1 (192.168.1.2), could be a broadcast 192.168.1.255
 * @param port - target port. 0-65000
 * @param message - message to be sent. Any format. Could be plain text, could be JSON string "{\"text\":\"awesome message\"}"
 */
ssize_t udp_client_send_message(int sock, char *ip, int port, char *message)
{
    // Set up parameters
    char rx_buffer[128];

    char host_ip[strlen(ip) + 1];
    strcpy(host_ip, ip);

    sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    // Send message
    return sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
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
long unsigned blast(int sock, char *msg)
{
    unsigned long start = micros();
    // Send a UDP packet to the broadcast address.
    ssize_t sent = udp_client_send_message(sock, "192.168.4.255", 3333, msg);
    if (sent < 0)
    {
        return sent;
    }
    return micros() - start;
}

char msg[2049];

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

    esp_log_level_set("*", ESP_LOG_WARN);

    for (int i = 0; i < 1024; i++)
    {
        msg[i] = 'A' + i % 64;
    }
    msg[2048] = 0;

    int sock = socket();
    if (sock < 0)
    {
        ESP_LOGE(SOFT_AP_TAG, "Unable to create socket: errno %d", errno);
        while (1)
            ;
    }

    unsigned long last = 0;
    for (int cnt = 0; cnt < 100000; cnt++)
    {
        // Reusing the socket, and sending 2048 byte messages, we can send 90 messages/sec.
        // With a higher message rate, we start getting send errors.
        vTaskDelay(11 / portTICK_PERIOD_MS);
        ssize_t blast_time = blast(sock, msg);
        if (blast_time < 0)
        {
            if (errno == ENOMEM)
                ESP_LOGE(SOFT_AP_TAG, "ENOMEM");
            else
                ESP_LOGE(SOFT_AP_TAG, "Error occurred during sending: errno %d", errno);
        }

        if (cnt % 100 == 0)
        {
            set_led(cnt % 200);
            unsigned long now = micros();
            unsigned long avg = (now - last) / 100;
            unsigned long throughput = 2000000 / avg; // kB/sec
            last = now;
            timeval datetime;
            gettimeofday(&datetime, NULL);
            printf("blast at %lld took %d usecs, average elapsed: %lu throughput: %lu kB/sec\n", datetime.tv_sec, blast_time, avg, throughput);
        }
    }
    // Close socket
    if (sock != -1)
    {
        shutdown(sock, 0);
        close(sock);
    }
    while (1)
        ;
}
