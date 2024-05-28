/*
We can send UDP packets at about 100 pkts / sec.  More than that, and we see ENOMEM errors
when there are clients attached.

If we collect 40-45 records per batch, that works out to 10-12 msec, which works fine.
If we want extra bandwidth for something else, we can reduce the data packet rate.

At 100 pkts/sec, the packets are around 400 bytes + a count field.

We probably need a retransimit window and protocol, so we may want to back off to 50 pkts/sec
and 90 records / packet, and allow retransmission up to perhaps 50 packets in the past.
*/

#include <optional>
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

#include "base64_encode.hpp"
#include "LSM6DSV16XSensor.h"

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
        ESP_LOGW(SOFT_AP_TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGW(SOFT_AP_TAG, "station " MACSTR " leave, AID=%d, reason=%d",
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
    int tries = 0;
    while (sent < 0)
    {
        vTaskDelay(5);
        sent = udp_client_send_message(sock, "192.168.4.255", 3333, msg);
        tries++;
    }
    if (tries > 3)
    {
        printf("Tries: %d\n", tries);
    }
    return micros() - start;
}

#define FIFO_SAMPLE_THRESHOLD 20
#define FLASH_BUFF_LEN 8192
// If we send the raw data to the other processor for output to WiFi,
// we can likely keep up with 3840 samples/sec.
#define SENSOR_ODR 1920

LSM6DSV16XSensor init_lsm()
{
    // Initialize i2c.
    Wire.begin(3, 4, 1000000);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    printf("I2C initialized\n");
    LSM6DSV16XSensor LSM(&Wire);
    printf("LSM created\n");

    if (LSM6DSV16X_OK != LSM.begin())
    {
        printf("LSM.begin() Error\n");
        while (1)
            ;
    }
    int32_t status = 0;

    // This doesn't seem to do anything - not seeing any compressed data.
    status |= LSM.FIFO_Set_Compression_Algo(LSM6DSV16X_CMP_16_TO_1);
    status |= LSM.Set_G_FS(1000); // Need minimum of 600 dps
    status |= LSM.Set_X_FS(16);   // To handle large impulses from clapper.
    status |= LSM.Set_X_ODR(SENSOR_ODR);
    status |= LSM.Set_G_ODR(SENSOR_ODR);
    status |= LSM.Set_Temp_ODR(LSM6DSV16X_TEMP_BATCHED_AT_1Hz875);

    // Set FIFO to timestamp data at 20 Hz
    status |= LSM.FIFO_Enable_Timestamp();
    status |= LSM.FIFO_Set_Timestamp_Decimation(LSM6DSV16X_TMSTMP_DEC_32);
    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);

    // Configure FIFO BDR for acc and gyro
    status |= LSM.FIFO_Set_X_BDR(SENSOR_ODR);
    status |= LSM.FIFO_Set_G_BDR(SENSOR_ODR);

    // Set FIFO in Continuous mode
    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

    status |= LSM.Enable_G();
    status |= LSM.Enable_X();

    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to configure FIFO\n");
        while (1)
            ;
    }
    else
        printf("LSM enabled\n");
    return LSM;
}

struct Tag
{
    unsigned int _unused : 1;
    unsigned int cnt : 2;
    unsigned int tag : 5;
};

int read_many(LSM6DSV16XSensor &LSM, uint16_t avail, void *records)
{
    uint16_t actual;
    if (LSM6DSV16X_OK != LSM.Read_FIFO_Data(avail, records, &actual))
    {
        printf("LSM6DSV16X Sensor failed to read FIFO data\n");
        while (1)
            ;
    }
    return actual;
}

std::string encode(void *records, int actual)
{
    char output[actual * 10 + 4];
    encode_base64((unsigned char *)records, actual * 7, (unsigned char *)output);
    return std::string(output); // Allocates on heap.
}

int wait_for(LSM6DSV16XSensor &lsm, int n)
{
    // Wait until there are FIFO_SAMPLE_THRESHOLD records in the FIFO.
    uint16_t avail = 0;
    for (int status = lsm.FIFO_Get_Num_Samples(&avail);
         (status == LSM6DSV16X_OK) && (avail < n);
         status = lsm.FIFO_Get_Num_Samples(&avail))
    {
        vTaskDelay(1); // Wait the minimum interval.
    }
    return avail;
}

char msg[2049];
char records[64 * 7];
int records_offset = 0;

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

    LSM6DSV16XSensor lsm = init_lsm();
    unsigned char records[32 * 7];

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
    int bytes_sent = 0;
    int msg_sent = 0;
    int blast_time = 0;
    while (1)
    {
        // This call should make many vTaskDelay(1) calls.
        int avail = wait_for(lsm, 20);
        if (avail > 50)
        {
            ESP_LOGW(SOFT_AP_TAG, "actual: %d", avail);
        }
        int actual = read_many(lsm, avail, (void *)(records + records_offset * 7));
        records_offset += actual;
        if (records_offset > 32)
        {
            std::string encoded = encode(records, records_offset);
            char msg[1024];
            sprintf(msg, "%d %s", msg_sent % 100, encoded.c_str());
            blast_time = blast(sock, msg);
            bytes_sent += encoded.size();
            msg_sent++;
            unsigned long now = micros();
            if (msg_sent % 10 == 0)
            {
                set_led((msg_sent / 100) % 2);
                printf("sent %d bytes for %d records over interval of %lu usecs\n", encoded.size(), records_offset, now - last);
            }
            records_offset = 0;
            last = now;
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
