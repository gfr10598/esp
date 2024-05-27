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

#include "Arduino.h"
#include "Wire.h"
#include <NetworkClient.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// The sensor produces data at 1920*7*2*4/3 + overhead ~= 36kB/sec.
// Set up to buffer a comfortable 1.5 seconds of data.
QueueHandle_t data_queue = xQueueCreate(128, 512);

// Set these to your desired credentials.
const char *ssid = "TheBelfry";
const char *password = "GrandsireCaters";

IPAddress local_ip(192, 168, 5, 5);
NetworkServer server(local_ip, 80);

void init_i2c()
{
    // Initialize i2c.
    Wire.begin(3, 4, 1000000);
    printf("I2C initialized\n");
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        delay(100);
    }
}

void start_server()
{

    // You can remove the password parameter if you want the AP to be open.
    // a valid password must have more than 7 characters
    if (!WiFi.softAP(ssid, password))
    {
        printf("Soft AP creation failed.");
        log_e("Soft AP creation failed.");
        while (1)
            ;
    }
    IPAddress myIP = WiFi.softAPIP();
    printf("AP IP address: %s\n", myIP.toString().c_str());
    server.begin();

    printf("Server started\n");
}

int led = HIGH;

struct ServingParams
{
    NetworkClient client;
    QueueHandle_t queue;
};

// This will serve data from the queue to a client.
void vATaskHandleClient(void *params)
{
    // Copy the serving parameters.
    ServingParams *serving_params = (ServingParams *)params;
    NetworkClient client = serving_params->client;
    QueueHandle_t queue = serving_params->queue;

    unsigned long int start = millis();

    printf("New Client.\n"); // print a message out the serial port
    String currentLine = ""; // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
        if (client.available())
        {                           // if there's bytes to read from the client,
            char c = client.read(); // read a byte, then
            printf("%c", c);        // print it out the serial monitor
            if (c == '\n')
            { // if the byte is a newline character

                // if the current line is blank, you got two newline characters in a row.
                // that's the end of the client HTTP request, so send a response:
                if (currentLine.length() == 0)
                {
                    // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                    // and a content-type so the client knows what's coming, then a blank line:
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html");
                    client.println();

                    // the content of the HTTP response follows the header:
                    client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
                    client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");

                    // Write about 4kB for speed testing.
                    // This takes about 90 msec for 4k, so we can likely move about 40kB/sec, which is barely adequate.
                    // So, we probably need to use streaming or UDP.
                    for (int i = 0; i < 80; i++)
                    {
                        client.println("abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz");
                    }

                    // The HTTP response ends with another blank line:
                    client.println();
                    // break out of the while loop:
                    break;
                }
                else
                { // if you got a newline, then clear currentLine:
                    currentLine = "";
                }
            }
            else if (c != '\r')
            {                     // if you got anything else but a carriage return character,
                currentLine += c; // add it to the end of the currentLine
            }

            // Check to see if the client request was "GET /H" or "GET /L":
            if (currentLine.endsWith("GET /H"))
            {
                digitalWrite(13, HIGH); // GET /H turns the LED on
            }
            if (currentLine.endsWith("GET /L"))
            {
                digitalWrite(13, LOW); // GET /L turns the LED off
            }
        }
    }
    // close the connection:
    client.stop();
    printf("Client Disconnected.\n");
    printf("Client served in %ld ms\n", millis() - start);

    vTaskDelete(NULL);
}

void loop()
{
    NetworkClient client = server.accept(); // listen for incoming clients

    if (client)
    { // if you get a client,
        ServingParams params = {client, data_queue};
        TaskHandle_t pxCreatedTask = NULL;
        if (pdPASS != xTaskCreate(vATaskHandleClient,
                                  "ClientHandler",
                                  2048, // Stack size
                                  &params,
                                  5,
                                  &pxCreatedTask))
        {
            printf("Failed to create task\n");
        }
    }
}

const char *udpAddress = "192.168.4.255";
const int udpPort = 3333;

// The udp library class
NetworkUDP udp;

void blast()
{
    // This is a test function to see how fast we can send data.
    // It's not really useful for anything else.
    unsigned long int start = micros();
    // Roughly 1kB in 1.7 msec.  But, we have to delay about 10msec between packets,
    // so we can only get out about 1kB/11msec = 90kB/sec.  This is fine, and low latency.
    udp.beginPacket(udpAddress, udpPort);
    for (int i = 0; i < 10; i++)
    {
        udp.printf("abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz");
    }
    udp.endPacket();

    printf("%lu  Blasted a 1.0k in %lu usec\n", start / 1000, micros() - start);
}

extern "C" void app_main()
{
    initArduino();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    init_i2c();

    start_server();

    while (1)
    {
        // With delay of 9, we get occasional dropouts.
        vTaskDelay(10 / portTICK_PERIOD_MS);
        blast();
        loop();
    }
}
