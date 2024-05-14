#include "Arduino.h"
#include <stdio.h>
#include "LSM6DSV16XSensor.h"
#include <string>
#include "base64_encode.hpp"
// #include "driver/gpio.h"

#define FIFO_SAMPLE_THRESHOLD 30
#define FLASH_BUFF_LEN 8192
// We can just barely keep up, with PERF, at 1MHz i2c.
#define SENSOR_ODR 1900.0f

uint16_t loop(LSM6DSV16XSensor &lsm);
uint16_t Read_FIFO_Data(LSM6DSV16XSensor &lsm);
uint16_t read7(LSM6DSV16XSensor &lsm);

extern "C" void app_main()
{
    initArduino();
    // gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);  // This also works with driver/gpio.h
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    // Initlialize i2c.
    Wire.begin(3, 4, 1000000);
    printf("I2C initialized\n");
    LSM6DSV16XSensor LSM(&Wire);
    printf("LSM created\n");

    if (LSM6DSV16X_OK != LSM.begin())
    {
        printf("LSM.begin() Error\n");
    }
    int32_t status = 0;

    status |= LSM.Enable_G();
    status |= LSM.Enable_X();

    status |= LSM.Set_G_FS(1000); // Need minimum of 600 dps
    status |= LSM.Set_X_FS(16);   // To handle large impulses from clapper.
    status |= LSM.Set_X_ODR(SENSOR_ODR);
    status |= LSM.Set_G_ODR(SENSOR_ODR);

    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);

    // Configure FIFO BDR for acc and gyro
    status |= LSM.FIFO_Set_X_BDR(SENSOR_ODR);
    status |= LSM.FIFO_Set_G_BDR(SENSOR_ODR);

    // Set FIFO in Continuous mode
    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to configure FIFO\n");
        while (1)
            ;
    }
    printf("LSM enabled\n");

    int led = HIGH;

    int n = 2 * 1920;
    uint64_t gap_start = esp_timer_get_time();
    while (1)
    {
        uint16_t records;
        if (LSM.FIFO_Get_Num_Samples(&records) != LSM6DSV16X_OK)
        {
            printf("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
            while (1)
                ;
        }

        if (records < FIFO_SAMPLE_THRESHOLD)
        {
            continue;
        }

        // When records >= FIFO_SAMPLE_THRESHOLD, we can read the FIFO data
        int64_t gap_end = esp_timer_get_time();
        printf("Gap was %lld\n", gap_end - gap_start);

        uint16_t actual = read7(LSM);
        gap_start = esp_timer_get_time();
        n -= actual;
        if (n > 0)
        {
            continue;
        }
        led = !led;
        digitalWrite(13, led);
        printf("%6lld  1920 samples ****************************************\n", esp_timer_get_time() / 1000);
        n += 2 * 1920;
    }
}

bool acc_available = false;
bool gyr_available = false;

struct Tag
{
    unsigned int _unused : 1;
    unsigned int cnt : 2;
    unsigned int tag : 5;
};
struct TagData
{
    Tag tag;
    int16_t data[3];
};

uint encode(const uint8_t *input, uint input_length, uint8_t *output)
{
    return encode_base64(input, input_length, output);
}

// Read 7 bytes of data from FIFO until it is empty.
uint16_t read7(LSM6DSV16XSensor &lsm)
{
    TagData one_record;
    int16_t acc_gyr[6] = {0};

    int acc = 0;
    int gyr = 0;

    uint16_t status = lsm.FIFO_Get_Tag_And_Data((uint8_t *)&one_record);
    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to get tag and data\n");
        while (1)
            ;
    }

    uint64_t start = esp_timer_get_time();
    uint8_t last = 0;

    while (one_record.tag.tag != 0)
    {
        last = one_record.tag.tag;
        switch (last)
        {
        case 1:
            memcpy(&acc_gyr[0], one_record.data, 6);
            gyr++;
            break;
        case 2:
            memcpy(&acc_gyr[3], one_record.data, 6);
            uint8_t output[12];
            encode_base64((uint8_t *)acc_gyr, 12, output);
            // printf("%s\n", output);
            if (acc % 4 == 0)
            {
                // printf("gyr: %6d %6d %6d   acc: %6d %6d %6d\n", acc_gyr[0], acc_gyr[1], acc_gyr[2], acc_gyr[3], acc_gyr[4], acc_gyr[5]);
            }
            acc++;
            break;
        default:
            break;
        }
        status = lsm.FIFO_Get_Tag_And_Data((uint8_t *)&one_record);
        if (status != LSM6DSV16X_OK)
        {
            printf("LSM6DSV16X Sensor failed to get tag and data\n");
            while (1)
                ;
        }
    }
    uint64_t end = esp_timer_get_time();
    printf("Time taken: %lld for %d + %d (%d)  ", end - start, acc, gyr, last);
    return acc + gyr;
}

uint16_t Read_FIFO_Data(LSM6DSV16XSensor &lsm)
{
    uint16_t i;
    uint16_t samples_to_read;
    int32_t acc_value[3];
    int32_t gyr_value[3];
    int count = 0;

    // Check the number of samples inside FIFO
    if (lsm.FIFO_Get_Num_Samples(&samples_to_read) != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
        while (1)
            ;
    }

    if (samples_to_read > FIFO_SAMPLE_THRESHOLD)
    {
        printf("Reading %d samples\n", samples_to_read);
    }
    else
    {
        return 0;
    }

    // This loop takes about 110msec for 256 samples.  However, almost all of them are tag=1.
    // If we increase the i2c clock to 1MHz, we can get it down to about 63msec for 256 records, which
    // would work out to about 250 usec/record, or 2k samples/sec.  NOTE that the cpu is blocked while
    // it reads the i2c bus.

    // This is also much too slow.  We need to read 8k x 2 samples per second, which means 60 usec/sample.
    // That would mean 15msec per 256 samples - about 8 times faster.
    int64_t start = esp_timer_get_time();
    // Each record has to move about 15 bytes, and should take about 2.5usec * 8 * 15 = 300 usec at 400kHz.
    // ***** This means our max throughput would be about 3k records per second, or about 1500 sample/sec. *****
    // But they currently take about 610 usec (DEBUG) or about 570usec (PERF).
    for (i = 0; i < samples_to_read; i++)
    {
        // int64_t start = esp_timer_get_time();

        uint8_t tag;

        // Check the FIFO tag - this requires moving about 5 bytes of data.
        // It takes about 160 usec, which is a bit slower than expected.
        // This takes 2x longer if we also time and print the gyro latency!!!
        if (lsm.FIFO_Get_Tag(&tag) != LSM6DSV16X_OK)
        {
            printf("LSM6DSV16X Sensor failed to get tag");
            while (1)
                ;
        }
        switch (tag)
        {
        // If we have a gyro tag, read the gyro data
        case 1:
        {
            // In principle, these should move 6 bytes of data.  There is a write to the device,
            // then a read from the device.  Altogether, this should move about 80 bits of data,
            // and at 400kHz, this should take about 200 usec, which would be great!!
            // int64_t gyr_start = esp_timer_get_time();
            if (lsm.FIFO_Get_G_Axes(gyr_value) != LSM6DSV16X_OK)
            {
                printf("LSM6DSV16X Sensor failed to get gyroscope data");
                while (1)
                    ;
            }
            gyr_available = true;
            break;
        }

        // If we have an acc tag, read the acc data
        case 2:
        {
            if (lsm.FIFO_Get_X_Axes(acc_value) != LSM6DSV16X_OK)
            {
                printf("LSM6DSV16X Sensor failed to get accelerometer data");
                while (1)
                    ;
            }
            acc_available = true;
            break;
        }

        // We can discard other tags
        default:
        {
            break;
        }
        }
        // If we have the measurements of both acc and gyro, we can store them with timestamp
        if (acc_available && gyr_available)
        {
            count++;
            acc_available = false;
            gyr_available = false;
        }
        // printf("Time taken for %d: %lld\n", tag, esp_timer_get_time() - start);
    }

    int64_t end = esp_timer_get_time();
    printf("Time taken: %lld\n", end - start);

    // printf("Found %d readings\n", count);
    return count;
}
