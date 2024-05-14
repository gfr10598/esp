#include "Arduino.h"
#include <stdio.h>
#include "LSM6DSV16XSensor.h"
#include <string>
#include "base64_encode.hpp"

#define FIFO_SAMPLE_THRESHOLD 200
#define FLASH_BUFF_LEN 8192
// We can't quite keep up with 1920, with PERF and 1MHz i2c.
// Actually, we can just barely keep up, if we don't print out the data.
// If we send the raw data to the other processor for output to WiFi,
// we can likely keep up with 1920 samples/sec.
#define SENSOR_ODR 1920

uint16_t Read_FIFO_Data(LSM6DSV16XSensor &lsm);
uint16_t read_until_empty(LSM6DSV16XSensor &lsm);
uint16_t read_until_empty_7(LSM6DSV16XSensor &lsm);

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

    int n = 2 * SENSOR_ODR;
    uint64_t gap_start = esp_timer_get_time();
    while (1)
    {
        // Wait until there are FIFO_SAMPLE_THRESHOLD records in the FIFO.
        uint16_t records = 0;
        for (status = LSM.FIFO_Get_Num_Samples(&records);
             (status == LSM6DSV16X_OK) && (records < FIFO_SAMPLE_THRESHOLD);
             status = LSM.FIFO_Get_Num_Samples(&records))
        {
        }
        int64_t gap_end = esp_timer_get_time();
        // uint16_t actual = Read_FIFO_Data(LSM);
        uint16_t actual = read_until_empty(LSM);
        if (actual == 0)
        {
            continue;
        }
        printf("Gap was %lld\n", gap_end - gap_start);

        gap_start = esp_timer_get_time();
        n -= actual;
        if (n > 0)
        {
            continue;
        }
        led = !led;
        digitalWrite(13, led);
        printf("%6lld  1920 samples ****************************************\n", esp_timer_get_time() / 1000);
        n += 2 * SENSOR_ODR;
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

int read_record(LSM6DSV16XSensor &lsm, int16_t *output)
{
    lsm6dsv16x_axis3bit16_t data_raw;
    if (lsm.FIFO_Get_Data(data_raw.u8bit) != LSM6DSV16X_OK)
    {
        return LSM6DSV16X_ERROR;
    }
    output[0] = data_raw.i16bit[0];
    output[1] = data_raw.i16bit[1];
    output[2] = data_raw.i16bit[2];
    return LSM6DSV16X_OK;
}
// Read 7 bytes of data from FIFO until it is empty.
uint16_t read_until_empty_7(LSM6DSV16XSensor &lsm)
{
    int16_t gyr_acc[6] = {0};

    int acc = 0;
    int gyr = 0;

    TagData tag_data = {0, 0, 0, 0};
    int status;
    uint8_t last = 0;
    uint64_t start = esp_timer_get_time();
    // It seems that reading the Tag as part of a i2c block read does not behave properly.
    for (status = lsm.FIFO_Get_Tag_And_Data((uint8_t *)&tag_data);
         (status == LSM6DSV16X_OK) && (tag_data.tag.tag > 0);
         lsm.FIFO_Get_Tag_And_Data((uint8_t *)&tag_data))
    {
        switch (tag_data.tag.tag)
        {
        case 1:
            gyr_acc[0] = tag_data.data[0];
            gyr_acc[1] = tag_data.data[1];
            gyr_acc[2] = tag_data.data[2];
            gyr++;
            last = 1;
            break;
        case 2:
            gyr_acc[3] = tag_data.data[0];
            gyr_acc[4] = tag_data.data[1];
            gyr_acc[5] = tag_data.data[2];
            acc++;
            last = 2;
            uint8_t output[16];
            // Somehow, this is corrupting the low order byte of gyr_acc[0]!!!
            // encode_base64((unsigned char *)gyr_acc, 12, output);
            if (acc % 10 == 0)
            {
                printf("FIFO (%2d)  gyr: %6d %6d %6d   acc: %6d %6d %6d  ", acc, gyr_acc[0], gyr_acc[1], gyr_acc[2], gyr_acc[3], gyr_acc[4], gyr_acc[5]);
                printf("\n");
                // printf("  %s\n", output);
            }
            break;
        default:
            break;
        }
    }
    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to get tag and data\n");
        while (1)
            ;
    }

    uint64_t end = esp_timer_get_time();

    printf("Time taken: %lld for %d + %d (%d)  ", end - start, acc, gyr, last);
    return acc + gyr;
}

// Read tag + 6 bytes of data from FIFO until it is empty.
uint16_t read_until_empty(LSM6DSV16XSensor &lsm)
{
    int16_t gyr_acc[6] = {0};

    int acc = 0;
    int gyr = 0;

    uint8_t tag = 0;
    int status;
    uint8_t last = 0;
    uint64_t start = esp_timer_get_time();
    // It seems that reading the Tag as part of a i2c block read does not behave properly.
    for (status = lsm.FIFO_Get_Tag(&tag); (status == LSM6DSV16X_OK) && (tag > 0); lsm.FIFO_Get_Tag(&tag))
    {
        switch (tag)
        {
        case 1:
            read_record(lsm, &gyr_acc[0]);
            gyr++;
            last = 1;
            break;
        case 2:
            read_record(lsm, &gyr_acc[3]);
            acc++;
            last = 2;
            uint8_t output[16];
            // Somehow, this is corrupting the low order byte of gyr_acc[0]!!!
            encode_base64((unsigned char *)gyr_acc, 12, output);
            if (acc % 100 == 0)
            {
                // printf("FIFO (%2d)  gyr: %6d %6d %6d   acc: %6d %6d %6d  ", acc, gyr_acc[0], gyr_acc[1], gyr_acc[2], gyr_acc[3], gyr_acc[4], gyr_acc[5]);
                //  printf("\n");
                printf("%s\n", output);
            }
            break;
        default:
            break;
        }
    }
    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to get tag and data\n");
        while (1)
            ;
    }

    uint64_t end = esp_timer_get_time();

    printf("Time taken: %lld for %d + %d (%d)  ", end - start, acc, gyr, last);
    return acc + gyr;
}

uint16_t Read_FIFO_Data(LSM6DSV16XSensor &lsm)
{
    uint16_t i;
    uint16_t samples_to_read;
    int32_t acc_value[3] = {0};
    int32_t gyr_value[3] = {0};
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
    int acc_count = 0;
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
            acc_count++;
            if (acc_count % 10 == 0)
                printf("FIFO   gyr: %6ld %6ld %6ld   acc: %6ld %6ld %6ld\n", gyr_value[0], gyr_value[1], gyr_value[2], acc_value[0], acc_value[1], acc_value[2]);
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
