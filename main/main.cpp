#include "Arduino.h"
#include <stdio.h>
#include "LSM6DSV16XSensor.h"

extern "C" void app_main()
{
    initArduino();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // Initlialize i2c.
    Wire.begin();
    LSM6DSV16XSensor LSM6DSV16X(&Wire);

    LSM6DSV16X.begin();
    LSM6DSV16X.Enable_X();

    int led = HIGH;

    while (1)
    {
        led = !led;
        digitalWrite(13, led);
        delay(1000);
        printf("loop\n");
    }
}
