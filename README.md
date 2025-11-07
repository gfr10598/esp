# esp
Esp-idf related code

# TODO
* Get minimal arduino dependency set.
* Get this working without hacks to arduino code.

# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.


### Hardware
The original hardware used for this is:
- [ESP32-S2 Feather w.FL](https://www.adafruit.com/product/6282) Or
- [ESP32-S2 Feather integrated antenna](https://www.adafruit.com/product/5000)
- [SparkFun Micro 6DoF IMU Breakout - LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-micro-6dof-imu-breakout-lsm6dsv16x-qwiic.html)
<img width="429" height="197" alt="image" src="https://github.com/user-attachments/assets/b838bbd0-dc3c-4988-b955-e6c54586b235" />
<img width="262" height="110" alt="image" src="https://github.com/user-attachments/assets/20841a2e-97d8-45c2-a77d-a050b36962be" />

I am currently working on a more capable hardware configuration that supports local storage and better power management.  
The main change is adding a microSD card, by switching to a Sparkfun Think Plus.

- [SparkFun Thing Plus - ESP32-S3](https://www.sparkfun.com/sparkfun-thing-plus-esp32-s3.html)
- [SparkFun Micro 6DoF IMU Breakout - LSM6DSV16X (Qwiic)](https://www.sparkfun.com/sparkfun-micro-6dof-imu-breakout-lsm6dsv16x-qwiic.html) - same as above
- [Flexible Qwiic Cable - 100mm](https://www.sparkfun.com/flexible-qwiic-cable-100mm.html)
- [Lithium Ion Battery - 2Ah](https://www.sparkfun.com/lithium-ion-battery-2ah.html)

The Thing Plus board has an integrated antenna, which limits the WiFi performance, but the intent is to separate data
collection during ringing from data download when the bell is stationary.

I may instead pursue using the [Adafruit Metro](https://www.adafruit.com/product/5500) since I had the best success
with the Adafruit product the first time around last year.  It is a larger board (2.1" x 2.7"), but can be stacked with the battery,
so there is no real penalty for the larger size.  They also have a compatible [2500 mAh battery](https://www.adafruit.com/product/328),
which is a bit more cost effective than the SparkFun, with dimensions 1.9" x 2.3" x 0.29" - just a little smaller than the Metro board.

In addition, the components have to be firmly attached to the headstock, either with magnets or with foam tape.
- [Rectangular Magnets 45 lbs Force 1.57" x 0.53" x 0.20"](https://www.amazon.com/dp/B0BKY6QYTT)
- [Double Sided Tape Heavy Duty Foam Tape, 3M VHB Tape](https://www.amazon.com/dp/B0DS8DGJ19)
If magnets are used, the same foam tape can be used to attach the components to the magnets, except for the sensor,
which should be glued to the magnet (or headstock), to minimize acoustic noise coupling from the air. (It may be that
the tape is perfectly adequate, but this needs to be tested).
A small amount of butyl rubber between the magnet and headstock will eliminate any rocking caused by non-flat surfaces.

A tape such as [this](https://www.amazon.com/dp/B0DXL6FV9P) applied over the sensor may further help reduce acoustic noise, but this has
not been tested.


