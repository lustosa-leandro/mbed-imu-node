/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "include/ICM20648.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     1000ms

void process_imu_data();

//InterruptIn int_imu(PF12);
InterruptIn int_button(BUTTON1);

volatile int test;

int main() {

    // power IMU on
    DigitalOut imu_enable(PF8); // 0: not powered, 1: powered
    imu_enable = 1;

    // configure interrupts for button and IMU to same handler
//    int_imu.rise(&process_imu_data);
    int_button.rise(&process_imu_data);

    // configure a LED for testing purposes!
    DigitalOut led(LED1);

    ICM20648 sensor(PC0, PC1, PC2, PC3, PF12);

    test = 0;

    printf("open comms!\n");
    ThisThread::sleep_for(10000ms);

    if (sensor.open())
        printf("Device detected!\n");

    float gyr_x, gyr_y, gyr_z;

    while (true) {
        ThisThread::sleep_for(BLINKING_RATE);
        led = !led;
        printf("led toggle!! \n");
        sensor.measure();
        sensor.get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        printf("gyr: %d  %d  %d\n", int(gyr_x), int(gyr_y), int(gyr_z));
        if (test) {
            printf("got an interrupt! \n");
            test = 0;
        }
    }

}

void process_imu_data() {
    test = 1;
}


