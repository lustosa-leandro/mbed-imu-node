/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "include/ICM20648.h"

void imu_thread();

float gyr_x, gyr_y, gyr_z;

ICM20648 sensor(PC0, PC1, PC2, PC3, PF12);

Mutex stdio_mutex;

Thread IMU;

int main() {

    // power IMU on
    DigitalOut imu_enable(PF8); // 0: not powered, 1: powered
    imu_enable = 1;

    // configure a LED for testing purposes!
    DigitalOut led(LED1);

    ThisThread::sleep_for(2s);

    if (sensor.open()) 
        printf("Device detected!\n");

    IMU.start(imu_thread);

    while (true) {
        ThisThread::sleep_for(1s);
        led = !led;
        stdio_mutex.lock();
        printf("main!\n");
        stdio_mutex.unlock();
    }

}

void imu_thread() {
    while (true) {
        sensor.measure();
        sensor.get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        ThisThread::sleep_for(1s);
        stdio_mutex.lock();
        printf("gyr: %d  %d  %d\n", int(gyr_x), int(gyr_y), int(gyr_z));
        stdio_mutex.unlock();
    }
}
