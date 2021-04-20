/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "include/ICM20648.h"

// Maximum number of element the application buffer can contain
#define MAXIMUM_BUFFER_SIZE 64

Thread IMU;
void imu_thread();

// the following should be active only on the tip IMU!!!
Thread tip_IMU;
void tip_imu_thread();

// rate-gyro data
float gyr_x, gyr_y, gyr_z;
Mutex gyro_mutex;

ICM20648 sensor(PC0, PC1, PC2, PC3, PF12);

static BufferedSerial serial_in (PK0, PK2);
static BufferedSerial serial_out(PF3, PF4);

int main() {

    // set UART desired properties (9600-8-N-1)
    serial_in.set_baud(9600);
    serial_in.set_format(8, BufferedSerial::None, 1);
    serial_out.set_baud(9600);
    serial_out.set_format(8, BufferedSerial::None, 1);

    // power IMU on
    DigitalOut imu_enable(PF8); // 0: not powered, 1: powered
    imu_enable = 1;

    // configure a LED for testing purposes!
    DigitalOut led(LED1);

    ThisThread::sleep_for(2s);

    if (sensor.open()) 
        printf("Device detected!\n");

    IMU.start(imu_thread);
    // comment the following line except on the tip IMU!
    //tip_IMU.start(tip_imu_thread);

    // application buffer to receive the data
    char buf[MAXIMUM_BUFFER_SIZE] = {0};

    while (true) {
        // the following is a blocking read function!
        if (uint32_t num = serial_in.read(buf, sizeof(buf))) {
            printf("got here with num = %d\n", num);
            led = !led;
            int *data_out = (int *)(buf+num);
            gyro_mutex.lock();
            data_out[0] = int(gyr_x);
            data_out[1] = int(gyr_y);
            data_out[2] = int(gyr_z);
            gyro_mutex.unlock();
            serial_out.write(buf, num+3*sizeof(int));
            // print all data at this node to screen!
            printf("MAIN GYROS: ");
            for (int i=0; i<num/sizeof(int); i+=sizeof(int)) {
                printf("%d ", *((int *)(buf+i)) );
            }
            printf("\n");
        }
        ThisThread::sleep_for(200ms);
        //printf("gyr: %d %d %d\n", int(gyr_x), int(gyr_y), int(gyr_z));
    }

}

void imu_thread() {
    while (true) {
        gyro_mutex.lock();
        sensor.measure();
        sensor.get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        gyro_mutex.unlock();
        ThisThread::sleep_for(1s);
    }
}

void tip_imu_thread() {
    // application buffer to send the data
    int data_out[3];
    while (true) {
        gyro_mutex.lock();
        data_out[0] = int(gyr_x);
        data_out[1] = int(gyr_y);
        data_out[2] = int(gyr_z);
        gyro_mutex.unlock();
        serial_out.write(data_out, sizeof(data_out));
        // print all data at this node to screen!
        printf("TIP GYROS: ");
        for (int i=0; i<3; i++) {
            printf("%d ", data_out[i] );
        }
        printf("\n");
        ThisThread::sleep_for(1s);
    }
}
