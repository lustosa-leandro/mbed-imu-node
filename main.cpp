/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "PinNames.h"
#include "mbed.h"

#include "include/ICM20648.h"
#include "include/mIMUs.h"

Thread IMU_thread;
void imu_periodic_callback();

// the following should be active only on the tip IMU!!!
Thread tip_IMU_thread;
void tip_imu_periodic_callback();

// rate-gyro data structures
float gyr_x, gyr_y, gyr_z;
Mutex gyro_mutex;

ICM20648 imu_device(PC0, PC1, PC2, PC3, PF12);

// serial comms structures
static BufferedSerial serial_in (PK0, PK2); // for incoming data from tip sensors
static BufferedSerial serial_out(PF3, PF4); // for outgoing data to root sensors
mQueueRH hrIMUrx;
mQueueWH hwIMUrx;
mtIMUState *IMUs_;

// (default) number of rate-gyros towards the tip (excluded this one!)
int number_of_in_gyros_  = 0;
// (default) number of rate-gyros towards the tip (included this one!)
int number_of_out_gyros_ = 1;

// private functions used by this module only
void output_imus_data();

DigitalIn button(BUTTON1);

int main() {

    // set UART desired properties (115200-8-N-1)
    serial_in.set_baud(115200);
    serial_in.set_format(8, BufferedSerial::None, 1);
    serial_out.set_baud(115200);
    serial_out.set_format(8, BufferedSerial::None, 1);

    // init UART state machine for handling incoming data parser
    if (mQueueCreate(&hrIMUrx, &hwIMUrx, 128)!=0) {
        printf("ERROR!");
        return -1;
    }
    IMUs_ = mIMUSetup(hrIMUrx);

    // power IMU device on
    DigitalOut imu_enable(PF8); // 0: not powered, 1: powered
    imu_enable = 1;

    // configure a LED for indicating UART RX data purposes!
    DigitalOut led(LED1);

    // sleep a little before attempting to open IMU device!
    ThisThread::sleep_for(2s);
    if (imu_device.open()) 
        printf("Device detected!\n");

    // this thread periodically updates the angular velocity for this module
    IMU_thread.start(imu_periodic_callback);

    // the BT0 button is pressed during initialization to indicate tip sensor!
    // - this sensor will be responsable for initiating daisy chain comms
    if (!button.read()) tip_IMU_thread.start(tip_imu_periodic_callback);

    while (true) {

        // transfer bytes from peripherical buffer to RAM
        unsigned char buf;
        serial_in.read(&buf,1);
        mQueueWriteChar(hwIMUrx,buf);

        // parse messages from queue
        unsigned char ID = 0;
        while ( (ID = mIMUParse(IMUs_)) != 0 ) { // Process all data stored in queue
            led = !led;
            if (ID == 16) {
                // read data and print on screen
                number_of_in_gyros_ = IMUs_->DATA_SO_FAR.number_IMUs_so_far;
                number_of_out_gyros_ = number_of_in_gyros_ + 1;
                for (int i=0; i<number_of_in_gyros_; i++) {
                    printf("GYRO %d: %d %d %d\n",
                        i,
                        IMUs_->DATA_SO_FAR.rate_gyro_P[i],
                        IMUs_->DATA_SO_FAR.rate_gyro_Q[i],
                        IMUs_->DATA_SO_FAR.rate_gyro_R[i]);
                }
                gyro_mutex.lock();
                printf("GYRO %d: %d %d %d\n", number_of_in_gyros_, int(gyr_x), int(gyr_y), int(gyr_z));
                gyro_mutex.unlock();
                output_imus_data();
            }
        }
    }

}

void imu_periodic_callback() {
    while (true) {
        gyro_mutex.lock();
        imu_device.measure();
        imu_device.get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
        gyro_mutex.unlock();
        ThisThread::sleep_for(1s);
    }
}

void tip_imu_periodic_callback() {
    while (true) {
        output_imus_data();
        // print all data at this node to screen!
        gyro_mutex.lock();
        printf("GYRO %d: %d %d %d\n", number_of_in_gyros_, int(gyr_x), int(gyr_y), int(gyr_z));
        gyro_mutex.unlock();
        ThisThread::sleep_for(1s);
    }
}

void output_imus_data() {

    // maximum buffer size for this message is 4(sync,id,count)+2*23(U2 payload)+2(cksum)
    signed short payload[3*number_of_out_gyros_+1];
    unsigned char buffer[4+sizeof(payload)+2], *p_end;
    unsigned short length;
    // create header with sync numbers, msg ID=16 and count=0
    p_end = mBinPacketOpen(buffer, 16); // 16 is msg ID
    // prepare payload
    payload[0] = number_of_out_gyros_;
    int j=1;
    for (int i=0; i<number_of_in_gyros_; i++) {
        payload[j] = IMUs_->DATA_SO_FAR.rate_gyro_P[i]; j++;
        payload[j] = IMUs_->DATA_SO_FAR.rate_gyro_Q[i]; j++;
        payload[j] = IMUs_->DATA_SO_FAR.rate_gyro_R[i]; j++;
    }
    gyro_mutex.lock();
    payload[j] = int(gyr_x); j++;
    payload[j] = int(gyr_y); j++;
    payload[j] = int(gyr_z); j++;
    gyro_mutex.unlock();
    // populate packet with payload
    for (int i=0; i<sizeof(payload)/sizeof(signed short); i++) {
        p_end = mBinPutShort(p_end, payload[i]);
    }
    // write the checksums and close packet
    length = (unsigned char) mBinPacketClose(buffer, p_end);
    // write to serial port if port is alright
    serial_out.write(buffer, length);

}
