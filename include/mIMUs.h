
#ifndef _mIMU_Header_
    #define _mIMU_Header_

#include "include/mBin.h"

#ifdef __cplusplus
extern "C" {
#endif

//=====================================================================
// IMU message - Status information
//   STATUS, ID 10, 2 bytes
//=====================================================================
typedef struct {

    unsigned char updated;

    unsigned short status;    // System status
    //  15-7:   reserved
    //   2-1:   command channel state
    //          0 = signal not present
    //          1 = 1ms
    //          2 = 1.5ms
    //          3 = 2ms
    //     0:   config not available (not locked out)
} mtIMU_STATUS;

//=====================================================================
// IMU message - Channel source
//    CHANNEL_SOURCE, ID 11, N bytes (max 24 outputs?)
//=====================================================================
typedef struct {

    unsigned char updated;

    unsigned char ch[24];

} mtIMU_CHANNEL_SOURCE;

//=====================================================================
// IMU message - Pulse outputs
//   PULSE_OUTPUTS, ID 12, 2*N bytes (max 24 outputs?)
//=====================================================================
typedef struct {

    unsigned char updated;

    unsigned short ch[24];

} mtIMU_PULSE_OUTPUTS;

//=====================================================================
// IMU message - Pulse inputs
//   PULSE_INPUTS, ID 13, 2*M bytes (max 16 inputs?)
//=====================================================================
typedef struct {

    unsigned char updated;

    unsigned short ch[16];

} mtIMU_PULSE_INPUTS;

//=====================================================================
// IMU message - Auxiliary inputs message
//   AUX_INPUTS, ID 14, not implemented yet - place holder
//=====================================================================
typedef struct {

    unsigned char updated;

} mtIMU_AUX_INPUTS;

//=====================================================================
// IMU message - System configuration message
//   SYS_CONFIG, ID 15, not implemented yet - place holder
//=====================================================================
typedef struct {

    unsigned char updated;

} mtIMU_SYS_CONFIG;

//=====================================================================
// IMU message - Pulse command message
//   PULSE_COMM, ID 20, not implemented yet - place holder
//=====================================================================
typedef struct {

    unsigned char updated;

} mtIMU_PULSE_COMM;

//=====================================================================
// IMU message - Auxiliary outputs
//   AUX_OUTPUTS, ID 21, not implemented yet - place holder
//=====================================================================
typedef struct {

    unsigned char updated;

} mtIMU_AUX_OUTPUTS;

//=====================================================================
// IMU message - Lockout now
//   LOCKOUT_NOW, ID 98, not implemented yet - place holder
//=====================================================================
typedef struct {

    unsigned char updated;

} mtIMU_LOCKOUT_NOW;

//=====================================================================
// IMU message - IMU data
//   DATA_SO_FAR, ID 16, the only one that matters!
//=====================================================================
typedef struct {

    unsigned char updated;

    signed short number_IMUs_so_far;

    signed short rate_gyro_P[5];
    signed short rate_gyro_Q[5];
    signed short rate_gyro_R[5];

} mtIMU_DATA_SO_FAR;


//=====================================================================
//=====================================================================
// In order to support multiple parses in use by the same program, a
// structure is created when the parser is instantiated that maintains the
// state information for the parser.  This structure holds the parsed
// messages, the message packetizer, and the receive queue handle.
typedef struct {

    // parsed message contents
    mtIMU_STATUS           STATUS;
    mtIMU_CHANNEL_SOURCE   CHANNEL_SOURCE;
    mtIMU_PULSE_OUTPUTS    PULSE_OUTPUTS;
    mtIMU_PULSE_INPUTS     PULSE_INPUTS;
    mtIMU_DATA_SO_FAR      DATA_SO_FAR;

    // other stuff
    mQueueRH hrIMU;
    mBinPacketizer Pktzr;

    // For configuration and ACK/NACK, we will want the parser to return the
    // ID, but also have a pointer available for getting the data.  'Message'
    // points to the received packet.  Actually, it points to the ID of the
    // packet.  This is the pointer that is returned by mBinGetPacket.  See
    // mBin.h for details.
    char *Message;
} mtIMUState;


//=====================================================================
//=====================================================================
// mIMUSetup initializes the IMU packetizing/parsing subsystem.
// It returns a pointer to a mtIMUState structure on success and
// zero on fail.
extern mtIMUState *mIMUSetup(mQueueRH hrIMU);

// mIMUParse extracts bytes from the receive queue (specified in the call to
// mIMUSetup) looking for valid IMU packets.  When a valid packet is
// found, it is stored in the myIMUState structure and its ID is returned
// to the user, regardless of whether the queue is empty. (i.e., there may
// be additional packets in the queue after this one).  When the queue is
// empty, this function returns zero.  Call this function until it returns
// zero to keep the queue empty.
extern unsigned char mIMUParse(mtIMUState *);

#ifdef __cplusplus
}
#endif
#endif
