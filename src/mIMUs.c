
#include <stdlib.h>
#include <string.h>
#include "include/mIMUs.h"

// local routines used to parse the IMU messages
static void StoreSTATUS(mtIMU_STATUS *p, unsigned char *buf);
static void StoreCHANNEL_SOURCE(mtIMU_CHANNEL_SOURCE *p, unsigned char *buf);
static void StorePULSE_OUTPUTS(mtIMU_PULSE_OUTPUTS *p, unsigned char *buf);
static void StorePULSE_INPUTS(mtIMU_PULSE_INPUTS *p, unsigned char *buf);
static void StoreDATA_SO_FAR(mtIMU_DATA_SO_FAR *p, unsigned char *buf);

//===================================================================
//                       mIMUSetup
//===================================================================
// mIMUSetup stores the queue handles that are to be used for talking
// to the receiver.  Also, it creates a mBin packetizer on the recieve queue.
mtIMUState *mIMUSetup(mQueueRH hr) {

    mtIMUState *p;

    // Create a new state variable.
    p = (mtIMUState *)malloc(sizeof(mtIMUState));
    if(!p)
        return(0);

    // zero the data in the new state structure
    memset((void *)p, 0, sizeof(mtIMUState));

    // Fill in the relevant data.
    p->hrIMU = hr;
    p->Pktzr=mBinCreatePacketizer(p->hrIMU, 256);
    if(!(p->Pktzr)) {
        free(p);
        return(0);
    }
    return(p);
}

//===================================================================
//                       mIMUParse
//===================================================================
// mIMUParse parses all the data in the IMU receive queue.  When a
// message is found, the data is stored and the message numer is returned.
// When the queue is empty, this function returns zero.  Call this function
// until it returns zero to keep the queue empty.
unsigned char mIMUParse(mtIMUState *p) {

    unsigned char *buf, ID;
    static unsigned char ck0, ck1;

    // get a packet is one is available
    buf=mBinGetPacket(p->Pktzr, &ck0, &ck1);

    // if no packet is available, return zero.
    if(!buf)
        return(0);

	// The buffer contents start with:
	//   ID (uc)
	//   Payload Length (uc, bytes)
	//   Payload (all elements are big-endian)
	// Lets call the appropriate storage function  based on the message ID.
	// We will not handle the length here, but will provide a pointer
    // to the length to each storage function.
    p->Message = (char*) buf;
    ID = *buf++;

    switch(ID) {
        case 10: StoreSTATUS(&(p->STATUS), buf);                  break;
        case 11: StoreCHANNEL_SOURCE(&(p->CHANNEL_SOURCE), buf);  break;
        case 12: StorePULSE_OUTPUTS(&(p->PULSE_OUTPUTS), buf);    break;
        case 13: StorePULSE_INPUTS(&(p->PULSE_INPUTS), buf);      break;
        case 16: StoreDATA_SO_FAR(&(p->DATA_SO_FAR), buf);        break;
    }

    // return the ID/Class word
    return(ID);

}

//===================================================================
//                       StoreSTATUS
//===================================================================
// Store STATUS message in the supplied structure mIMU_STATUS
static void StoreSTATUS(mtIMU_STATUS *p, unsigned char *buf) {

    // skip the length as the STATUS message is always 8 bytes.
    ++buf;

    p->status=mBinGetUShort(&buf);

    // Mark message as having been updated.
    p->updated=1;

}

//===================================================================
//                       StoreCHANNEL_SOURCE
//===================================================================
// Store CHANNEL_SOURCE message in the supplied structure mtIMU_CHANNEL_SOURCE
static void StoreCHANNEL_SOURCE(mtIMU_CHANNEL_SOURCE *p, unsigned char *buf) {

    int i;

    unsigned char len = *buf;
    ++buf;

    for(i=0;i<len;++i)
        p->ch[i]=*buf++;

    // Mark message as having been updated.
    p->updated=1;
}

//===================================================================
//                       StorePULSE_OUTPUTS
//===================================================================
// Store PULSE_OUTPUTS message in the supplied structure mtIMU_PULSE_OUTPUTS
static void StorePULSE_OUTPUTS(mtIMU_PULSE_OUTPUTS *p, unsigned char *buf) {

    int i;

    unsigned char len = *buf++;

    for(i=0;i<len/2;++i)
        p->ch[i]=mBinGetUShort(&buf);

    // Mark message as having been updated.
    p->updated=1;
}

//===================================================================
//                       StorePULSE_INPUTS
//===================================================================
// Store PULSE_INPUTS message in the supplied structure mtIMU_PULSE_INPUTS
static void StorePULSE_INPUTS(mtIMU_PULSE_INPUTS *p, unsigned char *buf) {

    int i;

    unsigned char len = *buf++;

    for(i=0;i<len/2;++i)
        p->ch[i]=mBinGetUShort(&buf);

    // Mark message as having been updated.
    p->updated=1;
}

//===================================================================
//                       StoreDATA_SO_FAR
//===================================================================
// Store DATA_SO_FAR message in the supplied structure mIMU_DATA_SO_FAR
static void StoreDATA_SO_FAR(mtIMU_DATA_SO_FAR *p, unsigned char *buf) {

    // skip the length as the STATUS message is always 8 bytes.
    unsigned char len = *buf++;

    p->number_IMUs_so_far = mBinGetShort(&buf);

    for (int i=0; i<p->number_IMUs_so_far; i++) {
       p->rate_gyro_P[i] = mBinGetShort(&buf);
       p->rate_gyro_Q[i] = mBinGetShort(&buf);
       p->rate_gyro_R[i] = mBinGetShort(&buf);
    }

    // Mark message as having been updated.
    p->updated=1;

}
