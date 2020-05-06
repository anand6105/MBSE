/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * sharedMemory.c
 *
 *  Created on: May 5, 2020
 *      Author: Anand Prakash
 */


#include "mbse.h"


/* Input data buffer for SFM and Detection process thread. Buffer size is 2MB of type int */
#define DATA_IN_SFM_DETECTION_BUFFER      ((2 * 1024 * 1024) / 4)
/* Output data buffer for SFM process. Buffer size is 24KB of type int */
#define DATA_OUT_SFM_BUFFER               ((24 * 1024) / 4)
/* Output data buffer for Detection process. Buffer size is 750KB of type int */
#define DATA_OUT_DETECTION_BUFFER         ((750 * 1024) / 4)

/* Buffer to store the detection and SFM Data */
static int cInDetectionSFMDataBuffer[DATA_IN_SFM_DETECTION_BUFFER];
static int cOutDetectionBuffer[DATA_OUT_DETECTION_BUFFER];
static int cOutSFMBuffer[DATA_OUT_SFM_BUFFER];



int shmemReadSFMDetectionDataInLabel(unsigned int index)
{
    if (index < DATA_IN_SFM_DETECTION_BUFFER)
    {
        return cInDetectionSFMDataBuffer[index];
    }
    return -1;
}


int shmemReadSFMDataOutLabel(unsigned int index)
{
    if (index < DATA_OUT_SFM_BUFFER)
    {
        return cOutSFMBuffer[index];
    }
    return -1;
}

int shmemReadDetectionDataOutLabel(unsigned int index)
{
    if (index < DATA_OUT_DETECTION_BUFFER)
    {
        return cOutDetectionBuffer[index];
    }
    return -1;
}

void shmemWriteDetectionDataOutLabel(int offset, int size, void *data)
{
    if (offset < DATA_OUT_DETECTION_BUFFER)
    {
        if (0 == memcpy(&cOutDetectionBuffer[offset], data, size))
        {
            fprintf(stdout,"Error in copying data to detection out buffer\n");
        }
        fflush(stdout);
    }
}


void shmemWriteSFMDataOutLabel(int offset, int size, void *data)
{
    if (offset < DATA_OUT_SFM_BUFFER)
    {
        if (0 == memcpy(&cOutSFMBuffer[offset], data, size))
        {
            fprintf(stdout,"Error in copying data to SFM out buffer\n");
        }
        fflush(stdout);
    }
}
