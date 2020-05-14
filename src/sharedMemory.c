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
/* Output data buffer for planner process. Buffer size is 1 KB of type int */
#define DATA_OUT_PLANNER_BUFFER           ((1 * 1024) / 4)
/* Output data buffer for planner process. Buffer size is 1 KB of type int */
#define DATA_GRID_BUFFER                  ((1 * 512) / 4)
/* Output data buffer for planner process. Buffer size is 1 KB of type int */
#define DATA_LANE_BOUNDARY_BUFFER           (256 / 4)

/* Buffer to store the detection and SFM Data */
static int cInDetectionSFMDataBuffer[DATA_IN_SFM_DETECTION_BUFFER];
static int cOutDetectionBuffer[DATA_OUT_DETECTION_BUFFER];
static int cOutSFMBuffer[DATA_OUT_SFM_BUFFER];
static int plannerBuffer[DATA_OUT_PLANNER_BUFFER];
static int gridDataBuffer[DATA_GRID_BUFFER];
static int laneBoundaryBuffer[DATA_LANE_BOUNDARY_BUFFER];


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


int shmemReadPlannerBufferLabel(unsigned int index)
{
    if (index < DATA_OUT_PLANNER_BUFFER)
    {
        return plannerBuffer[index];
    }
    return -1;
}


int shmemReadGridDataBufferLabel(unsigned int index)
{
    if (index < DATA_GRID_BUFFER)
    {
        return gridDataBuffer[index];
    }
    return -1;
}


int shmemLaneBoundaryBufferLabel(unsigned int index)
{
    if (index < DATA_LANE_BOUNDARY_BUFFER)
    {
        return laneBoundaryBuffer[index];
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


void shmemWritePlannerDataOutLabel(int offset, int size, int data)
{
    if (offset < DATA_OUT_PLANNER_BUFFER)
    {
        if (0 == memcpy(&plannerBuffer[offset], &data, size))
        {
            fprintf(stdout,"Error in copying data to planner out buffer\n");
        }
        fflush(stdout);
    }
}
