/************************************************************************************
 * Copyright (C) 2020 Dortmund University of Applied Sciences and Arts and Others.  *
 *                                                                                  *
 * This file is part of the implementation of 2019 Waters Challenge                 *
 *                                                                                  *
 * The processing power offered by GPUs and their capability to execute parallel    *
 * workloads is exploited to execute and accelerate applications related to         *
 * advanced driver assistance systems.                                              *
 *                                                                                  *
 * The challenge consists in analytically master the complex HW/SW system (that     *
 * will be available as Amalthea model) to answer the following questions:          *
 *                                                                                  *
 * Response Time Computation:                                                       *
 *  a) Given an application consisting of a set of dependent tasks and a given      *
 *     mapping, calculate its end-to-end response time.                             *
 *  b) The response time should account for the time for the copy engine to         *
 *     transfer data between the CPU and the GPU.                                   *
 *  c) The response time should account for the time for the data transfers between *
 *     the CPU and the shared main memory considering a read/execute/write semantic.*
 *  d) Optionally the off-loading mechanism (synchronous/asynchronous) can be       *
 *     changed to further optimize the end-to-end latencies.                        *
 ************************************************************************************/


/**
 * @file sharedMemory.c
 * @author Anand Prakash
 * @date 05 May 2020
 * @brief This file declares and implement the read and write operation of all the shared
 *        memory.
 *
 * It consists a common input buffer for Detection and Structure for Motion task which is
 * 2MB in size. Output data buffer for Structure for motion and Detection task with size
 * 24KB and 750KB respectively. The planner, data grid and lane boundary buffers are
 * 1KB, 512KB and 256 bytes in size respectively.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
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
/* Output data buffer for planner process. Buffer size is 512 KB of type int */
#define DATA_GRID_BUFFER                  ((1024 * 512) / 4)
/* Output data buffer for planner process. Buffer size is 256 bytes of type int */
#define DATA_LANE_BOUNDARY_BUFFER           (256 / 4)

/* Buffer to store the detection and SFM Input Data */
static int cInDetectionSFMDataBuffer[DATA_IN_SFM_DETECTION_BUFFER];
/* Buffer to store the detection Data */
static int cOutDetectionBuffer[DATA_OUT_DETECTION_BUFFER];
/* Buffer to store the SFM Data */
static int cOutSFMBuffer[DATA_OUT_SFM_BUFFER];
/* Buffer to store the planner Data */
static int plannerBuffer[DATA_OUT_PLANNER_BUFFER];
/* Buffer to store the grid Data */
static int gridDataBuffer[DATA_GRID_BUFFER];
/* Buffer to store the lane boundary  Data */
static int laneBoundaryBuffer[DATA_LANE_BOUNDARY_BUFFER];


/**
 * @brief Function to read to the Detection and SFM Input data buffer
 *
 * This function read to the Detection and SFM Input data buffer. This buffer is used as an input
 * to SFM and Detection task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadSFMDetectionDataInLabel(unsigned int index)
{
    if (index < DATA_IN_SFM_DETECTION_BUFFER)
    {
        return cInDetectionSFMDataBuffer[index];
    }
    return -1;
}



/**
 * @brief Function to read to the SFM Output data buffer
 *
 * This function read the SFM output data buffer. This buffer is used as an input to Planner task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadSFMDataOutLabel(unsigned int index)
{
    if (index < DATA_OUT_SFM_BUFFER)
    {
        return cOutSFMBuffer[index];
    }
    return -1;
}


/**
 * @brief Function to read to the Detection Output data buffer
 *
 * This function read the Detection output data buffer. This buffer is used as an input to Planner task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadDetectionDataOutLabel(unsigned int index)
{
    if (index < DATA_OUT_DETECTION_BUFFER)
    {
        return cOutDetectionBuffer[index];
    }
    return -1;
}

/**
 * @brief Function to read to the Planner Output data buffer
 *
 * This function read the Planner output data buffer. This buffer is used as an input to DASM task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadPlannerBufferLabel(unsigned int index)
{
    if (index < DATA_OUT_PLANNER_BUFFER)
    {
        return plannerBuffer[index];
    }
    return -1;
}

/**
 * @brief Function to read to the data grid buffer
 *
 * This function read the data grid buffer. This buffer is used as an input to Planner task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadGridDataBufferLabel(unsigned int index)
{
    if (index < DATA_GRID_BUFFER)
    {
        return gridDataBuffer[index];
    }
    return -1;
}


/**
 * @brief Function to read to the lane boundary data buffer
 *
 * This function read the lane boundary data buffer. This buffer is used as an input to Planner task.
 *
 * @param[in] index  Index at which the data is to be written.
 *
 * @return       The value at the requested index, -1 in case the index is out of bounds.
 */
int shmemReadLaneBoundaryBufferLabel(unsigned int index)
{
    if (index < DATA_LANE_BOUNDARY_BUFFER)
    {
        return laneBoundaryBuffer[index];
    }
    return -1;
}


/**
 * @brief Function to write to the Detection data buffer
 *
 * This function writes to the Detection data buffer. This buffer is used as an input
 * to Planner task.
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
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


/**
 * @brief Function to write to the SFM data buffer
 *
 * This function writes to the SFM data buffer. This buffer is used as an input
 * to Planner task.
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
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


/**
 * @brief Function to write to the Planner data buffer
 *
 * This function writes to the Planner data buffer. This buffer is used as an input
 * to DASM task.
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
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


/**
 * @brief Function to write to the SFM and detection Input buffer
 *
 * This function writes to the SFM and detection Input buffer. This buffer is used as an input
 * to SFM and detection task.
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
void shmemWriteSFMDetectionDataInLabel(int offset, int size, int data)
{
    if (offset < DATA_IN_SFM_DETECTION_BUFFER)
    {
        if (0 == memcpy(&cInDetectionSFMDataBuffer[offset], &data, size))
        {
            fprintf(stdout,"Error in copying data to planner out buffer\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief Function to write to the data grid buffer
 *
 * This function writes to the data grid buffer. This buffer is used as an input
 * to Planner task
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
void shmemWriteGridDataBufferLabel(int offset, int size, int data)
{
    if (offset < DATA_GRID_BUFFER)
    {
        if (0 == memcpy(&gridDataBuffer[offset], &data, size))
        {
            fprintf(stdout,"Error in copying data to planner out buffer\n");
        }
        fflush(stdout);
    }
}


/**
 * @brief Function to write to the lane boundary buffer
 *
 * This function writes to the lane boundary buffer. This buffer is used as an input
 * to Planner task
 *
 * @param[in] offset   Offset of the buffer.
 * @param[in] size     Length of data.
 * @param[in] data     Actual data that needs to be copied.
 *
 * @return void
 */
void shmemWriteLaneBoundaryBufferLabel(int offset, int size, int data)
{
    if (offset < DATA_LANE_BOUNDARY_BUFFER)
    {
        if (0 == memcpy(&laneBoundaryBuffer[offset], &data, size))
        {
            fprintf(stdout,"Error in copying data to planner out buffer\n");
        }
        fflush(stdout);
    }
}
