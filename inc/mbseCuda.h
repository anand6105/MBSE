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
 * @file mbseCuda.h
 * @author Anand Prakash
 * @date 22 April 2020
 * @brief This header file is used for declaring all CUDA specific source files..
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#ifndef MBSECUDA_H_
#define MBSECUDA_H_

#include <stdio.h>
#include <stdlib.h>

#define minVal(a, b)        ((a > b) ? b : a)

typedef struct detectObject_t
{
    int bboxDeviceDetection;
    int bboxHostDetection;
    int imageHostDetection;
    int imageDeviceDetection;
} detectObject;

typedef struct sfmData_t
{
    int matrixSFMHost;
    int imageSFMHost;
} sfmData;

typedef struct plannerData_t
{
    int canData;
    int yawRate;
    int velocity;
    int xCar;
    int yawCar;
    int yCar;
    int matrixSFM;
    int bBoxHost;
    int occupancyGrid;
    int laneBoundary;
    int steerObjective;
    int speedObjective;
} plannerData;


/* CUDA kernel function call to detect the object */
void cuDetectObject(const char *function, detectObject *objdetected);

/* CUDA function kernel call to process the structure from motion data. */
void cuObjDetectSFM(const char *func, sfmData *sfmInputData);

/* Function to fetch the CAN Bus Polling Data. */
void cuPlannerFetchCanBusData(const char *func, int *hostCanPollingData);

/* Function to plan the path */
void cuPlannerInterpolatePath(const char *func, plannerData *data);

/* CUDA function to calculate the steer and speed based on the planner input */
void cuProcessDASM(const char *func, int *steer, int *speed);

/* Function to add OS overhead to the tasks */
void addOSOverhead(const char * func);

/* Function to get CUDA device properties */
void getCudaDeviceProperties(void);

#endif /* MBSECUDA_H_ */
