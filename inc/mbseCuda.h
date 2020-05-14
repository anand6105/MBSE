/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * mbseCuda.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Anand Prakash
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
