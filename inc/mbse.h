/*
 *
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * mbse.h
 *
 *  Created on: Apr 11, 2020
 *      Author: Anand Prakash
 */



#ifndef MBSE_H_
#define MBSE_H_

#define _GNU_SOURCE
#include <stdio.h>
#include <sys/types.h>
#include <sched.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MBSE_NUMBER_OF_THREADS       6
#define MBSE_THREAD_STACK_SIZE       (100 * 1024)      /* 100 kB is enough for now. */

#define MICRO_SECONDS                1000
#define MILLI_SECONDS                (MICRO_SECONDS * 1000)
#define SECONDS                      (MILLI_SECONDS * 1000)


typedef enum timerTask_t
{
    DETECTION_TIMER = 0,
    SFM_TIMER,
    PLANNER_TIMER,
    CAN_BUS_POLLING_TIMER,
    DASM_TIMER,
    OS_OVERHEAD_TIMER
}timerTask;

/* Function to print error messages */
void error(int at);

/* Function to initialize the timer value */
void utilInitializeTimer(timerTask timer);

/* Function to add delay */
void utilAddDelay(uint32_t ms, timerTask timer);

/* Function to set the priority of the thread */
void utilSetThreadPriority(pthread_t threadId, int prio);

/* This task snoops the key vehicle information (steer/wheel/break/acceleration status...)
 * from the on-board CAN bus and sends it to the Localization, Planner and EKF tasks.*/
void *pathPlannerCanBusPolling(void *args);

/* The main purpose of this component is to define and follow a given trajectory.
 * This trajectory is defined as a spline, that is, a line built through polynomial
 * interpolation at times on the map that represents at each point the position
 * and orientation that the car will have to follow. The planner sends the goal state
 *  of the vehicle (i.e., target steer and speed) to the DASM task that is in charge
 *  of writing the commands in the CAN line the effective steer and speed to apply.*/
void *pathPlannerCalculation(void *args);

/*
 *  This task is responsible of detecting and classifying the objects in the road.
 *  All the objects detected are visualized and the information produced is sent
 *  to the Planner task.
 **/
void *objDetectGetObject(void *args);

/* Function to read the Planner buffer */
int shmemReadPlannerBufferLabel(unsigned int index);

/* Function to read the lane boundary buffer */
int shmemLaneBoundaryBufferLabel(unsigned int index);

/* Function to read the occupancy grid data buffer */
int shmemReadGridDataBufferLabel(unsigned int index);

/* Function to write to the planner buffer */
void shmemWritePlannerDataOutLabel(int offset, int size, int data);

/* Function to write to the Detection data buffer */
void shmemWriteDetectionDataOutLabel(int offset, int size, void *data);

/* Function to write to the Structure-From-motion and detection data buffer */
int shmemReadSFMDetectionDataInLabel(unsigned int index);

/* Function to write to SFM output data buffer. */
void shmemWriteSFMDataOutLabel(int offset, int size, void *data);

/* Function to read SFM data */
int shmemReadSFMDataOutLabel(unsigned int index);

/* Function to read detection data */
int shmemReadDetectionDataOutLabel(unsigned int index);

/*
 *  Structure-From-Motion is a method for estimating 3-D structures (depth) from
 *  vehicle motion and sequences of 2-D images. This task returns a matrix of points
 *  representing the distance with respect the objects of the image.
 **/
void *objDetectStructureFromMotion(void *args);


/* This task computes and establishes the speed and steer that must be effectively
 * employed from the information that is provided by the Path Planner task*/
void *computeSpeedAndSteer(void *args);

/* This task is used to add the OS overhead to the overall tasks */
void *computeOSOverhead(void *args);

#endif /* MBSE_H_ */
