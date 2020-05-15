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
 * @file mbse.h
 * @author Anand Prakash
 * @date 22 April 2020
 * @brief This header file used for all C specific source files.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
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
#include <errno.h>

#define MBSE_NUMBER_OF_THREADS       6
#define MBSE_THREAD_STACK_SIZE       (100 * 1024)      /* 100 kB is enough for now. */

#define NSEC_PER_SEC                 (1000 * 1000 * 1000)
#define MICRO_SECONDS                1000
#define MILLI_SECONDS                (MICRO_SECONDS * 1000)
#define SECONDS                      (MILLI_SECONDS * 1000)


/* Function to print error messages */
void error(int at);

/* Function to add delay */
void utilAddDelay(uint32_t ms, struct timespec *deadline);

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

/* Function to read the Planner buffer */
int shmemReadPlannerBufferLabel(unsigned int index);

/* Function to read the lane boundary buffer */
int shmemReadLaneBoundaryBufferLabel(unsigned int index);

/* Function to read the occupancy grid data buffer */
int shmemReadGridDataBufferLabel(unsigned int index);

/* Function to write to the planner buffer */
void shmemWritePlannerDataOutLabel(int offset, int size, int data);

/* Function to write to SFM and detection buffer */
void shmemWriteSFMDetectionDataInLabel(int offset, int size, int data);

/* Function to write to data grid buffer */
void shmemWriteGridDataBufferLabel(int offset, int size, int data);

/* Function to write to data boundary buffer */
void shmemWriteLaneBoundaryBufferLabel(int offset, int size, int data);

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


#endif /* MBSE_H_ */
