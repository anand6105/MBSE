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

#define MBSE_NUMBER_OF_THREADS       4
#define MBSE_THREAD_STACK_SIZE       (100 * 1024)      /* 100 kB is enough for now. */

#define MICRO_SECONDS                1000
#define MILLI_SECONDS                (MICRO_SECONDS * 1000)
#define SECONDS                      (MILLI_SECONDS * 1000)

/* Function to print error messages */
void error(int at);

/* Function to add delay */
void addDelay(uint32_t delay);

/* Function to set the priority of the thread */
void setThreadPriority(pthread_t threadId, int prio);


void *vCanBusPolling(void *args);

void *vPlanner(void *args);

/*
 *  This task is responsible of detecting and classifying the objects in the road.
 *  All the objects detected are visualized and the information produced is sent
 *  to the Planner task.
 **/
void *vDetection(void *args);

/*
 *  Structure-From-Motion is a method for estimating 3-D structures (depth) from
 *  vehicle motion and sequences of 2-D images. This task returns a matrix of points
 *  representing the distance with respect the objects of the image.
 **/
void *vSFM(void *args);


#endif /* MBSE_H_ */
