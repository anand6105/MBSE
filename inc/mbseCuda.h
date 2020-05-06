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

/* CUDA kernel function call to detect the object */
void cuDetectObject(const char *function, detectObject *objdetected);

/* CUDA function kernel call to process the structure from motion data. */
void cuObjDetectSFM(const char *func, sfmData *sfmInputData);

void getCudaDeviceProperties(void);

#endif /* MBSECUDA_H_ */
