/*
 ************************************************************************************
 *   Copyright (c) 2020 Dortmund University of Applied Sciences and Arts and others.
 *
 *   Contributors:
 *        Dortmund University of Applied Sciences and Arts -
 *        initial API and implementation
 ************************************************************************************
 * cuObjDetection.cu
 *
 *  Created on: Apr 17, 2020
 *      Author: Anand Prakash
 */

extern "C"{
    #include "mbseCuda.h"
}


#define NBIN  10000000  // Number of bins
#define NUM_BLOCK   13  // Number of thread blocks
#define NUM_THREAD 192  // Number of threads per block
/**
 * CUDA Kernel Device code
 *
 * Runnable to Process the image to detect and classify the objects by creating the Boundary Box.
 */
__global__ void
processImage(int *hostBbox, int *devBbox, int *hostImage, int *devImage, int numElements)
{
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    int sampleSize = 1024;

    if (index < numElements)
    {
        hostBbox[index] = sampleSize + 1;
        devBbox[index] = sampleSize + 1;
        hostImage[index] = sampleSize + 1;
        devImage[index] = sampleSize + 1;
    }
}

/* Runnable Host to device call to copy input data from host memory to device memory */
extern "C"
static void cudaCopyHostToDevice(int *bboxHost, int *bboxDevice, int *imageHost, int *imageDevice,
        int *devBboxHost, int *devBboxDevice, int *devImageHost, int *devImageDevice, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    /* Copy the host input vectors to device */
    err = cudaMemcpy(devBboxHost, bboxHost, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector A from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devBboxDevice, bboxDevice, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector B from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devImageHost, imageHost, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector B from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devImageDevice, imageDevice, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector B from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}

/* Runnable device to host call to copy output data from device memory to host memory */
extern "C"
static void cudaCopyDeviceToHost(int *bboxHost, int *bboxDevice, int *imageHost, int *imageDevice,
        int *devBboxHost, int *devBboxDevice, int *devImageHost, int *devImageDevice, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(bboxHost, devBboxHost, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector C from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(bboxDevice, devBboxDevice, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector C from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(imageHost, devImageHost, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector C from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(imageDevice, devImageDevice, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector C from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}


/* Function to detect the object and process the image. The output of this function is provided
 * to the pathPlanner for further processing */

extern "C"
void cuDetectObject(const char *func, detectObject *objdetected)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    // Take some random number of elements 512KB considered.
    unsigned int numElements = (512 * 1024);
    size_t size = numElements * sizeof(int);

    // Allocate the host input vector bboxHost
    int *bboxHost = (int *)malloc(size);
    // Allocate the host input vector bboxDevice
    int *bboxDevice = (int *)malloc(size);
    // Allocate the host output vector imageHost
    int *imageHost = (int *)malloc(size);
    // Allocate the host output vector imageDevice
    int *imageDevice = (int *)malloc(size);

    // Verify that allocations succeeded
    if (bboxHost == NULL || bboxDevice == NULL ||
            imageHost == NULL || imageDevice == NULL)
    {
        fprintf(stderr, "Failed to allocate host vectors!\n");
        exit(EXIT_FAILURE);
    }

    // Initialize the host input vectors
    for (int i = 0; i < numElements; ++i)
    {
        bboxHost[i] = objdetected->bboxHostDetection;
        bboxDevice[i] = objdetected->bboxDeviceDetection;
        imageHost[i] = objdetected->imageHostDetection;
        imageDevice[i] = objdetected->imageDeviceDetection;
    }

    // Allocate the device input vector devBboxDevice
    int *devBboxHost = NULL;
    err = cudaMalloc((void **)&devBboxHost, size);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector devBboxHost (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Allocate the device input vector devBboxDevice
    int *devBboxDevice = NULL;
    err = cudaMalloc((void **)&devBboxDevice, size);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector devBboxDevice (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Allocate the device output vector devImageHost
    int *devImageHost = NULL;
    err = cudaMalloc((void **)&devImageHost, size);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector devImageHost (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }


    // Allocate the device output vector devImageDevice
    int *devImageDevice = NULL;
    err = cudaMalloc((void **)&devImageDevice, size);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector devImageDevice (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }


    /* Runnable Host to device call to copy input data from host memory to device memory */
    cudaCopyHostToDevice(bboxHost, bboxDevice, imageHost, imageDevice,
                            devBboxHost, devBboxDevice, devImageHost, devImageDevice, size);


    // Launch the Vector Add CUDA Kernel
    int threadsPerBlock = 256;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    //fprintf(stdout,"CUDA kernel launch with %d blocks of %d threads from parent thread %s\n"
    //        , blocksPerGrid, threadsPerBlock, func);
    /* Runnable to process the image */
    processImage<<<blocksPerGrid, threadsPerBlock>>>(devBboxHost, devBboxDevice, devImageHost, devImageDevice, numElements);
    cudaDeviceSynchronize();
    err = cudaGetLastError();

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch vectorAdd kernel (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    /* Runnable device to host call to copy output data from device memory to host memory */
    cudaCopyDeviceToHost(bboxHost, bboxDevice, imageHost, imageDevice,
            devBboxHost, devBboxDevice, devImageHost, devImageDevice, size);

    /* Copy the data to output buffer */
    memcpy((int *)&objdetected->bboxDeviceDetection, (int *)&bboxDevice[0], sizeof(int));
    memcpy((int *)&objdetected->bboxHostDetection, (int *)&bboxHost[0], sizeof(int));
    memcpy((int *)&objdetected->imageDeviceDetection, (int *)&imageDevice[0], sizeof(int));
    memcpy((int *)&objdetected->imageHostDetection, (int *)&imageHost[0], sizeof(int));

    // Free device global memory
    err = cudaFree(devBboxDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to free device vector A (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaFree(devBboxHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to free device vector B (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaFree(devImageDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to free device vector C (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaFree(devImageHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to free device vector C (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Free host memory
    free(bboxDevice);
    free(imageDevice);
    free(bboxHost);
    free(imageHost);

    bboxDevice = bboxHost = imageDevice = imageHost = NULL;
}



// Kernel that executes on the CUDA device
__global__ void processSFMData(float *sum, int nbin, float step, int nthreads, int nblocks) {
    int i;
    float x;
    int idx = blockIdx.x*blockDim.x+threadIdx.x;  // Sequential thread index across the blocks
    for (i=idx; i< nbin; i+=nthreads*nblocks) {
        x = (i+0.5)*step;
        sum[idx] += 4.0/(1.0+x*x);
    }
}
extern "C"
void cuObjDetectSFM(const char *func, sfmData *sfmInput)
{
    dim3 dimGrid(NUM_BLOCK,1,1);  // Grid dimensions
    dim3 dimBlock(NUM_THREAD,1,1);  // Block dimensions
    float *sumHost, *sumDev;  // Pointer to host & device arrays
    int tid;
    float pi = 0;

    float step = 1.0/NBIN;  // Step size
    size_t size = NUM_BLOCK*NUM_THREAD*sizeof(float);  //Array memory size
    sumHost = (float *)malloc(size);  //  Allocate array on host
    cudaMalloc((void **) &sumDev, size);  // Allocate array on device
    // Initialize array in device to 0
    cudaMemset(sumDev, 0, size);
    // Do calculation on device
    processSFMData <<<dimGrid, dimBlock>>> (sumDev, NBIN, step, NUM_THREAD, NUM_BLOCK); // call CUDA kernel
    // Retrieve result from device and store it in host array
    cudaMemcpy(sumHost, sumDev, size, cudaMemcpyDeviceToHost);
    for(tid=0; tid<NUM_THREAD*NUM_BLOCK; tid++)
        pi += sumHost[tid];
    pi *= step;

    // Print results
    //printf("PI = %f\n",pi);

    // Cleanup
    free(sumHost);
    cudaFree(sumDev);
}

