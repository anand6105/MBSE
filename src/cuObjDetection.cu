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
 * @file cuObjDetection.cu
 * @author Anand Prakash
 * @date 17 April 2020
 * @brief This file contains the CUDA kernel implementation of Detection and Structure-
 *        from-motion tasks.
 *
 * This file implements runnables executed for Detection and Structure-from-Motion tasks.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

extern "C"{
    #include "mbseCuda.h"
}


/**
 * CUDA Kernel Device code
 *
 * Runnable to Process the image to detect and classify the objects by creating the Boundary Box.
 */
__global__ void
processImage(int *hostBbox, int *devBbox, int *hostImage, int *devImage, int numElements)
{
    int index = blockDim.x * blockIdx.x + threadIdx.x;

    if (index < numElements)
    {
        hostBbox[index] += hostBbox[index];
        devBbox[index] += devBbox[index];
        hostImage[index] += hostImage[index];
        devImage[index] += devImage[index];
    }
}

/* Runnable Host to device call to copy input data from host memory to device memory */
extern "C"
static void detectionCopyHostToDevice(int *bboxHost, int *bboxDevice, int *imageHost, int *imageDevice,
        int *devBboxHost, int *devBboxDevice, int *devImageHost, int *devImageDevice, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    /* Copy the host input vectors to device */
    err = cudaMemcpy(devBboxHost, bboxHost, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy host boundary box from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devBboxDevice, bboxDevice, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device boundary box from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devImageHost, imageHost, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy host image from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devImageDevice, imageDevice, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device image from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}

/* Runnable device to host call to copy output data from device memory to host memory */
extern "C"
static void detectionCopyDeviceToHost(int *bboxHost, int *bboxDevice, int *imageHost, int *imageDevice,
        int *devBboxHost, int *devBboxDevice, int *devImageHost, int *devImageDevice, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(bboxHost, devBboxHost, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy vector host boundary box from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(bboxDevice, devBboxDevice, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device boundary box from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(imageHost, devImageHost, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy host image from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(imageDevice, devImageDevice, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy device image from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}


/**
 * @brief Function to process the Detection task.
 *
 * Function to detect the object and process the image. The output of this function is provided
 * to the pathPlanner for further processing. It has three runnables.
 *
 * @param func              Function name
 * @param objdetected       Pointer to structure to detectObject input data
 *
 * @return void
 */
extern "C"
void cuDetectObject(const char *func, detectObject *objdetected)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    cudaEvent_t start, stop;

    // Take some random number of elements 256KB considered.
    unsigned int numElements = (896 * 2048);
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
        bboxHost[i] = objdetected->bboxHostDetection++;
        bboxDevice[i] = objdetected->bboxDeviceDetection++;
        imageHost[i] = objdetected->imageHostDetection++;
        imageDevice[i] = objdetected->imageDeviceDetection++;
    }

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

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

    cudaEventRecord(start, 0);
    /* Runnable Host to device call to copy input data from host memory to device memory */
    detectionCopyHostToDevice(bboxHost, bboxDevice, imageHost, imageDevice,
                            devBboxHost, devBboxDevice, devImageHost, devImageDevice, size);


    // Launch the Vector Add CUDA Kernel
    int threadsPerBlock = 2;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
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
    detectionCopyDeviceToHost(bboxHost, bboxDevice, imageHost, imageDevice,
            devBboxHost, devBboxDevice, devImageHost, devImageDevice, size);

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"Detection is %4.5f ms \n", elapsedTime);

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



// Kernel that executes on the CUDA device to process the SFM data
__global__ void processSFMData(int *image, int * matrix, int nbin, int step, int nthreads, int nblocks) {
    int i;
    int x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;  // Sequential thread index across the blocks
    for (i=idx; i< nbin; i += (nthreads * nblocks)) {
        x = (i+1)*step;
        image[idx] += (1.0+x*x);
        matrix[idx] += (1.0+x*x);
    }
}



/* Runnable Host to device call to copy input data from host memory to device memory */
extern "C"
static void sfmCopyHostToDevice(int *hostImage, int *devImage, int *hostMatrix,
                                int *devMatrix, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;

    /* Copy the host input vectors to device */
    err = cudaMemcpy(devImage, hostImage, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy SFM host Image from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    err = cudaMemcpy(devMatrix, hostMatrix, size, cudaMemcpyHostToDevice);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy SFM host matrix from host to device (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}



/* Runnable device to host call to copy output data from device memory to host memory */
extern "C"
static void sfmCopyDeviceToHost(int *imageHost, int *devImage,int *matrixHost,
                                int *devMatrix, size_t size)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(imageHost, devImage, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy image Host from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(matrixHost, devMatrix, size, cudaMemcpyDeviceToHost);

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy matrix host from device to host (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    return;
}


/**
 * @brief Function to process the SFM task.
 *
 * The functions process the data received from the input buffer and generates the input data for the
 * planner task. It has three runnables.
 *
 * @param func           Function name
 * @param sfmInput       Pointer to structure to SFM input data
 *
 * @return void
 */
extern "C"
void cuObjDetectSFM(const char *func, sfmData *sfmInput)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    cudaEvent_t start, stop;
    // Take some random number of elements 768 * 768 considered.
    unsigned int numElements = (512 * 512);
    size_t size = numElements * sizeof(int);

    int *sfmImage = (int *)malloc(size);  //  Allocate array on host
    int *sfmMatrix = (int *)malloc(size);  //  Allocate array on host
    if ((sfmImage == NULL) || (sfmMatrix == NULL))
    {
        fprintf(stderr, "Failed to allocate host vectors!\n");
        exit(EXIT_FAILURE);
    }

    sfmInput->imageSFMHost++;
    sfmInput->matrixSFMHost++;
    for(int idx = 0; idx < numElements; idx++){
        sfmImage[idx] = sfmInput->imageSFMHost;
        sfmMatrix[idx] = sfmInput->matrixSFMHost;
    }

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    int *devImage = NULL;
    err = cudaMalloc((void **)&devImage, size);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device Image (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    int *devMatrix = NULL;
    err = cudaMalloc((void **) &devMatrix, size);  // Allocate array on device
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to allocate device vector devMatrix (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }


    // Initialize array in device to 0
    cudaMemset(devImage, 0, size);
    cudaMemset(devMatrix, 0, size);

    cudaEventRecord(start, 0);
    sfmCopyHostToDevice(sfmImage, devImage, sfmMatrix, devMatrix, size);
    // Taking thread per block as 64 and step size as 1
    int threadsPerBlock = 64;
    int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
    int step = 1;
    // Do calculation on device
    processSFMData <<<blocksPerGrid, threadsPerBlock>>> (devImage, devMatrix, numElements, step, threadsPerBlock, blocksPerGrid); // call CUDA kernel
    cudaDeviceSynchronize();

    sfmCopyDeviceToHost(sfmImage, devImage, sfmMatrix, devMatrix, size);

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"SFM task is %4.5f ms \n", elapsedTime);

    // Cleanup
    cudaFree(devMatrix);
    cudaFree(devImage);
    free(sfmImage);
    free(sfmMatrix);

}

