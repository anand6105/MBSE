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
 * @file cuPathPlanner.cu
 * @author Anand Prakash
 * @date 12 May 2020
 * @brief This file contains the CUDA kernel implementation of Planner and CAN Bus
 *        Polling tasks.
 *
 * This file implements runnables executed for Planner and CAN Bus Polling tasks.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

extern "C"{
    #include "mbseCuda.h"
}


// Kernel that executes on the CUDA device to process the CAN Bus data.
__global__ void getCanBusData(int *canData, int size, int nthreads, int nblocks) {
    int i;
    int idx = blockIdx.x*blockDim.x+threadIdx.x;  // Sequential thread index across the blocks
    for (i=idx; i< size; i+=nthreads*nblocks) {
        canData[idx] += 1;
    }
}

/**
 * @brief Function to process the CAN bus polling task.
 *
 * The functions process the data obtained from the global buffer. It provides this value as n input
 * to Planner task.
 *
 * @param func                     Function name
 * @param hostCanPollingData       Pointer to host can polling data.
 *
 * @return void
 */
extern "C"
void cuPlannerFetchCanBusData(const char *func, int *hostCanPollingData)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    cudaEvent_t start, stop;
    int *canHost = NULL;
    // Set some random value to meet Amalthea model timing requirements.
    int numOfElements = (512 * 512);
    size_t size = (numOfElements *sizeof(int));  //Array memory size
    int *hostData = (int *)malloc(size);
    for(int idx = 0; idx < numOfElements; idx++)
    {
        hostData[idx] = *hostCanPollingData;
    }

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaMalloc((void **) &canHost, size);  // Allocate array on device
    // Initialize array in device to 0
    cudaMemcpy(canHost, hostData, size, cudaMemcpyHostToDevice);
    cudaEventRecord(start, 0);

    int threadsPerBlock = 8;
    int blocksPerGrid =(numOfElements + threadsPerBlock - 1) / threadsPerBlock;
    getCanBusData <<<blocksPerGrid, threadsPerBlock>>> (canHost, numOfElements, threadsPerBlock, blocksPerGrid); // call CUDA kernel
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"CAN polling task time is %4.5f ms \n", elapsedTime);

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(hostData, canHost, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write can data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    memcpy(hostCanPollingData, &hostData[0], sizeof(int));

    cudaFree(canHost);
    free(hostData);
}



// Kernel that executes on the CUDA device to process the path planner task.
__global__ void pathPlan( int *devSpeed, int *devSteer, int size )
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    while (tid < size) {
        devSpeed[tid] += 1;
        devSteer[tid] += 1;
        tid += blockDim.x * gridDim.x;
    }
}

/**
 * @brief Function to process the Planner task.
 *
 * The functions process the data received from the SFM, Detection, Can BUs Polling,
 * grid data and lane boundary detection and process the data to generate the input to the DASM task.
 *
 * @param func       Function name
 * @param data       Pointer to structure to planner input data
 *
 * @return void
 */
extern "C"
void cuPlannerInterpolatePath(const char *func, plannerData *data)
{
    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    cudaEvent_t start, stop;
    int *devSpeed = NULL, *devSteer = NULL;
    // Set some random data length
    const int length = 1024 * 2048;
    int size = length * sizeof(int);
    // Set 4 threads per block. To ensure the Amalthea timing
    const int threadPerBlock = 2;
    int blockPerGrid = ((length + threadPerBlock - 1) / threadPerBlock);
    int index = 0;

    int *speedInput = (int *) malloc(size);
    int *steerInput = (int *) malloc(size);
    for (index = 0; index < length; ++index)
    {
        speedInput[index] = data->bBoxHost + data->matrixSFM;
        steerInput[index] = data->laneBoundary + data->canData;
    }

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaMalloc((void **) &devSpeed, size);  // Allocate array on device
    cudaMalloc((void **) &devSteer, size);  // Allocate array on device
    // Initialize array in device to 0
    cudaMemcpy(devSpeed, speedInput, size, cudaMemcpyHostToDevice);
    cudaMemcpy(devSteer, steerInput, size, cudaMemcpyHostToDevice);

    cudaEventRecord(start, 0);
    pathPlan <<<blockPerGrid, threadPerBlock>>> (devSpeed, devSteer, length); // call CUDA kernel
    cudaDeviceSynchronize();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float elapsedTime = 0.0f;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    fprintf(stdout,"Planner task time is %4.5f ms \n", elapsedTime);

    // Copy the device result vector in device memory to the host memory
    err = cudaMemcpy(&data->speedObjective, devSpeed, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write Speed data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    err = cudaMemcpy(&data->steerObjective, devSteer, sizeof(int), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to write Steer data to host memory (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    cudaFree(devSpeed);
    cudaFree(devSteer);
    free(speedInput);
    free(steerInput);
}
