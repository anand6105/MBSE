/*! \mainpage Reverse Engineering Waters Challenge 2019
 *
 * \section Introduction
 *
 * This project is developed to reverse engineer the Waters Challenge 2019 model and 
 * implement it on Jetson TX2 platform using the capability of GPU accelerator.  
 * The processing power offered by GPUs and their capability to execute parallel
 * workloads is exploited to execute and accelerate applications related to     
 * advanced driver assistance systems.                                          
 *                                                                              
 * The challenge consists in analytically master the complex HW/SW system (that 
 * will be available as Amalthea model) to answer the following questions:      
 *                                                                              
 * Response Time Computation: 
 *                                                  
 *  a) Given an application consisting of a set of dependent tasks and a given  
 *     mapping, calculate its end-to-end response time.
 *                         
 *  b) The response time should account for the time for the copy engine to     
 *     transfer data between the CPU and the GPU.
 *                               
 *  c) The response time should account for the time for the data transfers between 
 *     the CPU and the shared main memory considering a read/execute/write semantic.
 *
 *  d) Optionally the off-loading mechanism (synchronous/asynchronous) can be       
 *     changed to further optimize the end-to-end latencies.                        
 *
 * The below sections consists of the steps for headless installation the RT-Linux on a 
 * Jetson TX2 platform. 
 *
 * \section  install_sec Installation Setup
 *
 * The application is implemented on NVIDIA Jetson TX2 board. At a high-level abstraction, 
 * embedded heterogeneous SoCs (System of Chip) featuring GP-GPU accelerators are characterized
 * by the following hardware components:
 *
 * 1) CPU
 *
 * 2) Accelerator
 *
 * 3) Memory hierarchy
 *
 * However, the code can be compiled and executed on any NVIDIA supported hardware device.
 *
 * The application uses three ARM A57 cores for running the applications parallelly. Each task 
 * on the CPU core follows RMS(Rate Monotonic Scheduling) approach. This scheduling approach is 
 * core specific. The kernel must support real time time scheduling in order to execute this application.
 *
 * The installation setup process mentioned below is on x86 Host machine running Ubuntu 18.04 64 bit Linux version.
 *
 * \subsection headless_install Jetson TX2 Headless installation
 *
 * Connect your Jetson TX2 device to your host machine via the USB cable and connect a wired ethernet to the device. 
 * Power on the device.
 *
 * Download the NVIDIA SDK Manager from the NVIDIA website on your host machine.
 *
 * - Start the sdkmanager and follow the steps to install the packages for the Jetson TX2 device.
 * - Choose the latest Jetson OS image available.
 * - The SDK manager will prompt to flash the OS on the Jetson TX2 target device.
 * - Switch the Jetson TX2 device to recovery mode by pressing and holding the RECOVERY button and then press and 
 * release the RESET button.
 * - Release the RECOVERY button after 2 seconds.
 * - Wait for the SDK manager to flash the OS on the Jetson TX2 target device.
 * - Install screen utility on the host Linux machine.
 * - The USB port for the Jetson TX2 device will appear in the /dev folder of the Host machine with value as ttyACM* device.
 * - Start the screen utility at a baud rate of 115200. screen /dev/ttyACM0 115200.
 * - The screen prompt will enable for the target device which can be used to configure the device. Follow the steps as prompted.
 * - Execute ifconfig command to get the IP address. Once the IP address is known, ssh can be used to connect to the Jetson TX2 board.
 * 
 * \subsection rt-linux Installing Real Time Kernel on NVIDIA Jetson TX2 board.
 *
 *
 * Follow below steps to install the SMP PREEMPT RT on NVIDIA Jetson TX2 board. Login to the 
 * NVIDIA Jetson TX2 device through ssh. Create a folder and open a terminal in that folder.
 *
 *
 * - git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
 * - checked out the tag vL4T32.1.0 - Same Tegra version as on the board
 * - Executed the script sudo ./getKernelSources.sh followed by sudo ./makeKernel.sh - Kernel will build successfully.
 * - cd /usr/src/kernel/kernel-4.9/scripts/rt-patches.sh. Executed the patch as : sudo ./scripts/rt-patches.sh apply-patches. 
 * Patch should  apply successfully.
 * - Go to the buildJetsonTX2Kernel folder.
 * - sudo ./makeKernel.sh
 * - sudo ./copyImage.sh - Everything compiled successfully and the image is also copied in the /boot folder.
 * -sudo reboot
 * - Login to the Jetson TX2 board and execute the command uname -a. The printed string must contain SMP PREEMPT RT in it.
 *
 * 
 * \section Dev_env Development Environment
 *
 * NVIDIA Eclipse Nsight edition is used for the development and debug of the application.
 *
 * \subsection exec Running the Application
 *
 * - git clone https://github.com/anand6105/MBSE.git
 * - cd MBSE
 * - make all
 * - jetsonsim executable will generated.
 * - Run the jetsonsim executable with sudo privileges.
 *
 * The application prints the time taken by each task in execution on the NVIDIA GPU device.
 */

/************************************************************************************
 * Copyright (C) 2020 Dortmund University of Applied Sciences and Arts and Others.  *
 *                                                                                  *
 ************************************************************************************/


/**
 * @file main.c
 * @author Anand Prakash
 * @date 11 April 2020
 * @brief This file contains the entry point of the applications and consists of the
 *        initialization of memory, threads of the application
 *
 * This is the entry point of the application. The application can be executed only with
 * root/superuser privileges and get the CUDA device properties along with initializing
 * the threads and start their execution.
 * The tasks are executed on three ARM A57 cores and uses RMS scheduling approach.
 *
 * @see https://www.ecrts.org/archives/fileadmin/WebsitesArchiv/ecrts2019/waters/index.html
 */

#include "mbse.h"
#include "mbseCuda.h"

/* Function pointer declaration for the the task specific threads */
typedef void *(*threadPool_t)(void *);

/* @brief Threadpool declaring the function pointer for all the tasks*/
static threadPool_t threadPool[MBSE_NUMBER_OF_THREADS] =
                            {objDetectGetObject, objDetectStructureFromMotion,
                                    pathPlannerCanBusPolling, computeOSOverhead,
                                    computeSpeedAndSteer, pathPlannerCalculation};

/**
 * @brief Function to start all the task threads.
 *
 * The functions initializes and creates all the task threads mentioned in the thread pool.
 * In case of any error it prints the error message with an error code.
 *
 * @return void
 */
static void startRealTimeThreads(void)
{
    pthread_t thread[MBSE_NUMBER_OF_THREADS];
    pthread_attr_t attr[MBSE_NUMBER_OF_THREADS];
    uint8_t threadIndex = 0;

    for(threadIndex = 0; threadIndex < MBSE_NUMBER_OF_THREADS; threadIndex++)
    {
        /* init to default values */
        if (pthread_attr_init(&attr[threadIndex]))
        {
            error(1);
        }

        /* And finally start the actual threads */
        if (pthread_create(&thread[threadIndex],
                &attr[threadIndex],
                threadPool[threadIndex], NULL))
        {
            error(2);
        }
    }
    for(threadIndex = 0; threadIndex < MBSE_NUMBER_OF_THREADS; threadIndex++)
    {
        pthread_join(thread[threadIndex], NULL);
    }
}


/* Entry point of the application. The application does not need any
 * mandatory arguments. */
int main(int argc, char *argv[])
{
    uint8_t userPrivilege;
    /* Check whether the application is started as root. Thread scheduling,
     * assigning cores and priorities require root privileges. */
    userPrivilege = getuid();
    if(userPrivilege != 0)
    {
        fprintf(stdout, "The current user is not root\n");
        fflush(stdout);
        exit(1);
    }

    /* Get the CUDA properties */
    getCudaDeviceProperties();
    /* Start spawning threads */
    startRealTimeThreads();
    return 0;
}

