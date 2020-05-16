# MBSE
This repository is used for the development and analysis of WATERS CHALLENGE 2019.

# Copyright
Copyright (C) 2020 Dortmund University of Applied Sciences and Arts and Others.

# Description
This project is part of the implementation of 2019 Waters Challenge. The processing power offered by GPUs and their capability to execute parallel workloads is exploited to execute and accelerate applications related to advanced driver assistance systems.                                              
                                                                                 
The challenge consists in analytically master the complex HW/SW system (that will be available as Amalthea model) to answer the following questions:          
                                                                                 
Response Time Computation:                                                       
 * Given an application consisting of a set of dependent tasks and a given mapping, calculate its end-to-end response time.                       
 * The response time should account for the time for the copy engine to transfer data between the CPU and the GPU.                   
 * The response time should account for the time for the data transfers between the CPU and the shared main memory considering a read/execute/write semantic.
 * Optionally the off-loading mechanism (synchronous/asynchronous) can be changed to further optimize the end-to-end latencies. 

# Installation Setup
The application is implemented on NVIDIA Jetson TX2 board. At a high-level abstraction, embedded heterogeneous SoCs (System of Chip) featuring GP-GPU accelerators are characterized by the following hardware components: 
  1. CPU 
  2. Accelerator
  3. Memory hierarchy
  
However, the code can be compiled and executed on any NVIDIA supported hardware device.

The application uses three ARM A57 cores for running the applications parallelly. Each task on the CPU core follows RMS(Rate Monotonic Scheduling) approach. This scheduling approach is core specific. The kernel must support real time time scheduling in order to execute this application. 

The installation setup process mentioned below is on x86 Host machine running Ubuntu 18.04 64 bit Linux version.

## Jetson TX2 Headless installation

Connect your Jetson TX2 device to your host machine via the USB cable and connect a wired ethernet to the device. Power on the device.

Download the NVIDIA SDK Manager from the NVIDIA website on your host machine. 
* Start the sdkmanager and follow the steps to install the packages for the Jetson TX2 device. 
* Choose the latest Jetson OS image available.
* The SDK manager will prompt to flash the OS on the Jetson TX2 target device.
* Switch the Jetson TX2 device to recovery mode by pressing and holding the RECOVERY button and then press and release the RESET button. Release the RECOVERY button after 2 seconds.
* Wait for the SDK manager to flash the OS on the Jetson TX2 target device.
* Install screen utility on the host Linux machine.
* The USB port for the Jetson TX2 device will appear in the /dev folder of the Host machine with value as ttyACM* device.
* Start the screen utility at a baud rate of 115200. **screen /dev/ttyACM0 115200**.
* The screen prompt will enable for the target device which can be used to configure the device. Follow the steps as prompted.
* Execute **ifconfig** command to get the IP address. Once the IP address is known, ssh can be used to connect to the Jetson TX2 board.

## Installing Real Time Kernel on NVIDIA Jetson TX2 board.

Follow below steps to install the SMP PREEMPT RT on NVIDIA Jetson TX2 board.
Login to the NVIDIA Jetson TX2 device through ssh. Create a folder and open a terminal in that folder.

1) git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
2) checked out the tag vL4T32.1.0 - Same Tegra version as on the board
3) Executed the script sudo ./getKernelSources.sh followed by sudo ./makeKernel.sh - Kernel will build successfully.
4) cd /usr/src/kernel/kernel-4.9/scripts/rt-patches.sh. Executed the patch as : **sudo ./scripts/rt-patches.sh apply-patches**. Patch should apply successfully.
5) Go to the buildJetsonTX2Kernel folder.
6) sudo ./makeKernel.sh
7) sudo ./copyImage.sh - Everything compiled successfully and the image is also copied in the /boot folder.
8) sudo reboot

Login to the Jetson TX2 board and execute the command **uname -a**. The printed string must contain **SMP PREEMPT RT** in it.

# Development Environment

NVIDIA Eclipse Nsight edition is used for the development and debug of the application.

# Running the Application

1. git clone **https://github.com/anand6105/MBSE.git**
2. cd MBSE
3. make all
4. jetsonsim executable will generated.
5. Run the jetsonsim executable with sudo privileges.
6. The application prints the time taken by each task in execution on the NVIDIA GPU device.

