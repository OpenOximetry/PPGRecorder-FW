
# Firmware Design Origins

The PPG Recorder firmware is derived from Nordic Semiconductor’s nRF Connect SDK (nRFSDK) and its many samples. The target SOC is the nRF5340 which is a dual core Arm Cortex m33 device. The Zephyr RTOS is a key component in this implementation as it’s operating system, Bluetooth Low Energy (BLE) communication stack and it’s many low level hardware device drivers

# Firmware Design

The firmware’s sole purpose is to collect a stream of PPG data from the Analog Devices’s MAX86171 Analog Front End (AFE), separate it into each of the sensor’s LED color, convert it to ASCII and format it and send it out the serial port. This process is controlled by 3 separate Zephyr threads; a main thread that waits for data availability from the AFE then retrieves the data, separates the data, calculates a DC value and pulse amplitude for each color finally forwarding it to format thread which converts the binary data to formatted ASCII data which is then passed to the serial transmit thread. A hardware interrupt and an event semaphore are used to signal data availability from the AFE. An 2 deep Zephyr FIFO buffer is used to transport the raw data to the format thread. 
While this is not the forum to describe in detail the design and implementation of nRFSDK features and functions, suffice to say that these are used as published at https://docs.nordicsemi.com/bundle/ncs-2.3.0/page/nrf/index.html An important aspect is this firmware is implemented using nRFSDK version 2.3.0. There are later versions of nRFSDK but I have found that the regression testing required of each new version of the SDK is substantial. For now, version 2.3.0 works.

## MAX86171 driver

This library is proprietary to Springer Design, Inc. and the source code will not be released. The following is description of the library’s functionality and its API.
The driver requires a Zephyr event semaphore to signal the main thread that data is available, this must be created in main.c and named max_event. When max_event is posted, the main thread must call max86171_read_fifo() to retrieve the available data from the MAX86171, failure to promptly this function will result in MAX86171 FIFO overflow errors. The API consists of the following functions:
**int max86171_init( void )** – Initializes the MAX86171 to operate as follows:
	86 frames per second
	3 LED drives, Green\*, IR and Red defaulting to 10 milliamps of drive current
	Using photodetector 1, photodetector 2 is powered down
	Analog ambient light cancellation is enables
	117.1 uS integration time, no averaging
	SINC3 filter is not used, Ambient light correction uses the central difference method
The function returns 0 if successful, -EIO or -ENODEV if not.

**int max86171_start( bool** *start* **)** – Starts (if *start* is true) or stops (if *start* is false) data collection, actually powers up or powers done the MAX86171.
The function returns 0 if successful, -EIO if not.

**int max86171_read_fifo( void )** – Reads the FIFO data from the MAX86171 and stores it in a ping-pong buffer. This function must be called from the main thread because Zephyr I2C communications is not allowed during interrupt service.
The function returns the number of bytes read from the FIFO if successful, -1, -2 or -3 if not.

**int max86171_get_fifo( int** *index***, int** *frame* **)** – Reads FIFO data from the appropriate ping-pong buffer and places it in the frame buffers specified by *index* and *frame*. The frame buffers are expected to be statically allocated by the calling function with the names GFrame, IRFrame and RFrame additionally a 1 second raw data buffer is expected of the type FIFO_RAW_DATA and name fifoRawData. Please see source file DataFrame.c for an example.
The function returns the number of samples read from the ping pong buffer if successful, a negative number if not. The number of samples is the byte count of individual 24 bit ADC values for each color LED, a full one second frame is 258 bytes (3 colors, 3 bytes each times 86 values). A negative return value will indicate a number of different conditions including bad FIFO data and ADC overflows, again, please see DataFrame.c for examples of error handling. 

**int max86171_get_event_count( void )** – This function returns the number of FIFO events that have occurred. The primary purpose of this function is detect false max_event postings

**int max86171_sync( void )** – This function is used to resyncronize the MAX86171 FIFO after an error condition.
The function returns 0 if successful, -EIO if not.

**uint8_t max86171_get_green( void )** – This functions returns the green LED drive level, the value will be 2 to 255. Each count represents 0.5 milliamp.

**int max86171_set_green( uint8_t** *drv* **)** – This function set the green LED drive level, *drv* can be 2 to 255. Each count represents 0.5 milliamp.
The function returns 0 if successful, -EIO if not.

**uint8_t max86171_get_ ir ( void )** – This functions returns the IR LED drive level, the value will be 2 to 255. Each count represents 0.5 milliamp.

**int max86171_set_ ir ( uint8_t** *drv* ) – This function set the IR LED drive level, *drv* can be 2 to 255. Each count represents 0.5 milliamp.
The function returns 0 if successful, -EIO if not.

**uint8_t max86171_get_red( void )** – This functions returns the red LED drive level, the value will be 2 to 255. Each count represents 0.5 milliamp.

**int max86171_set_red( uint8_t** *drv* **)** – This function set the red LED drive level, *drv* can be 2 to 255. Each count represents 0.5 milliamp.
The function returns 0 if successful, -EIO if not.

**Notes:** \* The Green LED is not used in this implementation as it requires a custom sensor. While the Green data is collected, it is ignored by the application.

# Building the firmware

Prior to attempting to build the firmware, nRFSDK version 2.3.0 must be installed on the build system. Instructions for this process can be found at https://docs.nordicsemi.com/bundle/ncs-2.3.0/page/nrf/getting_started.html All of the PPG Recorder files necessary to build the firmware are included in this repository. There are 2 approaches to building the application, via the command line and using Microsoft’s Visual Studio Code. This document will describe using Windows Powershell. The Powershell starting point is changing to the folder where the project resides such as “cd \PPGRecorder\firmware”. Next set the build environment via the command “$env:ZEPHYR_BASE='C:\ncs\v2.3.0\zephyr'” and finally build the firmware via the command “west build -b sdirefox_nrf5340_cpuapp” where the -b option specifies the Board Support file which in this case is included in the repository. Programming, test and debug requires a debug probe such as the Segger J-Link (https://www.segger.com/products/debug-probes/j-link/).

# License Information

Each source file includes the license type in the file header with the exception of the library max86171lib.a which is licensed under GNU AFFERO GENERAL PUBLIC LICENSE.

#Additional Information#

This project was implemented and completed by Springer Design, Inc. we can be contacted via email at openoximetry@springerdesign.com. We can provide custom development services as well as support your development efforts.
