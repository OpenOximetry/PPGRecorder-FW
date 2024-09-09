/*
 * Copyright (c) 2024 Springer Design, Inc
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */


extern struct k_event (max_event);
#define MAX_FIFO_EVENT		0x0001
#define MAX_ERROR_EVENT		0x0002
#define MAX_THRESH_EVENT	0x0004
#define MAX_ALL_EVENTS		0x0007

#define NUM_LEDS 3
#define FRAMESIZE 86
#define SERVOCOUNT 2
#define MAX_RAW_DATA 391
#define DEF_SIGNAL_TARGET 262144
#define MAX_DC_DELAY 60
#define MAX_MOD_DELAY 60

typedef struct _FIFO_RAW_DATA
{
	void *fifo_reserved;
	uint8_t fifoType;			// Flag indicating the type of fifo same as dumpType
	uint8_t data[MAX_RAW_DATA];
	uint16_t len;
} FIFO_RAW_DATA;

typedef struct _FIFO_DATA
{
	void *fifo_reserved;
	uint8_t fifoType;			// Flag indicating the type of fifo same as dumpType
} FIFO_DATA;

// DataFrame function prototypes
void InitFrames( void );
void StartFrames( bool start );
int GetFIFOFrame( void );
int ServoFrame( void );

// MAX 86171 driver constants and function prototypes
#define MAX86171_LED_DRV_DEF		0x14			// Default LED drive current
#define MAX86171_LED_DRV_MIN		0x02			// Minimum LED drive current
#define MAX86171_LED_DRV_MAX		0xFF			// Maximum LED drive current
#define MAX86171_LED_DRV_INC		0x04			// Perferred servo time LED current increment

#define MAX86171_PD_MAX				1048000			// Maximum and minimum PD values
#define MAX86171_PD_MIN				1024

#define MAX86171_MASK_FIFO_DATA		0x0FFFFF		// MAX 86171 FIFO data type mask

int max86171_init( void );
int max86171_read_fifo( void );
int max86171_get_fifo( int index, int frame );
int max86171_get_event_count( void );
int max86171_start( bool start );
int max86171_sync( void );
uint8_t max86171_get_green( void );
int max86171_set_green( uint8_t drv );
uint8_t max86171_get_ir( void );
int max86171_set_ir( uint8_t drv );
uint8_t max86171_get_red( void );
int max86171_set_red( uint8_t drv );


