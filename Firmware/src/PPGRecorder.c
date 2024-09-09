/*
 * Copyright (c) 2024 Springer Design, Inc
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */


#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>

#include <stdio.h>
#include <string.h>

#include "DataFrame.h"
#include "battery.h"

extern void	set_charger( bool enable );
extern bool check_usb( void );
extern int uart_init( void );

extern struct bt_conn *current_conn;
extern bool uart_dump;
extern uint16_t nBattery;

extern uint8_t dumpType;

void PPGRecorder( void )
{
	int err = 0;
	int batt_mV;
	uint32_t MaxEvents;
	int FrameCount;
	bool UARTLive = false;
	int z;

	err = max86171_init();

	printk( "Got %d from max86171 init\n", err );

	if( err != 0 )					// If MAX86171 didn't init properly
	{
		while( true )
		{
			/* Spin for ever */
			k_sleep( K_MSEC( 1000 ) );
		}
	}

	while( 1 )
	{
		while( current_conn == NULL && uart_dump == false )		// Wait for a connection or a uart dump request
		{
			k_sleep( K_SECONDS( 1 ) );
			batt_mV = battery_sample();
			nBattery = (uint16_t)batt_mV;
			if( !UARTLive )
			{
				if( check_usb() )
				{
					set_charger( false );
					err = uart_init();
					if( err )
					{
						while( true )
						{
							/* Spin for ever */
							k_sleep( K_MSEC( 1000 ) );
						}
					}
					UARTLive = true;
					printk( "UART initialized.\n" );
				}
			}
		}

		InitFrames();
		StartFrames( true );
		if( max86171_get_event_count() )
		{
			err = max86171_read_fifo();
		}

		z = 0;
		while( current_conn != NULL || uart_dump == true )		// As long as a connection exists or a uart dumping
		{	
			batt_mV = battery_sample();
			nBattery = (uint16_t)batt_mV;
			MaxEvents = k_event_wait( &max_event, MAX_ALL_EVENTS, true, K_FOREVER );
			if( MaxEvents == MAX_FIFO_EVENT )
			{
				
				err = max86171_read_fifo();
				if( err > 0 && (current_conn != NULL || uart_dump == true) )
				{
					FrameCount = GetFIFOFrame();
					if( FrameCount == SERVOCOUNT )
					{
						z = ServoFrame();
						printk( "Servo returned %d\n", z );
						if( z )
							FrameCount = 0;
					}
					else if( FrameCount == 0 )
					{
						printk( "Frame event but no data available\n" );
					}
					printk( "framecount = %d\n", FrameCount );
				}
			}
		}
		StartFrames( false );		// Stop collecting
		InitFrames();
	}
}

extern uint32_t irDC;
extern double irMod;
extern uint32_t redDC;
extern double redMod;

void format_uart_dump( FIFO_RAW_DATA *buf, char *dest_buf, int dest_len )
{
	uint8_t n;
	uint32_t r;
	uint32_t ir;
	char rdrv[8];
	char irdrv[8];
	char rsig[12];
	char irsig[12];
	char rdc[12];
	char irdc[12];
	char rmod[8];
	char irmod[8];
	int i = 0;

	memset( dest_buf, 0, dest_len );			// Clear destination buffer

	snprintf( rdc, sizeof( rdc ), "%u,", redDC );
	snprintf( irdc, sizeof( irdc ), "%u,", irDC );
	snprintf( rmod, sizeof( rmod ), "%.1f,", redMod );
	snprintf( irmod, sizeof( irmod ), "%.1f\r\n", irMod );
	
	while( i < buf->len )
	{
		if( i == 0 && buf->data[i] == 0xF0 )	// 3 LED drive levels are next
		{
			++i;
			n = buf->data[i];
			snprintf( rdrv, sizeof( rdrv ), "%d,", n );

			++i;
			n = buf->data[i];
			snprintf( irdrv, sizeof( irdrv ), "%d,", n );

			++i;
			++i;			// skip green too
			continue;
		}
		i += 3;
		ir = (uint32_t)((buf->data[i] << 16) | (buf->data[i + 1] << 8) | buf->data[i + 2]);
		snprintf( irsig, sizeof( irsig ), "%u,", (ir & MAX86171_MASK_FIFO_DATA) );
		i += 3;
		r = (uint32_t)((buf->data[i] << 16) | (buf->data[i + 1] << 8) | buf->data[i + 2]);
		snprintf( rsig, sizeof( rsig ), "%u,", (r & MAX86171_MASK_FIFO_DATA) );
		i += 3;
		strcat( dest_buf, rsig );
		strcat( dest_buf, rdrv );
		strcat( dest_buf, rdc );
		strcat( dest_buf, rmod );
		strcat( dest_buf, irsig );
		strcat( dest_buf, irdrv );
		strcat( dest_buf, irdc );
		strcat( dest_buf, irmod );
	};
}
