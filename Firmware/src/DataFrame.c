/*
 * Copyright (c) 2024 Springer Design, Inc
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include "max86171.h"
#include "DataFrame.h"

static uint32_t	DoAverage( uint32_t *src, int size );
static uint32_t	FindPeak( uint32_t *src, int size );
static void AdjustDrives( int whichOnes );


extern struct k_fifo (fifo_spo2_tx_data);
FIFO_RAW_DATA fifoRawData[2];					// 1 second of raw data
int fifoRawIndex;								// Raw data buffer to use
uint32_t dumpSize;								// MTU size for dumping raw data
uint8_t dumpType;								// Flag indicating what type of data to send
												// 'D' = raw, 'X' = nothing
												
uint32_t GFrame[4][FRAMESIZE];					// Raw Green FIFO data buffers
uint32_t IRFrame[4][FRAMESIZE];					// Raw IR FIFO data buffers
uint32_t RFrame[4][FRAMESIZE];					// Raw Red FIFO data buffers
static int buffIndex;							// next frame buffer destination
static int nFrameIndex;							// Current frame buffer
static int GIRRCollected;
static uint32_t signalGreen;					// Green signal target value 
static uint32_t signalIR;						// IR signal target value 
static uint32_t signalRed;						// Red signal target value 

static uint32_t dcIRMeans[MAX_DC_DELAY];		// IR DC of a frame
static int dcIRIn;								// next dcIRMean index
static int dcIROut;								// next dcIRMean index
static int dcIRCount;							// Number of dcIRMeans collected
uint32_t irDC;									// IR DC to report
static double modIRPeaks[MAX_MOD_DELAY];		// peak modulation for a frame
static int modIRIn;								// next modIRPeak index
double irMod;									// IR modulation to report

static uint32_t dcRedMeans[MAX_DC_DELAY];		// Red DC of a frame
static int dcRedIn;								// next dcRedMean index
static int dcRedOut;							// next dcRedMean index
static int dcRedCount;							// Number of dcRedMeans collected
uint32_t redDC;									// Red DC to report
static double modRedPeaks[MAX_MOD_DELAY];		// peak Red modulation for a frame
static int modRedIn;							// next modRedPeak index
double redMod;									// Red modulation to report

extern uint8_t nLed1Drv;
extern uint8_t nLed2Drv;
extern uint8_t nLed3Drv;
extern uint8_t nLed4Drv;
extern uint8_t nLed5Drv;
extern uint8_t nLed6Drv;
extern uint8_t nLed7Drv;
extern uint8_t nLed8Drv;
extern uint8_t nDCDelay;
extern uint8_t nModDelay;

void InitFrames( void )
{
	buffIndex = 0;
	nFrameIndex = 0;
	GIRRCollected = 0;
	signalGreen = DEF_SIGNAL_TARGET;
	signalRed = DEF_SIGNAL_TARGET;
	signalIR = DEF_SIGNAL_TARGET;

	dcIRIn = 0;
	dcIROut = 0;
	dcIRCount = 0;
	irDC = 0;
	modIRIn = 0;
	irMod = 0.0;
	memset( &dcIRMeans[0], 0, sizeof( dcIRMeans ) );

	dcRedIn = 0;
	dcRedOut = 0;
	dcRedCount = 0;
	redDC = 0;
	modRedIn = 0;
	redMod = 0.0;
	memset( &dcRedMeans[0], 0, sizeof( dcRedMeans ) );

	if( nLed1Drv != 0 )
		max86171_set_green( nLed1Drv );
	else
		max86171_set_green( MAX86171_LED_DRV_DEF );

	if( nLed2Drv != 0 )
		max86171_set_red( nLed2Drv );
	else
		max86171_set_red( MAX86171_LED_DRV_DEF );

	if( nLed3Drv != 0 )
		max86171_set_ir( nLed3Drv );
	else
		max86171_set_ir( MAX86171_LED_DRV_DEF );


	fifoRawIndex = 0;
}

void StartFrames( bool start )
{
	int	i, j;
	
	for( i = 0; i < 4; i++ )
	{
		for( j = 0; j < FRAMESIZE; j++ )			// Clear frame buffers
		{
			GFrame[i][j] = 0;
			IRFrame[i][j] = 0;
			RFrame[i][j] = 0;
		}
	}
	buffIndex = 0;
	nFrameIndex = 0;
	GIRRCollected = 0;
	fifoRawIndex = 0;

	max86171_start( start );
}

int GetFIFOFrame( void )
{
	int x, retVal;
	int i, j;

	x = max86171_get_fifo( buffIndex, nFrameIndex );	// Making an assumption that x will always equal 129...

	if( x == -10 )										// Found some bad data in fifo
	{
		printk( "Bad data collected from max86171 fifo\n" );
		return 0;
	}
	else if( x < 0 )									// Measurement overflow occured
	{
		printk( "Overflow data collected %d from max86171 fifo\n", x );
		AdjustDrives( x );
		max86171_sync();
		return 0;
	}
	else if( x == 0 )
	{
		return 0;
	}

	GIRRCollected += x;
//	printk( "Collected %d from max86171 fifo, total = %d\n", x, GIRRCollected );
	
	if( (GIRRCollected % 258) == 0 )
	{
		dcIRMeans[dcIRIn] = DoAverage( &IRFrame[nFrameIndex][0], FRAMESIZE );		// IR DC of a frame
		modIRPeaks[modIRIn] = ((double)((FindPeak( &IRFrame[nFrameIndex][0], FRAMESIZE ) - dcIRMeans[dcIRIn])) / dcIRMeans[dcIRIn]) * 100;		// peak modulation for a frame
		printf( "IR mean = %u, mod = %f\r\n", dcIRMeans[dcIRIn], modIRPeaks[modIRIn] );
		if( nModDelay == 0 )
		{
			irMod = modIRPeaks[modIRIn];		// IR modulation to report
		}
		else
		{
			if( ++modIRIn == nModDelay )
			{
				modIRIn = 0;
				for( i = 0; i < nModDelay; i++ )
				{
					irMod += modIRPeaks[i];
				}
				--i;
				irMod /= i;
			}
		}

		if( ++dcIRIn == MAX_DC_DELAY )
			dcIRIn = 0;
		if( dcIROut )
		{
			j = dcIROut;
			irDC = 0;
			for( i = 0; i < nDCDelay; i++ )
			{
				irDC += dcIRMeans[j];
				if( ++j == MAX_DC_DELAY )
					j = 0;
			}
			irDC /= i;

			if( ++dcIROut == MAX_DC_DELAY )
				dcIROut = 0;
		}

		if( ++dcIRCount == nDCDelay )
		{
			dcIRCount = 0;
			if( dcIROut == 0 )
			{
				++dcIROut;
			}
		}

		dcRedMeans[dcRedIn] = DoAverage( &RFrame[nFrameIndex][0], FRAMESIZE );		// Red DC of a frame
		modRedPeaks[modRedIn] = ((double)((FindPeak( &RFrame[nFrameIndex][0], FRAMESIZE ) - dcRedMeans[dcRedIn])) / dcRedMeans[dcRedIn]) * 100;		// peak modulation for a frame
		printf( "Red mean = %u, mod = %f\r\n", dcRedMeans[dcRedIn], modRedPeaks[modRedIn] );
		if( nModDelay == 0 )
		{
			redMod = modRedPeaks[modRedIn];		// Red modulation to report
		}
		else
		{
			if( ++modRedIn == nModDelay )
			{
				modRedIn = 0;
				for( i = 0; i < nModDelay; i++ )
				{
					redMod += modRedPeaks[i];
				}
				--i;
				redMod /= i;
			}
		}
		if( ++dcRedIn == MAX_DC_DELAY )
			dcRedIn = 0;
		if( dcRedOut )
		{
			j = dcRedOut;
			redDC = 0;
			for( i = 0; i < nDCDelay; i++ )
			{
				redDC += dcRedMeans[j];
				if( ++j == MAX_DC_DELAY )
					j = 0;
			}
			redDC /= i;

			if( ++dcRedOut == MAX_DC_DELAY )
				dcRedOut = 0;
		}

		if( ++dcRedCount == nDCDelay )
		{
			dcRedCount = 0;
			if( dcRedOut == 0 )
			{
				++dcRedOut;
			}
		}
		retVal = nFrameIndex + 1;
		if( ++nFrameIndex == 4 )
			nFrameIndex = 1;
		buffIndex = 0;
		fifoRawIndex = 0;
		if( dumpType == 'D' )
		{
			fifoRawData[1].fifoType = 'D';
			fifoRawData[1].len = MAX_RAW_DATA - 1;
			k_fifo_put( &fifo_spo2_tx_data, &fifoRawData[1] );
		}
	}
	else
	{
		retVal = nFrameIndex;
		buffIndex = x / 3;
		fifoRawIndex = 1;
		if( dumpType == 'D' )
		{
			fifoRawData[0].fifoType = 'D';
			fifoRawData[0].len = MAX_RAW_DATA - 1;
			k_fifo_put( &fifo_spo2_tx_data, &fifoRawData[0] );
		}
	}

	return retVal;
}

int ServoFrame( void )
{
	uint32_t sizeSignal;
	uint32_t sizeLoTarget;
	uint32_t sizeHiTarget;
	uint32_t driveInc;
	uint8_t sizeDrive;
//	double sizeDiff;
	uint8_t sizeAdjust;
	int retVal = 0;

	if( nLed1Drv != 0 )			// has drives been set via Bluetooth?
	{
		if( nLed1Drv != max86171_get_green() )
		{
			max86171_set_green( nLed1Drv );
			++retVal;
		}
	
		if( nLed2Drv != max86171_get_red() )
		{
			max86171_set_red( nLed2Drv );
			++retVal;
		}
	
		if( nLed3Drv != max86171_get_ir() )
		{
			max86171_set_ir( nLed3Drv );
			++retVal;
		}
		return retVal;			// Servo is not done when any LED drive is set via Bluetooth
	}


	sizeSignal = DoAverage( &IRFrame[0][0], FRAMESIZE );
	sizeSignal += DoAverage( &IRFrame[1][0], FRAMESIZE );
	sizeSignal /= 2;
	sizeLoTarget = (signalIR - (signalIR / 5));
	sizeHiTarget = (signalIR + (signalIR / 5));

	printk( "Servo IR, size = %d Lo = %d, Hi = %d\n", sizeSignal, sizeLoTarget, sizeHiTarget );
	if( sizeSignal > MAX86171_PD_MAX || sizeSignal < MAX86171_PD_MIN )	// Do we have a minimum or maximum signal?
	{
//		return 0x7f;				// return nothing to see BUT...
		return retVal;				// don't force a framecount reset
	}
	
	sizeDrive = max86171_get_ir();
	if( sizeSignal < sizeLoTarget )				// real signal below target?
	{
		driveInc = sizeSignal / sizeDrive;
		if( sizeLoTarget - sizeSignal < driveInc )
		{
			sizeAdjust = MAX86171_LED_DRV_INC;
		}
		else
		{
			sizeAdjust = (sizeLoTarget - sizeSignal) / driveInc;
			if( sizeAdjust & 1 )
				++sizeAdjust;
		}
//		sizeDiff = (double)sizeLoTarget / sizeSignal;
//		sizeAdjust = (uint8_t)((double)sizeDrive * sizeDiff);
//		sizeAdjust = ((((sizeSignal * 100) / sizeLoTarget) - 100) * 2);
		printk( "IR up a = %d\n", sizeAdjust );
		
		if( sizeAdjust > MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MAX )
		{
			if( sizeDrive < MAX86171_LED_DRV_MAX  - sizeAdjust )	// enough head room to use the adjustment?
			{
				sizeDrive += sizeAdjust;
					
				max86171_set_ir( sizeDrive );
				printk( "Small IR, drive = %d\n", sizeDrive );
				++retVal;
			}
			else
			{
				sizeDrive += (MAX86171_LED_DRV_MAX - sizeDrive);
				max86171_set_ir( sizeDrive );
				printk( "Small IR, max drive = %d\n", sizeDrive );
				++retVal;
			}
		}
		else if( sizeDrive < MAX86171_LED_DRV_MAX - MAX86171_LED_DRV_INC )	// at max LED drive level?
		{
			sizeDrive += MAX86171_LED_DRV_INC;

			max86171_set_ir( sizeDrive );
			printk( "Small IR, inc drive = %d\n", sizeDrive );
			++retVal;
		}
		else
		{
//			signalIR = sizeSignal;
			printk( "New < IR, signal = %d\n", sizeSignal );
		}
	}
	else if( sizeSignal > sizeHiTarget )				// real signal above target?
	{
		driveInc = sizeSignal / sizeDrive;
		if( sizeSignal - sizeHiTarget < driveInc )
		{
			sizeAdjust = MAX86171_LED_DRV_INC;
		}
		else
		{
			sizeAdjust = (sizeSignal - sizeHiTarget) / driveInc;
			if( sizeAdjust & 1 )
				++sizeAdjust;
		}
//		sizeDiff = sizeHiTarget / sizeSignal;
//		sizeAdjust = MAX86171_LED_DRV_MAX - (uint8_t)((double)sizeDrive * sizeDiff);
//		sizeAdjust = MAX86171_LED_DRV_MAX - ((((sizeSignal * 100) / sizeHiTarget) - 100) * 2);
		printk( "IR down a = %d\n", sizeAdjust );

		if( sizeAdjust > MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MIN )
		{
			if( sizeDrive > MAX86171_LED_DRV_MIN + sizeAdjust )		// at minimum LED drive level?
			{
				sizeDrive -= sizeAdjust;
				max86171_set_ir( sizeDrive );
				printk( "Big IR, drive = %d\n", sizeDrive );
				++retVal;
			}
			else
			{
				sizeDrive -= (sizeDrive - MAX86171_LED_DRV_MIN);
				max86171_set_ir( sizeDrive );
				printk( "Big IR, min drive = %d\n", sizeDrive );
				++retVal;
			}
		}
		else if( sizeDrive > MAX86171_LED_DRV_MIN - MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive -= MAX86171_LED_DRV_INC;
				
			max86171_set_ir( sizeDrive );
			printk( "Big IR, inc drive = %d\n", sizeDrive );
			++retVal;
		}
		else
		{
			signalIR = sizeSignal;
			printk( "New > IR, signal = %d\n", sizeSignal );
		}
	}

	sizeSignal = DoAverage( &RFrame[0][0], FRAMESIZE );
	sizeSignal += DoAverage( &RFrame[1][0], FRAMESIZE );
	sizeSignal /= 2;
	sizeLoTarget = (signalRed - (signalRed / 5));
	sizeHiTarget = (signalRed + (signalRed / 5));

	printk( "Servo Red, size = %d Lo = %d, Hi = %d\n", sizeSignal, sizeLoTarget, sizeHiTarget );

	sizeDrive = max86171_get_red();
	if( sizeSignal < sizeLoTarget )				// real signal below target?
	{
		driveInc = sizeSignal / sizeDrive;
		if( sizeLoTarget - sizeSignal < driveInc )
		{
			sizeAdjust = MAX86171_LED_DRV_INC;
		}
		else
		{
			sizeAdjust = (sizeLoTarget - sizeSignal) / driveInc;
			if( sizeAdjust & 1 )
				++sizeAdjust;
		}
//		sizeDiff = sizeLoTarget / sizeSignal;
//		sizeAdjust = (uint8_t)((double)sizeDrive * sizeDiff);
//		sizeAdjust = ((((sizeSignal * 100) / sizeHiTarget) - 100) * 2);
		printk( "RED up a = %d\n", sizeAdjust );

		if( sizeAdjust > MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MAX )
		{
			if( sizeDrive < MAX86171_LED_DRV_MAX  - sizeAdjust )	// enough head room to use the adjustment?
			{
				sizeDrive += sizeAdjust;
					
				max86171_set_red( sizeDrive );
				printk( "Small RED, drive = %d\n", sizeDrive );
				++retVal;
			}
			else
			{
				sizeDrive += (MAX86171_LED_DRV_MAX - sizeDrive);
				max86171_set_red( sizeDrive );
				printk( "Small RED, max drive = %d\n", sizeDrive );
				++retVal;
			}
		}
		else if( sizeDrive < MAX86171_LED_DRV_MAX - MAX86171_LED_DRV_INC )	// at max LED drive level?
		{
			sizeDrive += MAX86171_LED_DRV_INC;

			max86171_set_red( sizeDrive );
			printk( "Small Red, inc drive = %d\n", sizeDrive );
			++retVal;
		}
		else
		{
//			signalRed = sizeSignal;
			printk( "New < Red, signal = %d\n", sizeSignal );
		}
	}
	else if( sizeSignal > sizeHiTarget )				// real signal above target?
	{
		driveInc = sizeSignal / sizeDrive;
		if( sizeSignal - sizeHiTarget < driveInc )
		{
			sizeAdjust = MAX86171_LED_DRV_INC;
		}
		else
		{
			sizeAdjust = (sizeSignal - sizeHiTarget) / driveInc;
			if( sizeAdjust & 1 )
				++sizeAdjust;
		}
//		sizeDiff = sizeHiTarget / sizeSignal;
//		sizeAdjust = MAX86171_LED_DRV_MAX - (uint8_t)((double)sizeDrive * sizeDiff);
//		sizeAdjust = MAX86171_LED_DRV_MAX - ((((sizeSignal * 100) / sizeHiTarget) - 100) * 2);
		printk( "RED down a = %d\n", sizeAdjust );

		if( sizeAdjust > MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MIN )
		{
			if( sizeDrive > MAX86171_LED_DRV_MIN + sizeAdjust )		// at minimum LED drive level?
			{
				sizeDrive -= sizeAdjust;
					
				max86171_set_red( sizeDrive );
				printk( "Big Red, drive = %d\n", sizeDrive );
				++retVal;
			}
			else
			{
				sizeDrive -= (sizeDrive - MAX86171_LED_DRV_MIN);
				max86171_set_red( sizeDrive );
				printk( "Big Red, min drive = %d\n", sizeDrive );
				++retVal;
			}
		}
		else if( sizeDrive > MAX86171_LED_DRV_MIN  - MAX86171_LED_DRV_INC && sizeDrive != MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive -= MAX86171_LED_DRV_INC;
				
			max86171_set_red( sizeDrive );
			printk( "Big Red, inc drive = %d\n", sizeDrive );
			++retVal;
		}
		else
		{
//			signalRed = sizeSignal;
			printk( "New > Red, signal = %d\n", sizeSignal );
		}
	}

	return retVal;
}

static uint32_t	DoAverage( uint32_t *src, int size )
{
	int i;
	uint32_t retVal = 0;
	
	for( i = 0; i < size; i++ )
	{
		retVal += *src++;
	}
	return( retVal / size );
}

static uint32_t	FindPeak( uint32_t *src, int size )
{
	int i;
	uint32_t retVal = 0;
	
	for( i = 0; i < size; i++ )
	{
		if( *src > retVal )
			retVal = *src;
		++src;
	}

	return retVal;
}

static void AdjustDrives( int whichOnes )
{
	uint8_t sizeDrive;

	printk( "Overflow occured, measurements = %d\n", whichOnes );
	switch( whichOnes )
	{
	case -1:		// Just Green
		sizeDrive = max86171_get_green();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_green( sizeDrive );
			printk( "Just Green, overdrive = %d\n", sizeDrive );
		}
		break;

	case -2:		// Just IR
		sizeDrive = max86171_get_ir();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_ir( sizeDrive );
			printk( "Just IR, overdrive = %d\n", sizeDrive );
		}
		break;

	case -3:		// Just Red
		sizeDrive = max86171_get_red();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_red( sizeDrive );
			printk( "Just Red, overdrive = %d\n", sizeDrive );
		}
		break;

	case -4:		// Green and IR
		sizeDrive = max86171_get_green();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_green( sizeDrive );
			printk( "Green and, overdrive = %d\n", sizeDrive );
		}
		sizeDrive = max86171_get_ir();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_ir( sizeDrive );
			printk( "IR, overdrive = %d\n", sizeDrive );
		}
		break;

	case -5:		// Green and Red
		sizeDrive = max86171_get_green();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_green( sizeDrive );
			printk( "Green and, overdrive = %d\n", sizeDrive );
		}
		sizeDrive = max86171_get_red();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binay search for the best level
			max86171_set_red( sizeDrive );
			printk( "RED, overdrive = %d\n", sizeDrive );
		}
		break;

	case -6:		// IR and Red
		sizeDrive = max86171_get_ir();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binary search for the best level
			max86171_set_ir( sizeDrive );
			printk( "IR and, overdrive = %d\n", sizeDrive );
		}
		sizeDrive = max86171_get_red();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binary search for the best level
			max86171_set_red( sizeDrive );
			printk( "RED, overdrive = %d\n", sizeDrive );
		}
		break;

	case -7:		// Green, IR and Red
		sizeDrive = max86171_get_green();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binary search for the best level
			max86171_set_green( sizeDrive );
			printk( "Green and, overdrive = %d\n", sizeDrive );
		}
		sizeDrive = max86171_get_ir();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binary search for the best level
			max86171_set_ir( sizeDrive );
			printk( "IR and, overdrive = %d\n", sizeDrive );
		}
		sizeDrive = max86171_get_red();
		if( sizeDrive > MAX86171_LED_DRV_MIN )		// at minimum LED drive level?
		{
			sizeDrive /= 2;							// We're going to binary search for the best level
			max86171_set_red( sizeDrive );
			printk( "RED, overdrive = %d\n", sizeDrive );
		}
		break;

	default:
	}
}

