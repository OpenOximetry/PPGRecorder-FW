/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
//#include <zephyr/logging/log.h>

#include "battery.h"

//LOG_MODULE_REGISTER(BATTERY, CONFIG_ADC_LOG_LEVEL);

#define VBATT DT_PATH( vbatt )
#define BATTERY_ADC_GAIN ADC_GAIN_1

struct io_channel_config {
	uint8_t channel;
};

struct divider_config {
	struct io_channel_config io_channel;
	/* output_ohm is used as a flag value: if it is nonzero then
	 * the battery is measured through a voltage divider;
	 * otherwise it is assumed to be directly connected to Vdd.
	 */
	uint32_t output_ohm;
	uint32_t full_ohm;
};

static const struct divider_config b_divider_config =
{
#if DT_NODE_HAS_STATUS( VBATT, okay )
	.io_channel =
	{
		DT_IO_CHANNELS_INPUT( VBATT ),
	},
	.output_ohm = DT_PROP( VBATT, output_ohms ),
	.full_ohm = DT_PROP( VBATT, full_ohms ),
#endif /* /vbatt exists */
};

struct divider_data
{
	const struct device *adc;
	struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	int16_t raw;
};

static struct divider_data b_divider_data =
{
	.adc = DEVICE_DT_GET( DT_IO_CHANNELS_CTLR( VBATT ) ),
};

static int b_divider_setup( void )
{
	const struct divider_config *cfg = &b_divider_config;
	const struct io_channel_config *iocp = &cfg->io_channel;
	struct divider_data *ddp = &b_divider_data;
	struct adc_sequence *asp = &ddp->adc_seq;
	struct adc_channel_cfg *accp = &ddp->adc_cfg;
	int rc;

	if( !device_is_ready( ddp->adc ) )
	{
		printk( "Battery ADC device is not ready %s", ddp->adc->name );
		return -ENOENT;
	}

	*asp = ( struct adc_sequence )
	{
		.channels = BIT(0),
		.buffer = &ddp->raw,
		.buffer_size = sizeof( ddp->raw ),
		.oversampling = 4,
		.calibrate = true,
	};

	*accp = ( struct adc_channel_cfg )
	{
		.gain = BATTERY_ADC_GAIN,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	};

	if( cfg->output_ohm != 0 )
	{
		accp->input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + iocp->channel;
	}
	else
	{
		accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
	}

	asp->resolution = 14;

	rc = adc_channel_setup( ddp->adc, accp );
	printk( "Setup AIN%u got %d\n", iocp->channel, rc );

	return rc;

}

static bool battery_ok;

static int battery_setup( const struct device *arg )
{
	int rc = b_divider_setup();

	battery_ok = (rc == 0);
	printk("Battery setup: %d %d\n", rc, battery_ok);
	return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_sample( void )
{
	int rc = -ENOENT;

	if( battery_ok )
	{
		struct divider_data *ddp = &b_divider_data;
		const struct divider_config *dcp = &b_divider_config;
		struct adc_sequence *sp = &ddp->adc_seq;

		rc = adc_read( ddp->adc, sp );
		sp->calibrate = false;
		if( rc == 0 )
		{
			int32_t val = ddp->raw;

			adc_raw_to_millivolts( adc_ref_internal(ddp->adc),
								   ddp->adc_cfg.gain,
								   sp->resolution,
								   &val );

			if( dcp->output_ohm != 0 )
			{
				rc = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
//				printk("raw %u ~ %u mV => %d mV\n", ddp->raw, val, rc);
			}
			else
			{
				rc = val;
//				printk("raw %u ~ %u mV\n", ddp->raw, val);
			}
		}
	}

	return rc;
}

