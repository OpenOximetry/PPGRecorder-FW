/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <zephyr/bluetooth/hci.h>
#include <sdc_hci_vs.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>

#include "DataFrame.h"
#include "battery.h"

#define STACKSIZE 2048
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	( sizeof( DEVICE_NAME ) - 1 )

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define CHRG_CTL_NODE DT_NODELABEL( chrgctl )
#if !DT_NODE_HAS_STATUS(CHRG_CTL_NODE, okay)
#error "Unsupported board: CHRG_CTL_NODE devicetree alias is not defined"
#endif
static const struct gpio_dt_spec g_devChrgCtl = GPIO_DT_SPEC_GET_OR( CHRG_CTL_NODE, gpios, {0} );

#define USB_CON_NODE DT_NODELABEL( usbcon )
#if !DT_NODE_HAS_STATUS(USB_CON_NODE, okay)
#error "Unsupported board: USB_CON_NODE devicetree alias is not defined"
#endif
static const struct gpio_dt_spec g_devUSBCon = GPIO_DT_SPEC_GET_OR( USB_CON_NODE, gpios, {0} );

#define TP8_NODE DT_NODELABEL( testpt8 )
#if !DT_NODE_HAS_STATUS(TP8_NODE, okay)
#error "Unsupported board: TP8_NODE devicetree alias is not defined"
#endif
const struct gpio_dt_spec g_devTestPt8 = GPIO_DT_SPEC_GET_OR( TP8_NODE, gpios, {0} );

#define TP9_NODE DT_NODELABEL( testpt9 )
#if !DT_NODE_HAS_STATUS(TP9_NODE, okay)
#error "Unsupported board: TP9_NODE devicetree alias is not defined"
#endif
const struct gpio_dt_spec g_devTestPt9 = GPIO_DT_SPEC_GET_OR( TP9_NODE, gpios, {0} );

#define TP10_NODE DT_NODELABEL( testpt10 )
#if !DT_NODE_HAS_STATUS(TP10_NODE, okay)
#error "Unsupported board: TP10_NODE devicetree alias is not defined"
#endif
const struct gpio_dt_spec g_devTestPt10 = GPIO_DT_SPEC_GET_OR( TP10_NODE, gpios, {0} );

#define TP11_NODE DT_NODELABEL( testpt11 )
#if !DT_NODE_HAS_STATUS(TP11_NODE, okay)
#error "Unsupported board: TP11_NODE devicetree alias is not defined"
#endif
const struct gpio_dt_spec g_devTestPt11 = GPIO_DT_SPEC_GET_OR( TP11_NODE, gpios, {0} );

static K_SEM_DEFINE( ble_init_ok, 0, 1 );
K_FIFO_DEFINE( fifo_spo2_tx_data );
K_EVENT_DEFINE( max_event );

struct bt_conn *current_conn;

/* Pleth Recorder Custom Service Variables */
#define BT_UUID_PREC_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x5c190000, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba)

static struct bt_uuid_128 prec_uuid = BT_UUID_INIT_128( BT_UUID_PREC_SERVICE_VAL );

static ssize_t read_uint8( struct bt_conn *conn,
						   const struct bt_gatt_attr *attr,
						   void *buf,
						   uint16_t len,
						   uint16_t offset )
{
	uint8_t *value = attr->user_data;

	printk( "Read uint8 0x%X %d\n", *value, offset + len );
	return bt_gatt_attr_read( conn,
							  attr,
							  buf,
							  len,
							  offset,
							  value,
							  sizeof( uint8_t ) );
}

static ssize_t write_uint8( struct bt_conn *conn,
							const struct bt_gatt_attr *attr,
							const void *buf,
							uint16_t len,
							uint16_t offset,
							uint8_t flags )
{
	uint8_t *value = attr->user_data;

	printk( "Write uint8 0x%X %d\n", *((uint8_t*)buf), len );
	if( offset + len > sizeof( uint8_t ) )
	{
		return BT_GATT_ERR( BT_ATT_ERR_INVALID_OFFSET );
	}

	*value = *((uint8_t *)buf);

	return len;
}

static ssize_t read_uint16( struct bt_conn *conn,
							const struct bt_gatt_attr *attr,
							void *buf,
							uint16_t len,
							uint16_t offset )
{
	uint16_t *value = attr->user_data;

	printk( "Read uint16 0x%X %d\n", *value, len );
	return bt_gatt_attr_read( conn,
							  attr,
							  buf,
							  len,
							  offset,
							  value,
							  sizeof( uint16_t ) );
}

const struct bt_uuid_128 sdi_PREC_battery = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a001, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint16_t nBattery;

static const struct bt_uuid_128 sdi_PREC_led1Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a002, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed1Drv;

static const struct bt_uuid_128 sdi_PREC_led2Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a003, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed2Drv;

static const struct bt_uuid_128 sdi_PREC_led3Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a004, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed3Drv;

static const struct bt_uuid_128 sdi_PREC_led4Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a005, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed4Drv;

static const struct bt_uuid_128 sdi_PREC_led5Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a006, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed5Drv;

static const struct bt_uuid_128 sdi_PREC_led6Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a007, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed6Drv;

static const struct bt_uuid_128 sdi_PREC_led7Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a008, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed7Drv;

static const struct bt_uuid_128 sdi_PREC_led8Drv = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a009, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nLed8Drv;

static const struct bt_uuid_128 sdi_PREC_dcDelay = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a00a, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nDCDelay = 10;

static const struct bt_uuid_128 sdi_PREC_modDelay = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x5c19a00b, 0xd5d7, 0x4412, 0xa26c, 0xfacca0784cba));

uint8_t nModDelay = 0;

/* PPG Recorder Primary Service Declaration */
BT_GATT_SERVICE_DEFINE( prec_svc,										// Macro defined in gatt.h
	BT_GATT_PRIMARY_SERVICE( &prec_uuid ),

	BT_GATT_CHARACTERISTIC( &sdi_PREC_battery.uuid,
							BT_GATT_CHRC_READ,
							BT_GATT_PERM_READ,
							read_uint16,
							NULL,
							&nBattery ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led1Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed1Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led2Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed2Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led3Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed3Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led4Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed4Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led5Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed5Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led6Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed6Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led7Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed7Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_led8Drv.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nLed8Drv ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_dcDelay.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nDCDelay ),
	BT_GATT_CHARACTERISTIC( &sdi_PREC_modDelay.uuid,
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
							read_uint8,
							write_uint8,
							&nModDelay ),
);

/* Nordic BLE UART services definitions */
static const struct device *uart = DEVICE_DT_GET( DT_CHOSEN( nordic_nus_uart ) );
static struct k_work_delayable uart_work;

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

bool uart_dump;
#define START_UART		0x11			// Ctrl-Q
#define STOP_UART		0x13			// Ctrl-S
char tx_buff[2048];						// UART transmit buffer

static const struct bt_data ad[] =
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] =
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

void set_charger( bool enable )
{
	int err;
	
	err = gpio_pin_set_dt( &g_devChrgCtl, (int)enable );		// set pin to match enable flag
	if( err < 0 )
	{
		printk( "Error %d occured setting charger to %d\n", err, (int)enable );
	}
	return;
}

bool fUSBPwrState;

bool check_usb( void )
{
	return fUSBPwrState;
}

extern uint8_t dumpType;

static const struct device *const async_adapter;

static void uart_cb( const struct device *dev, struct uart_event *evt, void *user_data )
{
	ARG_UNUSED( dev );

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch( evt->type )
	{
	case UART_TX_DONE:
		printk( "UART_TX_DONE\n" );
		if( (evt->data.tx.len == 0) || (!evt->data.tx.buf) || (evt->data.tx.buf == (const uint8_t *)&tx_buff) )
		{
			return;
		}

		if( aborted_buf )
		{
			buf = CONTAINER_OF( aborted_buf,
								struct uart_data_t,
								data );
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF( evt->data.tx.buf,
								struct uart_data_t,
								data );
		}

		k_free( buf );
		break;

	case UART_RX_RDY:
		printk( "UART_RX_RDY\n" );

		if( evt->data.rx.len > 0 )
		{
//			Parse data received
			if( evt->data.rx.len == 1 && evt->data.rx.buf[evt->data.rx.offset] == START_UART )
			{
				set_charger( true );
				uart_dump = true;
				dumpType = 'D';
			}
			else if( evt->data.rx.len == 1 && evt->data.rx.buf[evt->data.rx.offset] == STOP_UART )
			{
				uart_dump = false;
				set_charger( false );
				dumpType = 'X';
			}
			else
				printk( "Received %s from UART\n", &evt->data.rx.buf[0] );
		}

		break;

	case UART_RX_DISABLED:
		printk( "UART_RX_DISABLED\n" );
		disable_req = false;

		buf = k_malloc( sizeof( *buf ) );
		if( buf )
		{
			buf->len = 0;
		}
		else
		{
			printk("Not able to allocate UART receive buffer\n");
			k_work_reschedule( &uart_work, UART_WAIT_FOR_BUF_DELAY );
			return;
		}

		uart_rx_enable( uart, buf->data,
						sizeof( buf->data ),
						UART_WAIT_FOR_RX );
		break;

	case UART_RX_BUF_REQUEST:
		printk( "UART_RX_BUF_REQUEST\n" );
		buf = k_malloc( sizeof( *buf ) );
		if( buf )
		{
			buf->len = 0;
			uart_rx_buf_rsp( uart, buf->data, sizeof( buf->data ) );
		}
		else
		{
			printk( "Not able to allocate UART receive buffer\n" );
		}
		break;

	case UART_RX_BUF_RELEASED:
		printk( "UART_RX_BUF_RELEASED\n" );
		buf = CONTAINER_OF( evt->data.rx_buf.buf,
							struct uart_data_t,
							data );

		k_free( buf );

		break;

	case UART_TX_ABORTED:
		printk( "UART_TX_ABORTED\n" );
		if( !aborted_buf )
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF( aborted_buf,
							struct uart_data_t,
							data );

		uart_tx( uart,
				 &buf->data[aborted_len],
				 buf->len - aborted_len,
				 SYS_FOREVER_MS );

		break;

	default:
		break;
	}
}

static void uart_work_handler( struct k_work *item )
{
	struct uart_data_t *buf;

	buf = k_malloc( sizeof( *buf ) );
	if( buf )
	{
		buf->len = 0;
	}
	else
	{
		printk( "Not able to allocate UART receive buffer\n" );
		k_work_reschedule( &uart_work, UART_WAIT_FOR_BUF_DELAY );
		return;
	}

	uart_rx_enable( uart,
					buf->data,
					sizeof( buf->data ),
					UART_WAIT_FOR_RX );
}

static bool uart_test_async_api( const struct device *dev )
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return( api->callback_set != NULL );
}

int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if( !device_is_ready( uart ) )
	{
		return -ENODEV;
	}

	rx = k_malloc( sizeof( *rx ) );
	if( rx )
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable( &uart_work, uart_work_handler );

	if( IS_ENABLED( CONFIG_BT_NUS_UART_ASYNC_ADAPTER ) && !uart_test_async_api( uart ) )
	{
		/* Implement API adapter */
		uart_async_adapter_init( async_adapter, uart );
		uart = async_adapter;
	}

	err = uart_callback_set( uart, uart_cb, NULL );
	if( err )
	{
		k_free( rx );
		printk( "Cannot initialize UART callback\n" );
		return err;
	}

	if( IS_ENABLED( CONFIG_UART_LINE_CTRL ) )
	{
		printk( "Wait for DTR\n" );
		while( true )
		{
			uint32_t dtr = 0;

			uart_line_ctrl_get( uart, UART_LINE_CTRL_DTR, &dtr );
			if( dtr )
			{
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep( K_MSEC( 100 ) );
		}
		printk( "DTR set\n" );
		err = uart_line_ctrl_set( uart, UART_LINE_CTRL_DCD, 1 );
		if( err )
		{
			printk( "Failed to set DCD, ret code %d\n", err );
		}
		err = uart_line_ctrl_set( uart, UART_LINE_CTRL_DSR, 1 );
		if( err )
		{
			printk( "Failed to set DSR, ret code %d\n", err );
		}
	}

	tx = k_malloc( sizeof( *tx ) );

	if( tx )
	{
		pos = snprintf( tx->data,
						sizeof( tx->data ),
						"SDI PPG Recorder V2.00\r\n");

		if( (pos < 0) || (pos >= sizeof( tx->data )) )
		{
			k_free(rx);
			k_free(tx);
			printk( "snprintf returned %d\n", pos );
			return -ENOMEM;
		}

		tx->len = pos;
		err = uart_tx( uart,
					tx->data,
					tx->len,
					SYS_FOREVER_MS );
		if( err )
		{
			k_free( rx );
			k_free( tx );
			printk( "Cannot display welcome message (err: %d)\n", err );
			return err;
		}
	}
	else
	{
		k_free( rx );
		return -ENOMEM;
	}

	err = uart_rx_enable( uart,
						  rx->data,
						  sizeof( rx->data ),
						  50 );
	if( err )
	{
		printk( "Cannot enable uart reception (err: %d)\n", err );
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

extern uint32_t dumpSize;

static void exchange_func( struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params )
{
	if( !err )
	{
		printk( "MTU exchange done\n" );
		dumpSize = bt_nus_get_mtu( current_conn );
		printk( "MTU size = %d\n", dumpSize );
	}
	else
	{
		printk( "MTU exchange failed (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if( err )
	{
		printk( "Connection failed (err %u)\n", err );
		return;
	}

	bt_addr_le_to_str( bt_conn_get_dst( conn ), addr, sizeof( addr ) );
	printk( "Connected %s\n", addr );

	current_conn = bt_conn_ref( conn );

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu( conn, &exchange_params );
	if( err )
	{
		printk( "MTU exchange failed (err %d)\n", err );
	}
	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str( bt_conn_get_dst( conn ), addr, sizeof( addr ) );

	printk( "Disconnected: %s (reason %u)\n", addr, reason );

	if( current_conn )
	{
		bt_conn_unref( current_conn );
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb( struct bt_conn *conn,
						   const uint8_t *const data,
						   uint16_t len )
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str( bt_conn_get_dst(conn), addr, ARRAY_SIZE( addr ) );

	if( len == 1 )
		printk("Received %c command from: %s\n", *data, addr);
	else
		printk("Received %d bytes from: %s\n", len, addr);
	
	dumpType = *data;

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

static int set_bd_addr(void)
{
	int err = 0;
	struct net_buf *buf;
	sdc_hci_cmd_vs_zephyr_write_bd_addr_t *cmd_params;

	buf = bt_hci_cmd_create( SDC_HCI_OPCODE_CMD_VS_ZEPHYR_WRITE_BD_ADDR, sizeof( *cmd_params ) );
	if( !buf )
	{
		printk( "Could not allocate command buffer\n" );
		return -ENOMEM;
	}

	cmd_params = net_buf_add( buf, sizeof( *cmd_params ) );

	cmd_params->bd_addr[0]= 0xA0;
	cmd_params->bd_addr[1]= 0x0F;
	cmd_params->bd_addr[2]= 0x00;		// == 4000
	cmd_params->bd_addr[3]= 0x55;
	cmd_params->bd_addr[4]= 0xAA;
	cmd_params->bd_addr[5]= 0x55;



    printk( "bt_hci_cmd_send_sync \n" );

	err = bt_hci_cmd_send_sync( SDC_HCI_OPCODE_CMD_VS_ZEPHYR_WRITE_BD_ADDR, buf, NULL );
	printk( "err: %d \n", err );
	if( err )
	{
		printk( "err: %d \n", err );
		return err;
	}

	printk( "Successfully set bd addr \n" );

	return 0;
}

static void usbcon_callback( const struct device *gpiob, struct gpio_callback *cb, uint32_t pins )
{
	if( gpio_pin_get_raw( g_devUSBCon.port, g_devUSBCon.pin ) )
		fUSBPwrState = true;
	else
		fUSBPwrState = false;
}

static struct gpio_callback usbon_cb;

bool USBConInit( void )
{
int ret;

	if( !device_is_ready( g_devUSBCon.port ) )
	{
		printk( "Error: USB Power port %s is not ready\n", g_devUSBCon.port->name );
		return false;
	}

	ret = gpio_pin_configure_dt( &g_devUSBCon, GPIO_INPUT );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devUSBCon.port->name, g_devUSBCon.pin);
		return false;
	}


	ret =  gpio_pin_interrupt_configure_dt( &g_devUSBCon, GPIO_INT_EDGE_BOTH );
	if( ret != 0 )
	{
		printk( "Error %d: failed to configure interrupt on %s pin %d\n",
				ret, g_devUSBCon.port->name, g_devUSBCon.pin );
		return false;
	}

	gpio_init_callback( &usbon_cb, usbcon_callback, BIT( g_devUSBCon.pin ) );
	gpio_add_callback( g_devUSBCon.port, &usbon_cb );

	printk("Set up USB On at %s pin %d\n", g_devUSBCon.port->name, g_devUSBCon.pin);
	if( gpio_pin_get_raw( g_devUSBCon.port, g_devUSBCon.pin ) )
	{
		fUSBPwrState = true;
	}
	else
	{
		fUSBPwrState = false;
	}

	return true;
}

static int GpioInit( void )
{
int ret;

	if( !device_is_ready( g_devChrgCtl.port ) )
	{
		printk( "Error: Charge Control pin %s is not ready\n", g_devChrgCtl.port->name );
		return -1;
	}

	ret = gpio_pin_configure_dt( &g_devChrgCtl, GPIO_OUTPUT_INACTIVE );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devChrgCtl.port->name, g_devChrgCtl.pin);
		return ret;
	}

	printk("Set up Charge Control at %s pin %d\n", g_devChrgCtl.port->name, g_devChrgCtl.pin);

	if( !device_is_ready( g_devTestPt8.port ) )
	{
		printk( "Error: Test Point 8 pin %s is not ready\n", g_devTestPt8.port->name );
		return -1;
	}

	ret = gpio_pin_configure_dt( &g_devTestPt8, GPIO_OUTPUT_INACTIVE );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devTestPt8.port->name, g_devTestPt8.pin);
		return ret;
	}

	printk("Set up Test Point 8 at %s pin %d\n", g_devTestPt8.port->name, g_devTestPt8.pin);

	if( !device_is_ready( g_devTestPt9.port ) )
	{
		printk( "Error: Test Point 9 pin %s is not ready\n", g_devTestPt9.port->name );
		return -1;
	}

	ret = gpio_pin_configure_dt( &g_devTestPt9, GPIO_OUTPUT_INACTIVE );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devTestPt9.port->name, g_devTestPt9.pin);
		return ret;
	}

	printk("Set up Test Point 9 at %s pin %d\n", g_devTestPt9.port->name, g_devTestPt9.pin);

	if( !device_is_ready( g_devTestPt10.port ) )
	{
		printk( "Error: Test Point 10 pin %s is not ready\n", g_devTestPt10.port->name );
		return -1;
	}

	ret = gpio_pin_configure_dt( &g_devTestPt10, GPIO_OUTPUT_INACTIVE );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devTestPt10.port->name, g_devTestPt10.pin);
		return ret;
	}

	printk("Set up Test Point 10 at %s pin %d\n", g_devTestPt10.port->name, g_devTestPt10.pin);

	if( !device_is_ready( g_devTestPt11.port ) )
	{
		printk( "Error: Test Point 11 pin %s is not ready\n", g_devTestPt11.port->name );
		return -1;
	}

	ret = gpio_pin_configure_dt( &g_devTestPt11, GPIO_OUTPUT_INACTIVE );
	if( ret != 0 )
	{
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, g_devTestPt11.port->name, g_devTestPt11.pin);
		return ret;
	}

	printk("Set up Test Point 11 at %s pin %d\n", g_devTestPt11.port->name, g_devTestPt11.pin);

	USBConInit();

	return 0;
}

void error( void )
{
	while( true )
	{
		/* Spin for ever */
		k_sleep( K_MSEC( 1000 ) );
	}
}

extern void PPGRecorder( void );

void main(void)
{
	int err = 0;
	int batt_mV;

	uart_dump = false;
	dumpType = 'X';				// Start by sending nothing

	err = GpioInit();
	if( err )
	{
		error();
	}

	printk( "GPIO initialized.\n" );

	batt_mV = battery_sample();
	printk( "battery voltage = %d\n", batt_mV );

	nLed1Drv = nLed2Drv = nLed3Drv = nLed4Drv = nLed5Drv = nLed6Drv = nLed7Drv = nLed8Drv = 0;

	err = bt_enable( NULL );
	if( err )
	{
		error();
	}

	set_bd_addr();

	if( IS_ENABLED( CONFIG_SETTINGS ) )
	{
		settings_load();
	}

	err = bt_nus_init( &nus_cb );
	if( err )
	{
		printk("Failed to initialize UART service (err: %d)\n", err);
		return;
	}

	err = bt_le_adv_start( BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE( sd ) );
	if( err )
	{
		printk( "Advertising failed to start (err %d)\n", err );
		return;
	}

	printk( "Bluetooth initialized.\n" );

	k_sem_give( &ble_init_ok );
	PPGRecorder();
}

extern void format_uart_dump( FIFO_RAW_DATA *buf, char *dest_buf, int dest_len );

void ble_write_thread( void )
{
	int err;
	FIFO_DATA *buf;

	/* Don't go any further until BLE is initialized */
	k_sem_take( &ble_init_ok, K_FOREVER );

	for( ;; )
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		buf = k_fifo_get( &fifo_spo2_tx_data, K_FOREVER );

		if( dumpType == 'X' )
		{
			continue;				// Ignore buffers until app say to send them
		}
		else if( buf->fifoType == 'D' )
		{
			FIFO_RAW_DATA *dbuf = (FIFO_RAW_DATA *)buf;

			if( uart_dump == false )
			{
				if( dbuf->len != MAX_RAW_DATA - 1 )
					printk( "Attempt to send %d BOGUS raw bytes over BLE connection\n", dbuf->len );
				else
				{
//					printk( "Sending %d raw bytes over BLE connection\n", buf->len );
					err = bt_nus_send( NULL, (uint8_t *)&dbuf->data, dbuf->len );
					if( err )
					{
//						printk( "Failed to send data over BLE connection err = %d\n", err );
					}
				}
			}
			else
			{
//				format_uart_raw( dbuf, tx_buff1, sizeof( tx_buff1 ) );
//				uart_tx( uart, tx_buff1, (strlen( tx_buff1 ) / 2), SYS_FOREVER_MS );
				format_uart_dump( dbuf, tx_buff, sizeof( tx_buff ) );
				uart_tx( uart, tx_buff, strlen( tx_buff ), SYS_FOREVER_MS );
			}
		}
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

