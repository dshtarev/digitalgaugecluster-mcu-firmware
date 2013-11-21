#include "uarthost.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "task.h"
#include "periph.h"
#include "kwp2000.h"
#include "params.h"
#include <stdlib.h>
#include "adc.h"
#include "rc5decoder.h"

#define USART1_TX_MASK 0x1F
#define USART1_TX_BUF_SIZE 16
#define USART1_RX_BUF_SIZE 16

static uint8_t cmd_received_flag = 0;

static char usart1_rx_buf[USART1_RX_BUF_SIZE];
static uint8_t usart1_rx_len = 0;

static char usart1_tx_buf[USART1_TX_BUF_SIZE];
static uint8_t usart1_tx_pos = 0;
static uint8_t usart1_tx_len = 0;

struct in_t
{
	uint8_t value: 1;
	uint32_t freq: 1;
	uint16_t pulse_length;	
};


void uarthost_init()
{

	UCSR1A = _BV( U2X1 );
	UCSR1B |= _BV( RXCIE1 ) /*| _BV( TXCIE1 )*/ | _BV( RXEN1) | _BV( TXEN1 );
	UCSR1C |= _BV( UCSZ10) | _BV( UCSZ11 );
	UBRR1H = 0;
	UBRR1L = 16;

	usart1_tx_buf[0] = 0;

}

void usart1_send(const char * _ptr, uint8_t len)
{

	uint8_t send_f = (usart1_tx_pos == usart1_tx_len);

	uint8_t i;
	for(i=0;i<len;i++)
	{
		usart1_tx_buf[usart1_tx_len] = _ptr[i];
		usart1_tx_len ++;
		usart1_tx_len &= USART1_TX_MASK;
	}
		

	if( send_f)
		UDR1 = usart1_tx_buf[usart1_tx_pos];

}


void usart1_send_char_bl(char ch)
{	
	while( !(UCSR1A & _BV(UDRE1)) );
	UDR1 = ch;
}

void usart1_send_uint8(uint8_t ch)
{
	while( !(UCSR1A & _BV(UDRE1)) );
	UDR1 = ch;	
}

void usart1_send_uint16(uint16_t ch )
{
	usart1_send_uint8( ch & 0xFF );	
	usart1_send_uint8( ch >> 8 );	
}

void usart1_send_uint32(uint32_t ch )
{
	usart1_send_uint8( ch & 0xFF );
	usart1_send_uint8( (ch>>8) & 0xFF );
	usart1_send_uint8( (ch>>16) & 0xFF );
	usart1_send_uint8( (ch>>24) & 0xFF );
}


void usart1_send_str_bl(const char * str)
{
	uint8_t i = 0;
	while( str[i] )
	{
		usart1_send_char_bl( str[i] );
		i++;
	}

}

/*SIGNAL( USART1_TX_vect )
{
	if( usart1_tx_pos != usart1_tx_len )
	{
		usart1_tx_pos++;
		usart1_tx_pos &= USART1_TX_MASK;
		UDR1 = usart1_tx_buf[ usart1_tx_pos ];
	}
}
*/

SIGNAL( USART1_RX_vect )
{
	usart1_rx_buf[ usart1_rx_len ] = UDR1;

	//UDR1 = usart1_rx_buf[ usart1_rx_len ];

	if( usart1_rx_buf[ usart1_rx_len ] == '\r' || usart1_rx_buf[ usart1_rx_len ] == '\n' )
	{
		usart1_rx_buf[ usart1_rx_len ] = 0;
		cmd_received_flag = 1;
		usart1_rx_len = 0;
		
		UCSR1B &= ( 0xFF ^ _BV( RXCIE1 ) );
		
	}
	else
		usart1_rx_len++;

}


extern uint32_t ircode;
extern volatile struct engine_params_t eng_params;
extern uint8_t num_trying;
extern uint8_t num_params_req;
extern volatile uint8_t g_pwr_state;
extern volatile uint8_t power_off_timer;

#include <util/atomic.h>

extern uint8_t rc5_num_retry;
extern struct ir_t ir;

extern uint32_t vss;
extern uint32_t rpm;

//void elm327_task(void *pvParameters) __attribute__ ( ( naked ) );
void uarthost_task(void *pvParameters)
{

	//uarthost_reset();
	//uarthost_welcome();

	while(1)
	{
		if( usart1_tx_buf[0] != 0 )
		{
			usart1_send_str_bl( usart1_tx_buf );
			usart1_tx_buf[0] = 0;			
		}
		
		
		//start sync seq
		
		usart1_send_uint32( 0xFCFDFEFF );

		usart1_send_uint8( HOSTPROTO_KWP2000_STATUS_CODE );
		usart1_send_uint8( kwp2000_connection_status() );
		
		usart1_send_uint8( HOSTPROTO_OBD_FUEL_LEVEL );
		usart1_send_uint8( eng_params.fuel_level );
		
		usart1_send_uint8( HOSTPROTO_OBD_VEHICLE_SPEED );
		usart1_send_uint8( eng_params.velocity );
		
		usart1_send_uint8( HOSTPROTO_OBD_CS_RPM | 0x40 );
		usart1_send_uint16( eng_params.cs_rpm );		

		usart1_send_uint8( HOSTPROTO_ADC0 );
		usart1_send_uint8( adc(0) );

		usart1_send_uint8( HOSTPROTO_ADC1 );
		usart1_send_uint8( adc(1) );

		usart1_send_uint8( HOSTPROTO_ADC2 );
		usart1_send_uint8( adc(2) );

		usart1_send_uint8( HOSTPROTO_GPIOS | 0x80 );
		usart1_send_uint32( allgpios() );

		usart1_send_uint8( 0x31 );
		usart1_send_uint8( num_trying );

		usart1_send_uint8( 0x32 );
		usart1_send_uint8( num_params_req );
		
		usart1_send_uint8( HOSTPROTO_INSTANT_CONS | 0x80 );
		usart1_send_uint32( eng_params.instant_fuel_cons );
		
		usart1_send_uint8( HOSTPROTO_COOLANT_T );
		usart1_send_uint8( eng_params.coolant_tmpr );

		usart1_send_uint8( HOSTPROTO_INTAKE_T );
		usart1_send_uint8( eng_params.intake_tmpr );
		
		usart1_send_uint8( HOSTPROTO_THROTTLE_POS );
		usart1_send_uint8( eng_params.thr_pos );

		usart1_send_uint8( HOSTPROTO_ACC_LEVEL );
		usart1_send_uint8( eng_params.acc_lev );

		usart1_send_uint8( HOSTPROTO_BAR_PRESSURE | 0x40);
		usart1_send_uint16( eng_params.bar_pressure );

		usart1_send_uint8( HOSTPROTO_ABS_PRESSURE | 0x40 );
		usart1_send_uint16( eng_params.abs_pressure );

		usart1_send_uint8( HOSTPROTO_AIR_CONS | 0x40);
		usart1_send_uint16( eng_params.air_cons );

		usart1_send_uint8( HOSTPROTO_INJ_TIME_START | 0x40);
		usart1_send_uint16( eng_params.inj_time_start );

		usart1_send_uint8( HOSTPROTO_GEAR_RATIO );
		usart1_send_uint8( eng_params.gear_ratio );

		usart1_send_uint8( HOSTPROTO_GEAR_RATIO );
		usart1_send_uint8( eng_params.gear_ratio );

		usart1_send_uint8( HOSTPROTO_POWER_STATE );
		usart1_send_uint8( g_pwr_state );

		usart1_send_uint8( HOSTPROTO_POWER_OFF_TIMER);
		usart1_send_uint8( power_off_timer );
		
		static uint32_t rc5_old_code;
		static uint8_t rc5_hold_timer = 0;
		
		usart1_send_uint8( HOSTPROTO_IR_CODE | 0x80);
		usart1_send_uint32( ir.code );
		
		if( ir.code == rc5_old_code )
		{
			if( rc5_hold_timer < 255 )
				rc5_hold_timer++;
		}
		else
			rc5_hold_timer = 0;
		
		rc5_old_code = ir.code;

		usart1_send_uint8( 0x33 );
		usart1_send_uint8( rc5_hold_timer );

		usart1_send_uint8( 0x34 | 0x80 );
		usart1_send_uint32( vss );

		usart1_send_uint8( 0x35 | 0x80 );
		usart1_send_uint32( rpm );

				
		//end sync seq
		usart1_send_uint32( 0xFFFEFDFC );


		if( !cmd_received_flag )
		{
			vTaskDelay(5);
			continue;
		}
		cmd_received_flag = 0;
		
//    int cmd;
//    char param[8];
//    char arg[4];
//    char value[5];

    //memset(param,0,8);
	//memset(arg,0,4);
	//memset(value,0,5);

    //parse_command(usart1_rx_buf,&cmd,param,arg,value );
	//bzero( usart1_rx_buf, 16 );	
	//UCSR1B |= _BV( RXCIE1 );

		//if( strcmp( param , "pwm0.T") == 0 )
		//{
			//uint8_t intens = atoi( value );
			//set_bkl_intensity( intens );
		//}
		//else
		//if( strcmp( param , "led.T") == 0 )	
		//{
//			uint8_t v = atoi( value );
			//g_led_T =  v;			
		//}
		//else
		//if( strcmp( param , "lcd.bkl") == 0 )
		//{
			//uint8_t v = atoi( value );
			//set_bkl_power(v);
		//}

	}
}




