#include "periph.h"


struct input_pin_descr_t const input_pin_descr[GPIO_COUNT]=
{
	{ 1 , PB4 }	,
	{ 1 , PB3 }	,
	{ 1 , PB2 }	,
	{ 1 , PB0 }	,
	{ 4 , PE6 }	,
	{ 4 , PE5 }	,
	{ 4 , PE4 }	,
	{ 4 , PE3 }	,
	{ 4 , PE2 }	,
	{ 5 , PF5 }	,
	{ 5 , PF6 }	,
	{ 5 , PF7 }	,
	{ 0 , PA0 }	,
	{ 0 , PA1 }	,
	{ 0 , PA2 }	,
	{ 0 , PA3 }	,
	{ 0 , PA4 }	,
	{ 0 , PA5 }	,
	{ 0 , PA6 }	,
	{ 0 , PA7 }	,
	{ 2 , PC5 }	,
	{ 2 , PC6 }	,
//	{ 2 , PC7 }
	{ 4, PE0 }
};

void set_mx53_reset(int v)
{
	SET_PIN( PORTC,  MX53RESET, v );
}

void set_pwr_en(int v)
{
	SET_PIN( PORTC, PWR_EN, v );
}

void set_bkl_power(int v)
{
	SET_PIN(PORTC, BKL_POWER_CTRL, v );
}

void set_led( int v )
{
	SET_PIN( PORTF, LED_CTRL, v );
}


void set_bkl_intensity(uint8_t v)
{
	OCR1BL = v;
}


uint8_t gpio( uint8_t num )
{
	uint8_t port = 0;
	switch( input_pin_descr[num].portnum )
	{
		case 0:
			port = PINA;
			break;
		case 1:
			port = PINB;
		break;
		case 2:
			port = PINC;
		break;
		case 3:
			port = PIND;
		break;
		case 4:
			port = PINE;
		break;
		case 5:
			port = PINF;
		break;
	}

	
	return (port >> input_pin_descr[num].pinnum ) & 0x01;
}

uint32_t allgpios( )
{
	uint32_t ret = 0;
	uint8_t port[6];
	port[0] = PINA;
	port[1] = PINB;
	port[2] = PINC;
	port[3] = PIND;
	port[4] = PINE;
	port[5] = PINF;
	
	for(int i=0;i< GPIO_COUNT; i++ )	
	{
		ret |= ( port[input_pin_descr[i].portnum] >> input_pin_descr[i].pinnum ) & 0x01;
		ret <<= 1;
	}	
	
	return ret;
}


