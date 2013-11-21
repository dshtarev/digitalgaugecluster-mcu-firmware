/*
* mcu.c
*
* Created: 22.03.2013 9:32:18
*  Author: Dmitry
*/


#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uarthost.h"
#include "adc.h"
#include "params.h"
#include "obdtask.h"
#include "kwp2000.h"
#include "eep.h"
#include "periph.h"
#include "rc5decoder.h"
#include "obd.h"

//////////////////////////////////////////////////////////////////////////
// GLOBAL PARAMS
//////////////////////////////////////////////////////////////////////////
extern struct eep_settings_t eep_settings;

//extern struct rt_params_t rt_params;
//extern struct trip_params_t trip_a;

uint32_t secTimer = 0;
volatile uint8_t g_pwr_state = PWR_OFF;
volatile struct engine_params_t eng_params;

uint8_t g_led_T = 30;
uint8_t hazard_state = 0;

//////////////////////////////////////////////////////////////////////////

static void led_task(void *pvParameters)
{
	int led = 1;
	while(1)
	{
		if( hazard_state )
		{
			PORTD ^= _BV( PWR1_CTRL );
			PORTD ^= _BV( PWR2_CTRL );
		}
		else
		{
			PORTD &= 0x3F;
		}
		
		vTaskDelay( g_led_T * 10 );
		set_led( led );
		led ^= 1;

		set_bkl_intensity( adc(0) );

	}
}

extern struct ir_t ir;
extern volatile char usart1_tx_buf[16];
uint32_t ircode;

static void ir_task(void *pvParameters)
{
	uint32_t tmp;
	
	while(1)
	{
		if( tmp != ir.code )
		if( ir.code == 0xa00a0aa )
		{
			hazard_state ^= 1;
		}	
		tmp = ir.code;
				
		vTaskDelay( 100 );
	}
}

void power_on()
{
	set_mx53_reset( 1 );  // turn on mx53 reset
	set_pwr_en( 1 );	  // switch power on
	_delay_ms( 10 );      // wait for dc-dc powerup
	set_mx53_reset( 0 );  // hold-off mx53 reset
}


void init_peripheral()
{
		DDRA = 0x00;  //
		DDRB = ( 1 << PB1 ) | ( 1 << PB5 ) | ( 1 << PB6 ) | ( 1 << PB7 );
		DDRC = ( 1 << PWR_EN ) | ( 1 << MX53RESET ) | ( 1 << BKL_POWER_CTRL );
		DDRD = ( 1 << PD3 ) | ( 1 << PD4 ) | ( 1 << PD5 ) | ( 1 << PWR1_CTRL ) | ( 1 << PWR2_CTRL );
		DDRE = ( 1 << PE1 ); //TXD0
		DDRF = ( 1 << LED_CTRL );
		
		PORTE |= _BV(PE7);
		
		//TODO:: Please write your application code
		
		// LCD PWM
		TCCR1A =  _BV(WGM10)  | _BV(COM1B1);
		TCCR1B = _BV(CS10) | _BV(WGM12);

		//TIMSK |= _BV( TOIE1 );

		set_bkl_intensity( 0x90 );				

		EIMSK |= _BV(INT5);
		EIMSK |= _BV(INT6);
		EICRB |= (1 << ISC50) | (1 <<ISC51);
		EICRB |= (1 << ISC60) | (1 <<ISC61);
		
		//PORTD |= ( 1 << PWR1_CTRL ) | ( 1 << PWR2_CTRL );

}
//////////////////////////////////////////////////////////
uint32_t tickCount = 0;
struct input_state_t
{
	uint8_t v:1;
	uint8_t v_o:1;
	uint32_t timerValue;
	uint32_t width_1;
	uint32_t width_0;	
	uint32_t count;
};


//volatile struct input_state_t inputs[23];

extern struct input_pin_descr_t const input_pin_descr[GPIO_COUNT];

int aa = 0;

void vApplicationIdleHook( void )
{	

}

//volatile uint8_t portStamp[5];

//extern volatile uint32_t vSystemTimer;

uint32_t vss=0;
uint32_t rpm=0;

SIGNAL( INT6_vect )
{
	rpm++;
}

SIGNAL( INT5_vect )
{
	vss++;
//	uint32_t tc = tickCount;
	
//	uint8_t pinnum = input_pin_descr[5].pinnum;
//	inputs[5].v = ( PINE & ( 1 << PE5 ) ) >> PE5;

//	if( inputs[5].v != inputs[5].v_o )
//	{
		//if( inputs[5].v == 1 )
		//{
			//inputs[5].width_0 = tc - inputs[5].timerValue;
		//}
		//else
		//{
			//inputs[5].width_1 = tc - inputs[5].timerValue;
		//}

		//inputs[5].count++;
		//inputs[5].timerValue = tc;
	//}
	
	//inputs[5].v_o = inputs[5].v;
 	
}

//SIGNAL( TIMER1_OVF_vect )
//{
//cli();
//	tickCount++;

// 	uint32_t tc = tickCount;
// 	portStamp[0]  = PINA;
// 	portStamp[1]  = PINB;
// 	portStamp[2]  = PINC;
// 	portStamp[3]  = PIND;
// 	portStamp[4]  = PINE;
// 	portStamp[5]  = PINF;
// 	SET_PIN( PORTC, PC0, aa );
// 	aa ^= 1;
// 			
// 	uint8_t portVal;
// 	for(int i=0;i<2;i++)
// 	{
// 		portVal = portStamp[ input_pin_descr[i].portnum ];
// 	
// 		uint8_t pinnum = input_pin_descr[i].pinnum;
// 		inputs[i].v = ( portVal & ( 1 << pinnum ) ) >> pinnum;
// 	
// 		
// 		if( inputs[i].v != inputs[i].v_o )
// 		{
// 			if( inputs[i].v == 1 )
// 			{
// 				inputs[i].width_0 = tc - inputs[i].timerValue;
// 			}
// 			else
// 			{
// 				inputs[i].width_1 = tc - inputs[i].timerValue;
// 			}
// 		
// 			inputs[i].count++;		
// 			inputs[i].timerValue = tc;								
// 		}
// 		inputs[i].v_o = inputs[i].v;
// 	}	
//}

volatile uint8_t power_off_timer = 0;
static void aux_task(void *pvParameters)
{

	while(1)
	{
		vTaskDelay(100);
		if( g_pwr_state == PWR_OFF )
		{
			if( adc(1) > 50 )
			{
				power_on();
				g_pwr_state = PWR_ON;
				vTaskDelay(100);
				set_bkl_power( 1 );
				power_off_timer = 0;
			}
		}
		else
		if( g_pwr_state == PWR_ON )
		{
			if( adc(1) < 30 )
			{
				power_off_timer++;
				if( power_off_timer > 30 )
				{
					g_pwr_state = PWR_OFF;
					set_bkl_power( 0 );
					set_pwr_en( 0 );
				}					
			}
			else
			{
				power_off_timer = 0;
			}
		}
		else
			g_pwr_state = PWR_OFF;
	}
}

//////////////////////////////////////////////////////////
int main(void)
{
	init_peripheral();
	
	eep_restore_settings();
	adc_init();
	uarthost_init();	
	usart0_init();
	obd_init_params();
	ir_init();  // IR Receiver init
	
	
	xTaskCreate( led_task, "led_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( obd_task, "obd_task", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( uarthost_task, "t5", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( ir_task, "ir_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( aux_task, "aux_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	
	vTaskStartScheduler();		
	//vTaskSuspendAll();

}

