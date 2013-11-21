#ifndef __PERIPH
#define __PERIPH

#include <avr/io.h>
#include <util/delay.h>


// K-LINE
// PE0 - RXD0
// PE1 - TXD0

// IR
// PE7 - INT7

// PA0 - SENS13
// PA1 - SENS14
// PA2 - SENS15
// PA3 - SENS16
// PA4 - SENS17
// PA5 - SENS18
// PA6 - SENS19
// PA7 - SENS20

// PC5 - SENS23
// PC6 - SENS22
// PC7 - SENS22

// PE2 - SENS9
// PE3 - SENS8
// PE4 - SENS7
// PE5 - SENS6
// PE6 - SENS5

// PB0 - SENS4
// PB2 - SENS3
// PB3 - SENS2
// PB4 - SENS1


// PF7 - SENS12
// PF6 - SENS11
// PF5 - SENS10


// LED
// PF4 - ANODE

#define LED_CTRL PF4

// LIGHT SENSOR
// PF3 - ADC3 - INPUT

//BKLIGHT
// PB6 - OC1B - PWM
//

//I2C
// PD0 - SDA
// PD1 - SCL

// UART LINK
// PD2 - RXD1
// PD3 - TXD1

//ADC
// PF0 - ADC0 - ILLU
// PF1 - ADC1 - ACC
// PF2 - ADC2 - GP_ANALOG

// GPIOe
// PC0 - GPIO3 - PWR_EN
#define PWR_EN PC0
// PC1 - GPIO1 - 5V_POWER_GOOD
#define POWER_GOOD_5V PC1
// PC2 - GPIO2 - 3V3_POWER_GOOD
#define POWER_GOOD_3V3 PC2
// PC3 - GPIO0 - MX53RESET
#define MX53RESET PC3
// PC4 - GPIO4 - BKL_POWER_CTRL
#define BKL_POWER_CTRL PC4
// PD4 - GPIO5 - NC
// PD5 - GPIO6 - NC
// PD6 - GPIO8 - PWR1_CTRL
#define PWR1_CTRL  PD6
// PD7 - GPIO7 - PWR2_CTRL
#define PWR2_CTRL PD7


#define SET_PIN( port, pin, val )   ( val?(port |= ( 1<<pin ) ): ( port &= 0xFF ^ ( 1<<pin ) ) )


void set_mx53_reset(int v);
void set_pwr_en(int v);
void set_bkl_power(int v);
void set_led( int v );
void set_bkl_intensity(uint8_t v);


#define GPIO_COUNT 23

struct input_pin_descr_t
{
	uint8_t portnum;
	uint8_t pinnum;
};

uint8_t gpio( uint8_t num );
uint32_t allgpios( );

#endif

