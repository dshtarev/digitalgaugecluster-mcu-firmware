#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "rc5decoder.h"
#include "periph.h"

// пороговое значение для сравнения длинн импульсов и пауз
static const int IrPulseThershold = 10;// 1024/8000 * 9 = 1.152 msec
// определяет таймаут приема посылки
// и ограничивает максимальную длину импульса и паузы
static const uint8_t TimerReloadValue = 20;
static const uint8_t TimerClock = (1 << CS02) | (1 << CS01) | (1 << CS00);// 8 MHz / 1024

volatile struct ir_t ir;


static void ir_start_timer()
{
	TIMSK |= (1 << TOIE0 );
	TCNT0 = 0;
	TCCR0 = TimerClock;
}

// когда таймер переполнится, считаем, что посылка принята
// копируем принятый код из буфера
// сбрасываем флаги и останавливаем таймер

int a = 0;
extern volatile char usart1_tx_buf[16];

uint8_t rc5_num_retry = 0;

SIGNAL(TIMER0_OVF_vect)
{
	ir.code = ir.rx_buffer;
	ir.rx_buffer = 0;
	ir.rx_started = 0;
	if(ir.code == 0)
	TCCR0 = 0;
	TCNT0 = TimerReloadValue;
	
	if( rc5_num_retry < 255 )
		rc5_num_retry++;
}


// внешнее прерывание по фронту и спаду
SIGNAL(INT7_vect)
{	
	rc5_num_retry = 0;
	
	uint8_t delta;
	if(ir.rx_started)
	{
		// если длительность импульса/паузы больше пороговой
		// сдвигаем в буфер единицу иначе ноль.
		delta = TCNT0 - TimerReloadValue;
		ir.rx_buffer <<= 1;
		if(delta > IrPulseThershold) ir.rx_buffer |= 1;
	}
	else{
		ir.rx_started = 1;
		ir_start_timer();
	}
	TCNT0 = TimerReloadValue;
}

inline void ir_init()
{
	EIMSK |= _BV(INT7);
	EICRB |= (1 << ISC70) | (0 <<ISC71);
	ir_start_timer();
}

