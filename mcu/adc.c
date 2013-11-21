#include "adc.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>



void adc_init()
{

	//�������� ����������� ����� 2.56
	ADMUX |= _BV(REFS0) | _BV(REFS1) | _BV(ADLAR);

	// ����������� ������� �������� ������� - 128
	ADCSRA = _BV( ADEN ) | _BV( ADIE ) | _BV( ADPS0 ) | _BV( ADPS1 ) | _BV( ADPS2 );

	// ������ ��������������
	ADCSRA |= _BV( ADSC );


}

static uint8_t adc_val[] = {0,0,0,0};
static uint8_t adc_channels[] = {0,1,2,3};

static uint8_t chnum = 0;
SIGNAL( ADC_vect )
{
	adc_val[chnum] = ADCH;

	chnum++;

	// ���������� ������ ������ ������ � �����
	if( chnum > 3 )
		chnum = 0;

	// �� ���������� ������ 4-7
	ADMUX &= 0xE0;
	ADMUX |= adc_channels[ chnum ];
	
	// ������ ��������������
	ADCSRA |= _BV( ADSC );
}


uint8_t adc( uint8_t ch )
{
	return adc_val[ch] * 110 / 100;//eep_settings.adc_calibration[ch];
}
