#ifndef __ADC
#define __ADC

#include <stdint.h>

//количество каналов АДЦ
#define ADC_CH_COUNT 4

void adc_init();

uint8_t adc( uint8_t ch );


#endif
