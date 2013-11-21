#ifndef __EEP
#define __EEP


#include <avr/eeprom.h>
#include <stdint.h>
#include "adc.h"

struct eep_settings_t{
	uint8_t lcd_backlight_level_day;
	uint8_t lcd_backlight_level_night;

	uint8_t obd_connection_req_delay;
	uint16_t obd_params_req_delay;

	uint8_t adc_calibration[ ADC_CH_COUNT ];

	uint16_t inj_rate;

	uint8_t eep_crc[ 2 ];

};


void eep_save_settings();
void eep_restore_settings();




#endif
