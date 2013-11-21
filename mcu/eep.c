#include "eep.h"


struct eep_settings_t eep_settings;
struct eep_settings_t eep_settings_in_eeprom EEMEM;


void eep_init_defaults()
{
	eep_settings.lcd_backlight_level_day = 30;
	eep_settings.lcd_backlight_level_night = 50;

	eep_settings.obd_params_req_delay = 150;
	eep_settings.obd_connection_req_delay = 1;

	int i = 0;
	for ( i = 0;i< ADC_CH_COUNT;i++)
		eep_settings.adc_calibration[i] = 100;

	eep_settings.inj_rate = 250;

	eep_settings.eep_crc[0] = 0x1B;
	eep_settings.eep_crc[1] = 0xE1;

}

void eep_save_settings()
{
	eeprom_write_block ((const void *)&eep_settings,(void *)&eep_settings_in_eeprom, sizeof(struct eep_settings_t));

}


void eep_restore_settings()
{
	eeprom_read_block ((void *)&eep_settings, (const void *)&eep_settings_in_eeprom,sizeof(struct eep_settings_t));

	//Если содержимое EEPROM не верное то восстанавливаем значения по умолчанию
	// и записываем их в ЕЕПРОМ
	if( (eep_settings.eep_crc[0] != 0x1B) || (eep_settings.eep_crc[1] != 0xE1) )
	{
		eep_init_defaults();
		eep_save_settings();
	}

}


