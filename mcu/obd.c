#include "obd.h"

#include <avr/pgmspace.h>

struct obd_pid_descr_t const obd_pid_descr[OBD_PID_COUNT] PROGMEM=
{

{OBD_COOLANT_T, 0x01, 0x05, "Coolant temperature" },
{OBD_RPM, 0x01, 0x0C, "Engine RPM" },
{OBD_MAF, 0x01, 0x10, "Maf Air Flow"},
{OBD_FUEL_LEVEL, 0x01, 0x2F, "Fuel Level Input"},
{OBD_SPEED, 0x01, 0x0D, "Vehicle Speed"},
{OBD_INTAKE_T, 0x01, 0x0F, "Intake Air Temperature"},
{OBD_THR_POS, 0x01, 0x11, "Absolute ThrPos"},
{OBD_ENGINE_LOAD, 0x01, 0x04, "Calculated engine load"},
{OBD_MAP, 0x01, 0x0B, "Intake Manifold Pressure"},
{OBD_VCC, 0x01, 0x42, "Control Module voltage"}
};

uint8_t obd_pid_results[10];
uint32_t obd_pid_on;

void obd_init_params()
{

	obd_pid_results[2] = 0xFF;

	obd_pid_on = 0;
	//obd_pid_on |= ( 1<<OBD_ENGINE_LOAD );


/*	obd_pid_results[ OBD_COOLANT_T ].result[2] = 23;
	obd_pid_results[ OBD_RPM ].result[2] = 10;
	
	obd_pid_results[ OBD_FUEL_LEVEL ].result[2] = 190;

	obd_pid_results[ OBD_SPEED ].result[2] = 83;

	obd_pid_results[ OBD_INTAKE_T ].result[2] = 37;

	obd_pid_results[ OBD_THR_POS ].result[2] = 80;

	obd_pid_results[ OBD_ENGINE_LOAD ].result[2] = 200;

	obd_pid_results[ OBD_MAP ].result[2] = 27;

	obd_pid_results[ OBD_VCC ].result[2] = 100;*/


}


int obd_param_to_int(uint8_t param)
{
	switch( param )
	{

	case OBD_COOLANT_T:
		return (int)obd_pid_results[2] - 40;
		break;

	case OBD_RPM:
		return (((int)obd_pid_results[2])*256 + obd_pid_results[3]) / 4;
		break;

	case OBD_MAF:
		break;

		
	case OBD_FUEL_LEVEL:
		return (int)(obd_pid_results[2] * 100)>>8 ;
		break;


	case OBD_SPEED:
		return (int)(obd_pid_results[2]);
		break;

	case OBD_INTAKE_T:
		return (int)(obd_pid_results[2])-40;
		break;

	case OBD_THR_POS:
		return (int)(obd_pid_results[2] * 100)>>8 ;
		break;

	case OBD_ENGINE_LOAD:
		return (int)(obd_pid_results[2] * 100)>>8 ;
		break;

	case OBD_MAP:
		return (int)(obd_pid_results[2]);
		break;

	case OBD_VCC:
		return (((int)obd_pid_results[2])*256 + obd_pid_results[3]) / 1000;
		break;	

	}
	
	return 0;
}



int8_t obd_validate_pid( uint8_t pid )
{
	return obd_pid_results[2]==0xFF?-1:0;

}

