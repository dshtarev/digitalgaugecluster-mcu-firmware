#ifndef __OBD
#define __OBD

#include <stdint.h>

#define OBD_PID_COUNT 10

#define OBD_COOLANT_T    0
#define OBD_RPM          1
#define OBD_MAF          2
#define OBD_FUEL_LEVEL   3
#define OBD_SPEED        4
#define OBD_INTAKE_T     5
#define OBD_THR_POS      6
#define OBD_ENGINE_LOAD  7
#define OBD_MAP          8
#define OBD_VCC          9

struct obd_pid_descr_t{
	uint8_t id;

	uint8_t mode;
	uint8_t pidnum;

	char    descr[25];
};


int obd_param_to_int(uint8_t);
int obd_param_to_int2(uint8_t *, uint8_t);

int8_t obd_validate_pid( uint8_t );


void obd_init_params();

#endif

