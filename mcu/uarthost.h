#ifndef __ELM327IMIT
#define __ELM327IMIT

#include "freertos.h"

#define HOSTPROTO_KWP2000_STATUS_CODE 0x01
#define HOSTPROTO_OBD_FUEL_LEVEL 0x02
#define HOSTPROTO_OBD_VEHICLE_SPEED 0x03
#define HOSTPROTO_OBD_CS_RPM 0x04
#define HOSTPROTO_IR_CODE 0x05
#define HOSTPROTO_ADC0 0x06
#define HOSTPROTO_ADC1 0x07
#define HOSTPROTO_ADC2 0x08
#define HOSTPROTO_GPIOS 0x09
#define HOSTPROTO_INSTANT_CONS 0x0A
#define HOSTPROTO_COOLANT_T 0x0B
#define HOSTPROTO_INTAKE_T 0x0C
#define HOSTPROTO_THROTTLE_POS 0x0D
#define HOSTPROTO_ACC_LEVEL 0x0E
#define HOSTPROTO_BAR_PRESSURE 0x0F
#define HOSTPROTO_ABS_PRESSURE 0x10
#define HOSTPROTO_AIR_CONS 0x11
#define HOSTPROTO_INJ_TIME_START 0x12
#define HOSTPROTO_GEAR_RATIO 0x13
#define HOSTPROTO_ACC_VOLTAGE 0x14
#define HOSTPROTO_POWER_STATE 0x15
#define HOSTPROTO_POWER_OFF_TIMER 0x16


void uarthost_init();
void uarthost_task(void *pvParameters);

#endif

