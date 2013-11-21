#ifndef __PARAMS
#define __PARAMS

#include <stdint.h>


#define PWR_OFF 0
#define PWR_ON  1


#define MAX_SENSORS_COUNT 2

struct engine_params_t
{	
	uint8_t   fuel_level;
	uint8_t   gear_ratio;

	int8_t    coolant_tmpr;
	int8_t    intake_tmpr;
	int8_t    thr_pos;
	int8_t    acc_lev;
	uint8_t   velocity;
	uint16_t  cs_rpm;
	uint16_t  bar_pressure;
	uint16_t  abs_pressure;
	uint16_t  air_cons;
	uint16_t  inj_time;	
	uint16_t  inj_time_start;	

	uint32_t     instant_fuel_cons;
	
	uint8_t   eng_load;

};


struct avg_speed_t
{
	uint32_t speed;
	uint32_t count;
};

struct trip_params_t
{
	float distance;
	uint32_t travel_time;
	uint32_t stop_time;
	struct avg_speed_t avg_speed_total;
	struct avg_speed_t avg_speed_wo_stops;
	uint8_t            max_speed;
	float avg_fuel_consumption;
	float avg_fuel_consumption_sum;


};

struct rt_params_t
{
	//напряжение борт сети
	float acc_voltage;
	//мгновенный расход
	float instant_fuel_consumption;
	//обороты
	uint16_t rpm;
	//cкорость
	uint8_t speed;
	//остаток топлива в бакеl
	uint8_t fuel_level;

	// Скорость прогрева двигателя
	int engine_heating_speed;
};


#endif
