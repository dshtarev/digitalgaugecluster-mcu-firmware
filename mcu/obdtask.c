#include "obdtask.h"
#include "params.h"
#include <avr/pgmspace.h>
#include "kwp2000.h"
#include "eep.h"
#include "obd.h"
#include <FreeRTOS.h>
#include <task.h>
#include <avr/interrupt.h>

extern volatile uint8_t g_pwr_state;
extern uint16_t ce_error[3];
extern volatile struct engine_params_t eng_params;
extern struct eep_settings_t eep_settings;
extern struct obd_pid_descr_t obd_pid_descr[];

extern uint8_t obd_pid_results[10];
extern uint32_t obd_pid_on;


uint16_t ce_error[3] = {0,0,0};
uint8_t g_read_ce_flag = 0;
uint8_t g_clear_ce_flag = 0;

volatile uint8_t num_trying = 0;
volatile uint8_t num_params_req = 0;
volatile uint16_t cs_rpm = 0;

void obd_task(void *pvParameters) 
{
	static uint8_t pidnum = 0;

	static uint8_t lc = 0;

	static uint8_t obd_fault_timer = 0;

	while(1)
	{
		
		cli();
		uint8_t pwrstate = g_pwr_state;
		sei();
		
		if( pwrstate == PWR_OFF )
		{
			vTaskDelay(1000);
			continue;
		}
		
		usart_clear_rx_buf();

		if( kwp2000_connection_status() == KWP2000_NOT_CONNECTED )
		{
			kwp2000_fast_init();
			
			//imit;
			//delay_ms(500);
//uint8_t imit[]={0x83, 0xf1 ,0x01,0xc1 , 0xe9, 0x8f, 0xae};
//kwp2000_send_raw_pkt(imit,7);
						
			//delay_ms(500);

			pidnum = 0;

			vTaskDelay(eep_settings.obd_connection_req_delay * 1000);

			uint8_t imit[]={0x81, 0x11 ,0xF1,0x81 , 0x04};
			kwp2000_send_raw_pkt(imit,5);

			vTaskDelay(eep_settings.obd_connection_req_delay * 1000);
			kwp2000_check_connection_status();
			usart_clear_rx_buf();
			
			num_trying++;

		}
		else
		{
				if(obd_pid_on & (1<<pidnum) )
				{
					uint8_t msg[2];
					msg[0] = pgm_read_byte( &(obd_pid_descr[pidnum].mode) );
					msg[1] = pgm_read_byte( &(obd_pid_descr[pidnum].pidnum) );
					
					kwp2000_send_pkt(KWP2000_ECU_ADDR, KWP2000_SCANTOOL_ADDR, msg,2 );
					
					vTaskDelay( eep_settings.obd_params_req_delay );


//uint8_t imit[]={0x84, 0xf1 ,0x01,0x41 ,0x0c, 0x01, 0x12, 0x8f};
//imit[6] = i++;
//kwp2000_send_raw_pkt(imit,8);

//					delay_ms(300);

					uint8_t len;
					if( (lc=kwp2000_check_msg(&len)) == KWP2000_OK )
					{
						int j=0;
						for(j=0;j<len;j++)
							obd_pid_results[j] = kwp2000_data_buf()[j];

						if (pidnum == OBD_ENGINE_LOAD )
							eng_params.eng_load = obd_param_to_int( OBD_ENGINE_LOAD );
					}
					else
					{
						int j=0;
						for(j=0;j<8;j++)
							obd_pid_results[j] = 0xFF;

						obd_fault_timer++;

						if( obd_fault_timer > OBD_MAX_FAULTS )
						{
							kwp2000_set_connection_status( KWP2000_NOT_CONNECTED );
							obd_fault_timer = 0;
						}
					}
				
				}	

			usart_clear_rx_buf();

			pidnum++;
			if( pidnum >= OBD_PID_COUNT )
			{
				uint8_t reqmanuf[6] = {0x82,0x11,0xF1,0x21,0x02,0xA7};
				kwp2000_send_raw_pkt(reqmanuf,6);

				vTaskDelay( eep_settings.obd_params_req_delay );

				num_params_req++;

				


				uint8_t hi_injt = usart_rx_buf_f()[27-7];
				uint8_t low_injt = usart_rx_buf_f()[28-7];

//				int i=0;
//				for(i=0;i<24;i++)
//					g_s_p[i] = usart_rx_buf_f()[i];

				eng_params.inj_time = ((usart_rx_buf_f()[27-7]<<8)+usart_rx_buf_f()[28-7]);

				eng_params.fuel_level = usart_rx_buf_f()[37-7];

				eng_params.gear_ratio = usart_rx_buf_f()[33-7];

				eng_params.coolant_tmpr = ((uint16_t)usart_rx_buf_f()[12-7] - 65)*3/4;
				eng_params.intake_tmpr  = ((uint16_t)usart_rx_buf_f()[13-7] - 65)*3/4;
				eng_params.thr_pos      =  usart_rx_buf_f()[14-7]*100/255;
				eng_params.acc_lev      =  usart_rx_buf_f()[15-7]*100/255;
				eng_params.velocity      =  usart_rx_buf_f()[16-7];
				eng_params.cs_rpm        =  (usart_rx_buf_f()[17-7]<<8)+usart_rx_buf_f()[18-7];
				eng_params.bar_pressure      =  ((usart_rx_buf_f()[19-7]<<8)+usart_rx_buf_f()[20-7])/550;
				eng_params.abs_pressure      =  ((usart_rx_buf_f()[21-7]<<8)+usart_rx_buf_f()[22-7])/550;
				eng_params.air_cons      =  ((usart_rx_buf_f()[23-7]<<8)+usart_rx_buf_f()[24-7])/47;
				eng_params.inj_time_start      =  ((usart_rx_buf_f()[25-7]<<8)+usart_rx_buf_f()[26-7])/255;

				uint32_t tmp = (uint32_t)eng_params.inj_time * (uint32_t)eng_params.cs_rpm / 128;
				eng_params.instant_fuel_cons = tmp * (uint32_t)eep_settings.inj_rate;

				pidnum = 0;
				
				if( eng_params.fuel_level == 0 )
				{
					obd_fault_timer++;
				}
				if( obd_fault_timer > OBD_MAX_FAULTS )
				{
					kwp2000_set_connection_status( KWP2000_NOT_CONNECTED );
					obd_fault_timer = 0;
				}				
				
			}
			usart_clear_rx_buf();	
		}
		
		if( g_read_ce_flag )
		{

			// mode3 reqest
			// DTC Request
			uint8_t msg = 0x03;
					
			kwp2000_send_pkt(KWP2000_ECU_ADDR, KWP2000_SCANTOOL_ADDR, &msg,1 );
			vTaskDelay( eep_settings.obd_params_req_delay );

			uint16_t * ce_ptr = ((uint16_t * )(kwp2000_data_buf()+1));
			ce_error[0] = ce_ptr[0];
			ce_error[1] = ce_ptr[1];
			ce_error[2] = ce_ptr[2];

			g_read_ce_flag = 0;

			usart_clear_rx_buf();
		}
			

		if( g_clear_ce_flag )
		{
			// mode4 reqest
			// CLear DTC
			uint8_t msg = 0x04;

			kwp2000_send_pkt(KWP2000_ECU_ADDR, KWP2000_SCANTOOL_ADDR, &msg,1 );
			vTaskDelay( eep_settings.obd_params_req_delay );

			g_clear_ce_flag = 0;
		}

	}

}



void obd_clear_dtc()
{
	g_clear_ce_flag = 1;
}

void obd_read_dtc()
{
	g_read_ce_flag = 1;
}
