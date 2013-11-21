#include <avr/io.h>
#include <avr/interrupt.h>

#include "kwp2000.h"



#define TX0_SET()   PORTE |= _BV( PE1 )
#define TX0_CLEAR() PORTE &= (0xFF^_BV( PE1 ));

#define USART0_ENABLE()  UCSR0B |= ( _BV(RXCIE0) |_BV(TXCIE0) |_BV(RXEN0) | _BV(TXEN0) ) ;
#define USART0_DISABLE() UCSR0B &= (0xFF ^ ( _BV(RXCIE0) |_BV(TXCIE0) |_BV(RXEN0) | _BV(TXEN0) ) );

#define USART0_RX_EN()  UCSR0B |= ( _BV(RXEN0))  ;
#define USART0_RX_DIS() UCSR0B &= (0xFF ^ ( _BV(RXEN0) ));


void usart0_init()
{
	UCSR0C |=  _BV(UCSZ01) | _BV(UCSZ00);
	UBRR0L = 95;
}

int kwp2000_fast_init()
{
	USART0_DISABLE();

	TX0_SET();
	vTaskDelay(300);
	TX0_CLEAR();
		
	vTaskDelay(25);
	TX0_SET();
	vTaskDelay(25);

	USART0_ENABLE();
	
	// Последовательность инициализации обмена
	static uint8_t initc[] = {0x81};
	kwp2000_send_pkt( KWP2000_ECU_ADDR, KWP2000_SCANTOOL_ADDR,initc, 1 );

	return 0;
}

static uint8_t usart_rx_buf[90];
static uint8_t usart_rx_pos;
static uint8_t usart_rx_len;

uint8_t usart_tx_buf[16];
uint8_t usart_tx_pos=0;
uint8_t usart_tx_len=0;

volatile static uint8_t g_kwp2000_connection_status = KWP2000_NOT_CONNECTED;

uint8_t kwp2000_calc_crc(uint8_t * msg, uint8_t len)
{
	uint8_t i = 0;
	uint8_t crc = 0;
	for(i=0;i<len;i++)
		crc += msg[i];

	return crc;
}

int kwp2000_send_raw_pkt(uint8_t * _pkt,uint8_t len)
{
	int i=0;
	for(i=0;i<len;i++)
	{
		usart_tx_buf[i] = _pkt[i];
	}

	usart_tx_len = len;
	usart_tx_pos = 0;

	USART0_RX_DIS();

	UDR0 = usart_tx_buf[0];

	return 0;
}

int kwp2000_send_pkt(uint8_t dst_addr, uint8_t src_addr, uint8_t * _pkt,uint8_t len)
{
	int i=0;

	usart_tx_buf[0] = KWP2000_LENGTH_BYTE(len);
	usart_tx_buf[1] = dst_addr;
	usart_tx_buf[2] = src_addr;

	for(i=0;i<len;i++)
	{
		usart_tx_buf[i+3] = _pkt[i];
	}

	usart_tx_buf[len+3] = kwp2000_calc_crc(usart_tx_buf, len+3);

	usart_tx_len = len+4;
	usart_tx_pos = 0;

	USART0_RX_DIS();

	UDR0 = usart_tx_buf[0];

	return 0;
}

uint8_t * usart_tx_buf_f()
{	
	return usart_tx_buf;
}

uint8_t * usart_rx_buf_f()
{	
	return usart_rx_buf;
}

uint8_t usart_rx_len_f()
{
	return usart_rx_len;
}

void usart_clear_rx_buf()
{
	usart_rx_pos  = 0;
	usart_rx_len  = 0;

	int i=0;
	for(i=0;i<90;i++)
		usart_rx_buf[i] = 0;
}

uint8_t kwp2000_connection_status()
{
	return g_kwp2000_connection_status;

}

uint8_t kwp2000_check_msg(uint8_t * _len)
{
	if( usart_rx_len == 0 )
		return KWP2000_RX_BUF_EMPTY;
	
	uint8_t len = KWP2000_GET_LENGTH( usart_rx_buf );

	if( (len)+4 < usart_rx_len )
	{
		if( _len != 0 )
			*_len = usart_rx_len;
		return KWP2000_MSG_INCOMPLETED;
	}

	if( _len != 0 )
		*_len = len;

	return KWP2000_OK;
}

uint8_t kwp2000_check_connection_status()
{
	
	if( kwp2000_check_msg(0) == KWP2000_OK )
	{
		if(kwp2000_data_buf()[0] == 0xC1)
		{
			g_kwp2000_connection_status= KWP2000_CONNECTED;
			return KWP2000_OK;
		}
	}
	return KWP2000_FAILED;

}

uint8_t * kwp2000_data_buf(){ return usart_rx_buf + 3; }

SIGNAL( USART0_TX_vect )
{
	usart_tx_pos++;
	if( usart_tx_pos < usart_tx_len )
	{
		UDR0 = usart_tx_buf[ usart_tx_pos ];
	}
	else
		USART0_RX_EN();

}

SIGNAL( USART0_RX_vect )
{
	if( usart_rx_pos < 90 )
	{
		usart_rx_buf[ usart_rx_pos++ ] = UDR0;
		usart_rx_len++;
	}		
	
}

void kwp2000_set_connection_status( uint8_t _status)
{
	g_kwp2000_connection_status = _status;
}






