#ifndef __KWP2000
#define __KWP2000

#include <stdint.h>

#define KWP2000_SCANTOOL_ADDR 0xF1
#define KWP2000_ECU_ADDR 0x33

#define KWP2000_GET_LENGTH(x)  ((x[0])&0x3F)

#define KWP2000_LENGTH_BYTE( len )  (0xC0 | len)


#define KWP2000_NOT_CONNECTED 10
#define KWP2000_CONNECTED     11

#define KWP2000_CRC_ERROR     12
#define KWP2000_ECU_ERROR     13
#define KWP2000_RX_BUF_EMPTY  14
#define KWP2000_MSG_INCOMPLETED 15

#define KWP2000_OK            1
#define KWP2000_FAILED        0

int kwp2000_fast_init();
int kwp2000_send_pkt(uint8_t dst_addr, uint8_t src_addr,uint8_t *,uint8_t len);
int kwp2000_send_raw_pkt(uint8_t *,uint8_t len);

void kwp2000_set_connection_status( uint8_t );

uint8_t kwp2000_connection_status();
uint8_t kwp2000_check_connection_status();

uint8_t kwp2000_check_msg(uint8_t * len);
uint8_t * kwp2000_data_buf();

void usart0_init();
void      usart_clear_rx_buf();
uint8_t * usart_rx_buf_f();
uint8_t * usart_tx_buf_f();
uint8_t   usart_rx_len_f();

#endif
