/*
 * rc5decoder.h
 *
 * Created: 01.04.2013 21:14:36
 *  Author: dshtarev
 */ 


#ifndef RC5DECODER_H_
#define RC5DECODER_H_

struct ir_t
{
	// ���� ������ ������ �������
	uint8_t rx_started;
	// �������� ���
	uint32_t code,
	// ����� �����
	rx_buffer;
};

void ir_init();


#endif /* RC5DECODER_H_ */