/*
 * TMC2209.cpp
 *
 *  Created on: Dec 3, 2020
 *      Author: hletrd
 */

#include "TMC2209.h"

TMC2209::TMC2209() {
	// TODO Auto-generated constructor stub

}

TMC2209::uart_tx() {
	unsigned char uart_buf[8];
	uart_buf[0] = 0b10100000; //sync + reserved
	uart_buf[1] = 0b00000000; //slave address
}

TMC2209::uart_calc_crc(unsigned char* datagram, int datagramLength) {
	unsigned char* crc = datagram + (datagramLength-1);
	unsigned char currentByte;
	*crc = 0;
	for (int i=0; i<(datagramLength-1); i++) {
		currentByte = datagram[i];
		for (int j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01)) {
				*crc = (*crc << 1) ^ 0x07;
			} else {
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		}
	}
}

TMC2209::~TMC2209() {
	// TODO Auto-generated destructor stub
}

