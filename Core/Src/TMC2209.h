/*
 * TMC2209.h
 *
 *  Created on: Dec 3, 2020
 *      Author: hletrd
 */

#ifndef SRC_TMC2209_H_
#define SRC_TMC2209_H_

class TMC2209 {
public:
	TMC2209();
	virtual ~TMC2209();
private:
	void uart_tx();
	void uart_calc_crc(unsigned char* datagram, int datagramLength);

};

#endif /* SRC_TMC2209_H_ */
