/*
 * Keypad_I2C.h
 *
 *  Created on: ۱۴ دی ۱۳۹۹
 *      Author: Mashroti
 */

#ifndef INC_KEYPAD_I2C_H_
#define INC_KEYPAD_I2C_H_

#include <stdint.h>

#define hi2c	hi2c1


#define I2C_KEYPAD_NOKEY		16
#define I2C_KEYPAD_FAIL			17

#define ADDRESS					0x26


uint8_t getKey(void);
uint8_t _read(uint8_t mask);



#endif /* INC_KEYPAD_I2C_H_ */
