#include "Keypad_I2C.h"
#include "main.h"


#define KEYPAD_ADDRESS			ADDRESS<<1

extern I2C_HandleTypeDef hi2c;

uint8_t getKey(void)
{
  // key = row + 4 x col
  uint8_t key = 0;

  // mask = 4 rows as input-pullup, 4 colomns as output
  uint8_t rows = _read(0xF0);
  // check if single line has gone low.
  if (rows == 0xF0)      return I2C_KEYPAD_NOKEY;
  else if (rows == 0xE0) key = 0;
  else if (rows == 0xD0) key = 1;
  else if (rows == 0xB0) key = 2;
  else if (rows == 0x70) key = 3;
  else return I2C_KEYPAD_FAIL;


  // 4 columns as input-pullup, 4 rows as output
  uint8_t cols = _read(0x0F);
  // check if single line has gone low.
  if (cols == 0x0F)      return I2C_KEYPAD_NOKEY;
  else if (cols == 0x0E) key += 0;
  else if (cols == 0x0D) key += 4;
  else if (cols == 0x0B) key += 8;
  else if (cols == 0x07) key += 12;
  else return I2C_KEYPAD_FAIL;


  return key;   // 0..15
}

uint8_t _read(uint8_t mask)
{
	uint8_t data[1] = {mask};
	HAL_I2C_Master_Transmit(&hi2c, KEYPAD_ADDRESS, data, 1, 100);
	data[0] = 0;
	HAL_I2C_Master_Receive(&hi2c, KEYPAD_ADDRESS, data, 1, 100);
	return data[0];
}
