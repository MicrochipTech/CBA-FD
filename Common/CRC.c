/*
 * CRC.c
 *
 * Created: 12.10.2017 15:11:12
 *  Author: M43734
 */ 

#include "CRC.h"

static uint32_t crc32_table[256] = { 0 };

uint16_t crc16(const uint8_t* data_p, uint16_t length)
{
  uint8_t x;
  uint16_t crc = 0xFFFF;

  while (length--){
    x = crc >> 8 ^ *data_p++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
  }
  return crc;
}

void crc32c_init_table()
{
	const uint32_t Polynomial = 0xEDB88320;
	for(uint32_t i = 0; i <= 0xFF; i++)
	{
		uint32_t crc = i;
		for (unsigned int j = 0; j < 8; j++)
		{
			crc = (crc >> 1) ^ (-((int32_t)crc & 1) & Polynomial);
		}
		crc32_table[i] = crc;
	}
}

uint32_t crc32(const uint8_t *message, uint32_t length)
{
	return ~crc32c(message, length, 0xFFFFFFFFu);
}

uint32_t crc32c(const uint8_t *message, uint32_t length, uint32_t crc)
{
	int32_t i = 0;

	while (length--)
	{
		crc = (crc >> 8) ^ crc32_table[(crc ^ message[i++]) & 0xFF];
	}
	return crc;
}
