/*
 * CRC.h
 *
 * Created: 12.10.2017 15:11:22
 *  Author: M43734
 */ 

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

#define CRC32_INITVAL 0xFFFFFFFFu

#ifdef __cplusplus
extern "C" {
#endif

void crc32c_init_table();
uint32_t crc32(const uint8_t *message, uint32_t length);
uint32_t crc32c(const uint8_t *message, uint32_t length, uint32_t crc);
uint16_t crc16(const uint8_t* data_p, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H_ */
