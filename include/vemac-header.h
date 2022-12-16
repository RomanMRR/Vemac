#ifndef VEMAC_H
#define VEMAC_H

#include <stdint.h>
#include "crypto/aes.h"
#include "cmac.h"

#define LS_MIC_KEY_LEN AES_KEY_SIZE



typedef struct 
{	
	uint8_t mHdr;
	uint8_t fCtrl;
	uint16_t fcnt;
	uint8_t slots_id [10];
// 	uint8_t mic_key[LS_MIC_KEY_LEN];
// 	uint8_t aes_key[AES_KEY_SIZE] = {
//     0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
//     0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
// };
// 	uint8_t join_key[AES_KEY_SIZE] = {
//     0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
//     0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
// };

} vemac_header_t;

void serialize (vemac_header_t * header, uint8_t *message);
uint32_t  deserialize (uint8_t *message, vemac_header_t * header);

#endif