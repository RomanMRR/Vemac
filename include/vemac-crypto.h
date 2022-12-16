#ifndef VEMAC_CRYPTO
#define VEMAC_CRYPTO


#include "vemac-header.h"

void LoRaMacComputeMic( const uint8_t *buffer, uint8_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint32_t *mic );

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint8_t *encBuffer );

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint8_t *decBuffer );

#endif 