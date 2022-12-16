#include "include/vemac-crypto.h"
#include <stdint.h>

// #include "include/utilities.h"

#define LORAMAC_MIC_BLOCK_B0_SIZE                   16

/*!
 * AES computation context variable
 */
static cipher_context_t AesContext;

/*!
 * CMAC computation context variable
 */
static AES_CMAC_CTX AesCmacCtx[1];

/*!
 * MIC field computation initial data
 */
static uint8_t MicBlockB0[] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              };

static AES_CMAC_CTX AesCmacCtx[1];

static uint8_t Mic[16];

static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };

void LoRaMacComputeMic( const uint8_t *buffer, uint8_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint32_t *mic )
{
    MicBlockB0[5] = dir;
    
    MicBlockB0[6] = header->slots_id[0];
    MicBlockB0[7] = header->slots_id[1];
    MicBlockB0[8] = header->slots_id[2];
    MicBlockB0[9] = header->slots_id[3];

    MicBlockB0[10] = header->fcnt & 0xFF;
    MicBlockB0[11] = ( header->fcnt >> 8 ) & 0xFF;
    MicBlockB0[12] = 0x00;
    MicBlockB0[13] = 0x00;

    MicBlockB0[15] = size;

    AES_CMAC_Init( AesCmacCtx );

    AES_CMAC_SetKey( AesCmacCtx, key );

    AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
    
    AES_CMAC_Update( AesCmacCtx, buffer, size );
    
    AES_CMAC_Final( Mic, AesCmacCtx );
    
    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
}

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint8_t *encBuffer )
{
    uint16_t i;
    uint8_t bufferIndex = 0;
    uint16_t ctr = 1;

    //memset1( AesContext.ksch, '\0', 240 );
    //aes_set_key( key, 16, &AesContext );
    aes_init(&AesContext, key, 16);

    aBlock[5] = dir;

    aBlock[6] = header->slots_id[0];;
    aBlock[7] = header->slots_id[1];;
    aBlock[8] = header->slots_id[2];;
    aBlock[9] = header->slots_id[3];;

    aBlock[10] = header->fcnt & 0xFF;
    aBlock[11] = ( header->fcnt >> 8 ) & 0xFF;
    aBlock[12] = 0x00;
    aBlock[13] = 0x00;

    while( size >= 16 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        ctr++;
        //aes_encrypt( aBlock, sBlock, &AesContext );
        aes_encrypt(&AesContext, aBlock, sBlock);
        for( i = 0; i < 16; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if( size > 0 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        //aes_encrypt( aBlock, sBlock, &AesContext );
        aes_encrypt(&AesContext, aBlock, sBlock);
        for( i = 0; i < size; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }
}

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, vemac_header_t* header, uint8_t dir, uint8_t *decBuffer )
{
    LoRaMacPayloadEncrypt( buffer, size, key, header, dir, decBuffer );
}

