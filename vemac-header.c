#include "include/vemac-header.h"


uint32_t  deserialize (uint8_t *message, vemac_header_t * header)
{
    uint32_t byte_read = 0;

    header->mHdr = message[0];
    byte_read++;

    for (int i = 0; i < 4; ++i)
    {
        header->slots_id [i] = message[i+1];
        byte_read++;
    }

    header->fCtrl = message[5];

    header->fcnt = message[6] | message[7];
    byte_read += 2;

    for (int i = 4; i < 10; ++i)
    {
    	header->slots_id[i] = message[i+4];
    	byte_read++;
    }

    return byte_read;
}

void serialize (vemac_header_t * header, uint8_t *message)
{
	message[0] = 0xE0;

	for (int i = 0; i < 4; ++i)
	{
		message[i+1] = header->slots_id[i];
	}

	message[5] = 0x06;

	message[6] = (header->fcnt & 0xff);
	message[7] = (header->fcnt >> 8) & 0xff;

	for (int i = 4; i < 10; ++i)
	{
		message[i+4] = header->slots_id[i];
	}
}