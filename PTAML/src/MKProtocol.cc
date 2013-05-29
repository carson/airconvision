/*
 * MikroKopter Protocol
 * This code is based on the MikroKopter NaviCtrl firmware, Copyright MikroKopter.
 */

#include "MKProtocol.h"

#include <cstdlib>
#include <cstdarg>
#include <cstring>

namespace PTAMM {

void Buffer_Clear(Buffer_t* pBuffer)
{
    pBuffer->DataBytes = 0;
    pBuffer->Position = 0;
    pBuffer->Locked = false;
}

void Buffer_Init(Buffer_t* pBuffer, uint8_t* pDataBuffer, uint16_t DataBufferSize)
{
    pBuffer->pData = pDataBuffer;
    pBuffer->Size = DataBufferSize;
    Buffer_Clear(pBuffer);
}

uint8_t Buffer_Copy(Buffer_t* pSrcBuffer, Buffer_t* pDstBuffer)
{
    uint8_t retval = 0;

    if( (pSrcBuffer != NULL) && (pDstBuffer != NULL) )
    {
        if(pSrcBuffer->Locked && !(pDstBuffer->Locked) && (pDstBuffer->Size >= pSrcBuffer->DataBytes))
        {
            memcpy(pDstBuffer->pData, pSrcBuffer->pData, pSrcBuffer->DataBytes);
            pDstBuffer->DataBytes = pSrcBuffer->DataBytes;
            pDstBuffer->Position = 0;
            pDstBuffer->Locked = true;
            retval = 1;
        }
    }
    return retval;
}


/**************************************************************/
/* Create serial output frame                                 */
/**************************************************************/
uint8_t MKProtocol_CreateSerialFrame(Buffer_t* pTxBuff, uint8_t CmdID,
		uint8_t Address, uint8_t numofbuffers , ...) //uint8_t *data, uint8_t len, ....
{
	va_list ap;

	uint8_t a,b,c;
	uint16_t ptr = 0;
	uint16_t tmpCRC = 0, i;

	uint8_t* pdata = NULL;
	int len = 0;
	
	if(pTxBuff->Locked) return(0);

	// tx-buffer is not in use
	// lock the buffer
	pTxBuff->Locked = true;
	pTxBuff->Position = 0;
	pTxBuff->pData[pTxBuff->Position++] = '#';			    // Start character
	pTxBuff->pData[pTxBuff->Position++] = 'a' + Address;	// Address (a=0; b=1,...)
	pTxBuff->pData[pTxBuff->Position++] = CmdID;			// Command

	va_start(ap, numofbuffers);
	if(numofbuffers)
	{
		pdata = va_arg(ap, uint8_t*);
		len = va_arg(ap, int);
		ptr = 0;
		numofbuffers--;
	}
	while(len)
	{
		if(len)
		{
			a = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else a = 0;
		if(len)
		{
			b = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else b = 0;
		if(len)
		{
			c = pdata[ptr++];
			len--;
			if((!len) && numofbuffers) // try to jump to next buffer
			{
				pdata = va_arg(ap, uint8_t*);
				len = va_arg(ap, int);
				ptr = 0;
				numofbuffers--;
			}
		}
		else c = 0;
		pTxBuff->pData[pTxBuff->Position++] = '=' + (a >> 2);
		pTxBuff->pData[pTxBuff->Position++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
		pTxBuff->pData[pTxBuff->Position++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
		pTxBuff->pData[pTxBuff->Position++] = '=' + ( c & 0x3f);
	}
	va_end(ap);
	// add crc
	for(i = 0; i < pTxBuff->Position; i++)
	{
		tmpCRC += pTxBuff->pData[i];
	}
	tmpCRC %= 4096;
	pTxBuff->pData[pTxBuff->Position++] = '=' + tmpCRC / 64;
	pTxBuff->pData[pTxBuff->Position++] = '=' + tmpCRC % 64;
	pTxBuff->pData[pTxBuff->Position++] = '\r';
	pTxBuff->DataBytes = pTxBuff->Position;
	pTxBuff->Position = 0;  // reset buffer position for transmision
	return(pTxBuff->Locked);
}

// typical called in an UART Rx ISR
/**************************************************************/
/* Collect serial frame                                       */
/**************************************************************/
uint8_t MKProtocol_CollectSerialFrame(Buffer_t* pRxBuff, uint8_t c)
{
	if(!pRxBuff->Locked)
	{ // rx buffer not locked
		if(c == '#') // if syncronisation character is received
		{
			pRxBuff->Position = 0;						// reset buffer
			pRxBuff->pData[pRxBuff->Position++] = c;	// copy 1st byte to buffer
			pRxBuff->DataBytes = 1;
		}
		else if (pRxBuff->Position < pRxBuff->Size) // rx buffer not full
		{
			pRxBuff->pData[pRxBuff->Position++] = c; // copy byte to rxd buffer
			pRxBuff->DataBytes++;
			// termination character received and sync has been established
			if ((c == '\r') && (pRxBuff->pData[0]== '#'))
			{
				// calculate checksum from transmitted data
				uint16_t crc = 0, i;
				uint8_t crc1, crc2;
				for(i = 0; i < (pRxBuff->Position-3); i++)
				{
					crc +=  pRxBuff->pData[i];  
				}		
				crc %= 4096;
				crc1 = '=' + crc / 64;
				crc2 = '=' + crc % 64;
				// compare checksum to transmitted checksum bytes
				if((crc1 == pRxBuff->pData[pRxBuff->Position-3]) && (crc2 == pRxBuff->pData[pRxBuff->Position-2]))
				{
				    // checksum is valid
					pRxBuff->Position = 0;					
					pRxBuff->Locked = true;          			    // lock the rxd buffer
				} // eof checksum valid
				else
				{	// checksum is invalid
					Buffer_Clear(pRxBuff);
				}  // eof checksum invalid
			} // eof termination character received
		} // rxd buffer not full
		else // rxd buffer overrun
		{
			Buffer_Clear(pRxBuff);
		} // eof rxd buffer overrun
	}
	return(pRxBuff->Locked);
}

/**************************************************************/
/* Decode destination address                                  */
/**************************************************************/
void MKProtocol_DecodeSerialFrameHeader(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg)
{
	if(pRxBuff->Locked)
	{
		pSerialMsg->Address = pRxBuff->pData[1] - 'a';
		pSerialMsg->CmdID = pRxBuff->pData[2];
	}
	else
	{
		pSerialMsg->Address = 0;
		pSerialMsg->CmdID = ' ';
	}	
}

/**************************************************************/
/* Decode data                                                */
/**************************************************************/
void MKProtocol_DecodeSerialFrameData(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg)
{
	uint8_t a,b,c,d;
	uint8_t x,y,z;
	uint16_t ptrIn = 3; // start with first data byte in rx buffer
	uint16_t ptrOut = 3;
	uint16_t len = pRxBuff->DataBytes - 6;	 // must be a multiple of 4 (3 bytes at begin and 3 bytes at end are no payload )
	while(len)
	{
		a = pRxBuff->pData[ptrIn++] - '=';
		b = pRxBuff->pData[ptrIn++] - '=';
		c = pRxBuff->pData[ptrIn++] - '=';
		d = pRxBuff->pData[ptrIn++] - '=';
		//if(ptrIn > ReceivedBytes - 3) break;

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if(len--) pRxBuff->pData[ptrOut++] = x; else break;
		if(len--) pRxBuff->pData[ptrOut++] = y; else break;
		if(len--) pRxBuff->pData[ptrOut++] = z; else break;
	}
	pSerialMsg->pData = &(pRxBuff->pData[3]);
	pSerialMsg->DataLen = ptrOut - 3;	// return number of data in bytes
	pRxBuff->Position = 0;
	pRxBuff->DataBytes = ptrOut;
}

}

