/*
 * MikroKopter Protocol
 * This code is based on the MikroKopter NaviCtrl firmware, Copyright MikroKopter.
 */

#ifndef __MKPROTOCOL_H
#define __MKPROTOCOL_H

#include <cstdint>

namespace PTAMM {

// slave addresses
#define ANY_ADDRESS 0
#define FC_ADDRESS 1
#define NC_ADDRESS 2
#define MK3MAG_ADDRESS 3
#define MKOSD_ADDRESS 4
#define BL_ADDRESS 5
#define OBC_ADDRESS 6 // On-Board Computer

typedef struct
{
    uint8_t* pData;
    uint16_t Size;
    uint16_t DataBytes;
    uint16_t Position;
    uint8_t  Locked;
} __attribute__((packed)) Buffer_t;

void Buffer_Init(Buffer_t* pBuffer, uint8_t* pDataBuffer, uint16_t DataBufferSize);
void Buffer_Clear(Buffer_t* pBuffer);
uint8_t Buffer_Copy(Buffer_t* pSrcBuffer, Buffer_t* pDstBuffer);

typedef struct
{
	uint8_t Address;
	uint8_t CmdID;
 	uint8_t* pData;
	uint16_t DataLen;
} __attribute__((packed)) SerialMsg_t;

uint8_t MKProtocol_CollectSerialFrame(Buffer_t* pRxBuff, uint8_t c);
uint8_t MKProtocol_CreateSerialFrame(Buffer_t* pTxBuff, uint8_t CmdID, uint8_t Address, uint8_t numofbuffers , ...); //uint8_t *data, uint8_t len, ....;
void MKProtocol_DecodeSerialFrameHeader(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg);
void MKProtocol_DecodeSerialFrameData(Buffer_t* pRxBuff, SerialMsg_t* pSerialMsg);

}

#endif // __MKPROTOCOL_H
