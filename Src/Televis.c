/**
  ******************************************************************************
  * File Name          : Televis.c
  * Description        : This file provides code for Send/Receive functions
  *                      of the Televis protocol.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Televis.h"

uint8_t TelevisBuf[BUFLEN];
uint32_t TelevisTimeIsExpired;
extern TEvent Event;
extern uint8_t ReplyCntr;
extern bool DeviceFound;

//-----------------------------------------------------------
static uint32_t TelevisCrc16(uint8_t *data, uint8_t lenght) 
{
  uint32_t crc=0xFFFF;
  for (uint8_t i=0; i < lenght; i++) crc -= data[i];
  return crc;
}

//-----------------------------------------------------------
bool TelevisReceive(TEvent *ev)
{
  uint32_t crc;
  uint8_t n, cmd, *crcptr, ptr, len;
	bool ready;
	
  TelevisReadReply *trr=(TelevisReadReply*)TelevisBuf;
//  if (ev->debug == FARLOOP1) {timer.Stop(); return FALSE;}

  if(TelevisTimeIsExpired){ //no response
    if (!ReplyCntr){
			Event.addr--; 
			ReplyCntr = REPLYCNTR; 
			DeviceFound = false;
		} else {
			ReplyCntr--;
		}
 
    return false;
  }
	
  if(ptr == 0){
		return false;//no data
	}

  ReplyCntr = REPLYCNTR;//Reply got, counting again
  len=ptr; ptr=0; n = len-2;
  if(trr->header != 0x82){
		return false;//wrong header
	}
  if(trr->sender != ev->addr){
		return false;//wrong addr
	}
  cmd = ev->cmd;
  //--------------------------------
  //82  13  44  31  03  00 00  C2   FE 30 - read response
  //Hdr Cmd Src Dst Len Id     Data Crc
  if ((cmd==CMD03)||(cmd==CMD73))//Read response
  {
    if (trr->cmd != 0x13 || len < 10) return false;//short message
    ev->len = trr->len-2;
    ev->dataptr = &trr->data;
    crcptr = &TelevisBuf[n];//pointer to crc in the message
    crc = TelevisCrc16(TelevisBuf, n);//calculate crc
  }
  //--------------------------------
  //82  05  44  31  00  FF 03 - write response
  //Hdr Cmd Src Dst Len Crc
  else if (cmd==CMD06 || cmd==CMD10||(cmd==CMD76))//Write response
  {
    if (trr->cmd != 0x05 || len < 7) return false;//short message
    crcptr = &TelevisBuf[n];//pointer to crc in the message
    crc = TelevisCrc16(TelevisBuf, n);//calculate crc
  }
  //--------------------------------
  else if (cmd == CMD43)//Device ID response
  {
    //if (trr->cmd != 0x56 || len < 7) return FALSE;//short message
    ev->dataptr = &TelevisBuf[5];
    ev->len = trr->len;
    crcptr = &TelevisBuf[n];//pointer to crc in the message
    crc = TelevisCrc16(TelevisBuf, n);//calculate crc
  }
  //--------------------------------
  //82  56  33  EE  03  02 04 02  FD FB - scan response
  //Hdr Cmd Src Dst Len Data      Crc
  else if (cmd == SCAN)//Scan response
  {
    //if (trr->cmd != 0x56 || len < 7) return FALSE;//short message
    ev->dataptr = &TelevisBuf[5];
    ev->len = 3;//trr->len;
    crcptr = &TelevisBuf[n];//pointer to crc in the message
    crc = TelevisCrc16(TelevisBuf, n);//calculate crc
  }
  //--------------------------------
  else return false;//unknown command
  if ((crc>>8) != *crcptr || (crc&0xFF) != *(crcptr+1)) return false;//wrong crc

//  if (ev->debug == FARLOOP2) {Modbus.Send(TelevisBuf, len); return false;}
  return true;
}
//-----------------------------------------------------------
void TelevisSend(TEvent *ev)
{
  uint8_t i, n, cmd=ev->cmd, ptr, len;
  uint32_t crc;

  TelevisReadRequest  *trr=(TelevisReadRequest*)TelevisBuf;
  TelevisWriteRequest *twr=(TelevisWriteRequest*)TelevisBuf;
  trr->header = 0x82;
  trr->sender = 0;
  trr->receiver = ev->addr;
  //--------------------------------
  //82  92  31  44  05  10 A8  01   00 00  FD E8
  //Hdr Cmd Src Dst Len Reg    Num  Id     Crc
  if ((cmd==CMD03) ||(cmd==CMD73))
  {
    if(cmd==CMD03)trr->cmd = 0x92; else trr->cmd = 0x12;
    trr->len = 5;
    trr->regH = ev->reg>>8;
    trr->regL= ev->reg&0xFF;
    trr->num = ev->len;
    trr->id  = 0;
    len = 10;
  }
  //--------------------------------
  //82  93  31  44  04  10 02  32 00  FE 2D
  //Hdr Cmd Src Dst Len Reg    Data   Crc
  else if (cmd==CMD06 || cmd==CMD10)
  {
    if((n=ev->len)>20) return;
    twr->cmd  = 0x93;
    twr->len  = n+2;
    twr->regH = ev->reg>>8;
    twr->regL = ev->reg;
    //x=ev->size;//if byte size no need to swap
    //for (i=0; i<n; i++) *(&twr->data+(i^x)) = *(ev->dataptr+i);//^x means swap bytes
    for (i=0; i<n; i++) *(&twr->data+(i^1)) = *(ev->dataptr+i);//^1 because need to swap bytes
    len = n+7;
  }
  else if(cmd==CMD76)
  {
    twr->cmd  = 0x93;
    twr->len  = 3;
    twr->regH = ev->reg>>8;
    twr->regL = ev->reg;
    twr->data = *(ev->dataptr+1);//+1 because we send LSB only
    len = 8;
  }
  //--------------------------------
  //82  D6  EE  33  01  02   FD 83
  //Hdr Cmd Src Dst Len Data Crc
  else if (cmd == CMD43)
  {
    twr->cmd = SCAN;
    twr->sender = 0;
    twr->receiver = ev->addr;
    twr->len = 0x01;//Len
    twr->regH = *ev->dataptr;//0x02;
    len = 6;
  }
  else if (cmd == SCAN)
  {
    twr->cmd = SCAN;
    twr->sender = 0;
    twr->receiver = ev->addr;
    TelevisBuf[4] = 0x01;
    TelevisBuf[5] = 0x02;
    len = 6;
  }
  //--------------------------------
  else len = 1;
  crc = TelevisCrc16(TelevisBuf, len);
  TelevisBuf[len++] = crc>>8;//Add CRC to buf
  TelevisBuf[len++] = crc&0xFF;
  ptr = 1;
//  if(ev->debug == FARLOOP1) Modbus.Send(TelevisBuf, len);
}
//-----------------------------------------------------------
/************************ (C) COPYRIGHT *****END OF FILE****/
