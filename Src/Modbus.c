/**
  ******************************************************************************
  * File Name          : Modbus.c
  * Description        : This file provides code for Send/Receive functions
  *                      of the Modbus protocol.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Televis.h"
#include "Modbus.h"

uint8_t ModbusBuf[BUFLEN];
uint32_t ModbusTimeIsExpired;
//extern TEvent Event;
extern uint8_t ReplyCntr;
extern bool DeviceFound;

//-----------------------------------------------------------
static uint32_t ModbusCrc16(uint8_t *data, uint8_t len) 
{
  uint32_t w = 0;
  char shift_cnt;
  if (data) 
  {
    w = 0xffffU;
    for (; len > 0; len--) 
    {
      w = (uint32_t) ((w / 256U) * 256U + ((w % 256U) ^ (*data++)));
      for (shift_cnt = 0; shift_cnt < 8; shift_cnt++) 
      {
        if ((w & 0x1) == 1) w = (uint32_t) ((w >> 1) ^ 0xa001U);
        else w >>= 1;
      }
    }
  }
  return w;
}

//-----------------------------------------------------------
bool ModbusReceive(uint8_t *data)
{
  ModbusCmd *mb = (ModbusCmd*)ModbusBuf;
  uint32_t crc;
  uint8_t *crcptr, i, ptr, len;
	
  if(ptr == 0) return false; //no data

  len = ptr; ptr = 0; i = len-2;
  if(mb->addr != BROADCAST) return false;	//if wrong addr
  if(mb->cmd == ADDR)	//Set addr command
  {
    if( len < 5 ) return false;//short message
    crc = ModbusCrc16(ModbusBuf, i);//calculate crc
    crcptr = &ModbusBuf[i];	//pointer to crc in the message
    if(((crc&0xFF) == *crcptr) && ((crc>>8) == *(crcptr+1))) //verify crc
    {
      *data = ModbusBuf[2];//new addr of controller
      return true;
    }
  }
  return false;
}
bool TModbusReceive(TEvent *ev)//Request Called by App
{
  ModbusCmd *mb = (ModbusCmd*)ModbusBuf;
  ModbusCmd10 *cmd10 = (ModbusCmd10*)ModbusBuf;
  uint32_t crc;
  uint8_t cmd, *crcptr, n, ptr, len;

  if (ptr == 0) return false;//no data
  len = ptr; ptr = 0; n = len-2;
  if ((mb->addr != ev->addr) && (mb->addr != BROADCAST)) return false;//if wrong addr
  cmd = mb->cmd;
  //--------------------------------
  //44   03   C0 A8  00 01  B2 BC
  //Addr Cmd  Reg    Num    Crc
  if ((cmd==CMD03)||(cmd==CMD73))//multy reg read command
  { 
    if( len < 8 ) return false;//short message
    ev->len = mb->num << 1;//len alwais doubled
    crc = ModbusCrc16(ModbusBuf,n);//calculate crc
    crcptr = &ModbusBuf[n];//pointer to crc in the message
  }
  //--------------------------------
  //44   06   C0 A8  12 34  B2 BC
  //Addr Cmd  Reg    Data   Crc
  else if ((cmd==CMD06)||(cmd==CMD76))//one reg write command
  {
    if (len<7) return false;//short message
    crc = ModbusCrc16(ModbusBuf,n);//calculate crc
    crcptr = &ModbusBuf[n];//pointer to crc in the message
    ev->len = 2;
    //if (len==7) {ev->len=1; ev->size=0;} else {ev->len=2; ev->size=1;}//len depends on request len
    ev->dataptr = &ModbusBuf[4];
  }
  //--------------------------------
  //44   10  C0 A8  00 01  02  12 34  B2 BC
  //Addr Cmd Reg    Num    Len Data   Crc
  else if (cmd==CMD10)//multy reg write command
  {
    if (len<10) return false;//short message
    crc = ModbusCrc16(ModbusBuf,n);//calculate crc
    crcptr = &ModbusBuf[n];//pointer to crc in the message
    //if (cmd10->len == cmd10->num) {ev->len = cmd10->len; ev->size=0;}//len depends on data len
    //else {ev->len = cmd10->num<<1; ev->size=1;}
    ev->len = cmd10->num<<1;
    ev->dataptr = &ModbusBuf[7];
  }
  //-------------------------------
  //44   2B  00  01 02  B2 BC
  //Addr Cmd MEI Data   Crc
  else if (cmd == CMD43)
  {
    if (len<5) return false;//short message
    ev->dataptr = &ModbusBuf[2];
    ev->len = 1;
    crc = ModbusCrc16(ModbusBuf,n);//calculate crc
    crcptr = &ModbusBuf[n];//pointer to crc in the message
  }
  //-------------------------------
  else if (cmd == DEBUG)
  {
    if (len<5) return false;//short message
    crc = ModbusCrc16(ModbusBuf,n);//calculate crc
    crcptr = &ModbusBuf[n];//pointer to crc in the message
  }
  //-------------------------------
  else
	{		
		return false;//unknown command
	}
	
  if ((crc&0xFF) == *crcptr && (crc>>8) == *(crcptr+1))//verify crc
  {
    ev->cmd = cmd;
    ev->reg = mb->reg;
    if (cmd == DEBUG) ev->debug = ModbusBuf[2];
    if (mb->addr == BROADCAST) return false;

//    if(ev->debug == NEARLOOP) {Send(); return FALSE;}
//    if (cmd == DEBUG) {Send(); return FALSE;}
    return true;
  }
  return false;
}

void TModbusSend(TEvent *ev)//Reply Called by App
{
  uint32_t crc;
  uint8_t cmd, i, n, len, ptr;
	
  ModbusReply03 *r03=(ModbusReply03*)ModbusBuf;
  r03->addr = ev->addr;
  cmd = ev->cmd;
  r03->cmd = cmd;
  //--------------------------------
  //44   03   02  00 C2   F4 1A
  //Addr Cmd Len  Data    Crc
  if((cmd==CMD03)||(cmd==CMD73))
  {
    if ((n=ev->len) > 20) n=2;
    r03->len = n;
    //if (ev->reg & 0xC000) x=1; else x=0;
    for (i=0; i<n; i++) *(&r03->data+(i^1)) = *(ev->dataptr+i);//need to swap bytes
    len = n+3;
  }
  //--------------------------------
  //44   06   C0 A8  12 34  B2 BC
  //Addr Cmd  Reg    Data   Crc
  else if((cmd==CMD06)||(cmd==CMD76))//Should return the same reply as request
  {
    //ModbusReply06 *r06=(ModbusReply06*)ModbusBuf;
    //r06->reg = ev->reg;
    //for (i=0; i<2; i++) *(&r06->data+i) = *(ev->dataptr+i);
    //if(ev->len == 1) len = 5; else 
    len = 6;
  }
  //--------------------------------
  //44   10   C0 A8  00 01  B2 BC
  //Addr Cmd  Reg    Num    Crc
  else if(cmd==CMD10) 
  {
    ModbusReply10 *r10=(ModbusReply10*)ModbusBuf;
    r10->reg = ev->reg;
    r10->num = ev->len;
    len = 6;
  }
  //--------------------------------
  else if(cmd==CMD43)
  {
    for(i=0; i<ev->len; i++) ModbusBuf[i+3] = *(ev->dataptr+i);
    len = ev->len+3;
  }
  //--------------------------------
  else len=1;
  ptr=1;
  crc = ModbusCrc16(ModbusBuf, len);//Calculate CRC
  ModbusBuf[len++] = crc;//Add CRC to buf
  ModbusBuf[len++] = crc>>8;
}
/************************ (C) COPYRIGHT *****END OF FILE****/
