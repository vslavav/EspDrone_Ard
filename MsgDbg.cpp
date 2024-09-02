#include "MsgDbg.h"
#include "udpMgr.h"
#include "Arduino.h"

static void MsgDbg_printSerial(char* pMsg);
static void MsgDbg_printUdp(char* pMsg);

bool isSerialAvail = true;
bool isUdpAvail = true;


void MsgDbg_printMsg(char* pMsg)
{
  MsgDbg_printSerial( pMsg );
  MsgDbg_printUdp( pMsg );
}

void MsgDbg_printMsgLn(char* pMsg)
{

}

void MsgDbg_printLn()
{

}

void MsgDbg_printSerial(char* pMsg)
{
    if(isSerialAvail == false)
    {
        return;
    }
    Serial.print(pMsg);
}

void MsgDbg_printUdp(char* pMsg)
{
    if(isUdpAvail == false)
    {
      return;
    }
    udpMgr_debug_send(pMsg);
}