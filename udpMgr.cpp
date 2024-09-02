#include <WiFiUdp.h>
#include "udpMgr.h"
#include "CmdHandler.h"


#include <string>



using namespace std;

WiFiUDP Udp;



IPAddress remote_ip(192, 168, 2, 249);
uint16_t  remote_port = 4000;

IPAddress dl_ip(192, 168, 2, 249); //datalogger ip

void udpMgr_receive();
void udpMgr_broadcast();


void setup_udpMgr()
{
     //This initializes udp and transfer buffer
    
    Udp.begin(remote_port);

    udpMgr_broadcast();
}

void loop_udpMgr()
{
    udpMgr_receive();
}

//IPAddress remote_ip(192, 168, 2, 72);
//uint16_t remote_port = 4000;

void udpMgr_send(string sMsg)
{
    uint32_t dwIp =  (uint32_t)remote_ip;
    if(dwIp == 0)
    {
        return;
    }

    Udp.beginPacket(remote_ip, remote_port);

    Udp.write((const uint8_t *)sMsg.c_str(), sMsg.length());

    Udp.endPacket();
}

void udpMgr_send(char* pBuff, uint nLenght)
{
    

    Udp.beginPacket(remote_ip, remote_port);

    Udp.write((const uint8_t *)pBuff, nLenght);

    Udp.endPacket();
}

void udpMgr_debug_send(std::string sMsg)
{
    
    Udp.beginPacket(dl_ip, remote_port);

    Udp.write((const uint8_t *)sMsg.c_str(), sMsg.length());

    Udp.endPacket();
}

void udpMgr_broadcast()
{
    

    IPAddress ip_broadcast(192, 168, 2, 255); //broadcast 


    Udp.beginPacket(ip_broadcast, remote_port);

    string sMsg = "EspDrone_Ard";

    Udp.write((const uint8_t *)sMsg.c_str(), sMsg.length());

    Udp.endPacket();
}


uint8_t buffer[500];
void udpMgr_receive()
{
    //processing incoming packet, must be called before reading the buffer
    int nSize = Udp.parsePacket();
    if(nSize == 0)
    {
        return;
    }
    //receive response from server, it will be HELLO WORLD
    memset(buffer,0,500);
    nSize = Udp.read(buffer, 500);
    if (nSize > 0)
    {
        Serial.print(" udp new message: ");
        Serial.println((char *)buffer);
        string sMsg = (char*)buffer;
        remote_ip = Udp.remoteIP();
        //remote_port = Udp.remotePort();

        CmdHandler_SetNewCommand(sMsg);
        
    }
}