#include <iostream>
#include "Transport.h"

using namespace rt_net;

Transport::Transport(net::ysuga::SerialPort* pSerialPort) :
  m_pSerialPort(pSerialPort)
{

}

Transport::~Transport()
{

}

Packet Transport::receive()
{
  int retval;
 transport_receive_start:
  while(true) {
    uint8_t c = 0;
    retval = m_pSerialPort->Read(&c, 1);
    if(c == PACKET_HEADER_ZERO) {
      break;
    }
  }
  while(true) {
    uint8_t c = 0;
    m_pSerialPort->Read(&c, 1);
    if(c == PACKET_HEADER_ONE) {
      break;
    } 
    goto transport_receive_start;
  }

  uint8_t length;
  do {
    retval = m_pSerialPort->Read(&length, 1);
  } while(retval != 1);
  
  Packet packet(length);
  uint8_t* pBuffer = packet.getBuffer();
  for(int i = 0;i < length;i++) {
    do {
      retval = m_pSerialPort->Read(pBuffer+i, 1);
    } while(retval != 1);
  }
  
  uint8_t checksum;
  do {
    retval = m_pSerialPort->Read(&checksum, 1);
  } while(retval != 1);
  
  uint8_t sum = 0;
  sum ^= length;
  for(int i = 0;i < length;i++) {
    sum ^= pBuffer[i];
  }
  sum ^= checksum;
  if(sum != 0) {
	  m_pSerialPort->FlushRxBuffer();
    throw CheckSumError();
  }

  return packet;
}

void Transport::transmit(const Packet& packet)
{
  uint8_t c = PACKET_HEADER_ZERO;
  m_pSerialPort->Write(&c, 1);
  c = PACKET_HEADER_ONE;
  m_pSerialPort->Write(&c, 1);
  uint8_t sum = 0;
  c = packet.length();
  m_pSerialPort->Write(&c, 1);
  sum ^= c;
  for(int i = 0;i < packet.length();i++) {
    m_pSerialPort->Write(&(packet[i]), 1);
    sum ^= packet[i];
  }
  m_pSerialPort->Write(&sum, 1);
}
