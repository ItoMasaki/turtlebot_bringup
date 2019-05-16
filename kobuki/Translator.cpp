#include <iostream>
#include "KobukiBase.h"
#include "Translator.h"

using namespace rt_net;

Translator::Translator(KobukiBase* pKobuki) :
  m_pKobuki(pKobuki)
{

}

Translator::~Translator()
{

}

std::vector<Packet> Translator::split(const Packet& packet)
{
  std::vector<Packet> packets;
  uint8_t index = 0;
  uint8_t packet_length = packet.length();
  while(index < (packet_length-1)) {
    uint8_t data_length = packet[index+1];
    Packet pac(data_length+2);
    pac[0] = packet[index];
    pac[1] = packet[index+1];
    for(int i = 0;i < pac[1];i++) {
      pac[2+i] = packet[index+2+i];
    }
    packets.push_back(pac);
    index += pac[1] + 2;
  }
  return packets;
}

void Translator::translate(const Packet& packet)
{
  std::vector<Packet> packets = split(packet);
  std::vector<Packet>::iterator it = packets.begin();
  for(;it != packets.end();++it) {
    switch((*it).uchar(0)) {
    case BASIC_SENSOR_DATA:
      onBasicSensorData(*it);
      break;
    case DOCKING_IR:
      onDockingIR(*it);
      break;
    case INERTIAL_SENSOR:
      onInertialSensor(*it);
      break;
    case CLIFF:
      onCliff(*it);
      break;
    case CURRENT:
      onCurrent(*it);
      break;
    case RAWDATA_OF_3AXIS_GYRO:
      onRawDataOf3AxisGyro(*it);
      break;
    case GENERAL_PURPOSE_INPUT:
      onGeneralPurposeInput(*it);
      break;
    default:
      std::cout << "Unknown packet " << (int)(*it).uchar(0) << std::endl;
      break;
    }
  }
}


void Translator::onBasicSensorData(const Packet& packet)
{
  uint8_t size = packet[1];
  uint16_t timestamp = packet.ushort(2);
  m_pKobuki->m_Bumper      = packet[4];
  m_pKobuki->m_WheelDrop   = packet[5];
  m_pKobuki->m_Cliff       = packet[6];
  m_pKobuki->m_LeftEncoder = packet.ushort(7);
  m_pKobuki->m_RightEncoder= packet.ushort(9);
  m_pKobuki->m_LeftPWM     = packet[11];
  m_pKobuki->m_RightPWM    = packet[12];
  m_pKobuki->m_Button      = packet[13];
  m_pKobuki->m_Charger     = packet[14];
  m_pKobuki->m_Battery     = packet[15];
  m_pKobuki->m_OverCurrentFlag = packet[16];
}

void Translator::onDockingIR(const Packet& packet)
{
  uint8_t dockSignalBufferCount = m_pKobuki->m_DockSignalBufferCount;
  m_pKobuki->m_DockRightSignalBuffer[dockSignalBufferCount] = packet[2];
  m_pKobuki->m_DockCenterSignalBuffer[dockSignalBufferCount] = packet[3];
  m_pKobuki->m_DockLeftSignalBuffer[dockSignalBufferCount] = packet[4];
  m_pKobuki->m_DockSignalBufferCount = dockSignalBufferCount == 4 ? 0 : dockSignalBufferCount + 1;
  
  uint8_t right = 0;
  uint8_t center = 0;
  uint8_t left = 0;
  for(int i = 0;i < BUFFER_LENGTH;i++) {
    right |= m_pKobuki->m_DockRightSignalBuffer[i];
    center |= m_pKobuki->m_DockCenterSignalBuffer[i];
    left |= m_pKobuki->m_DockLeftSignalBuffer[i];
  }
  
  m_pKobuki->m_DockRightSignal = right;
  m_pKobuki->m_DockLeftSignal = left;
  m_pKobuki->m_DockCenterSignal = center;
}

void Translator::onInertialSensor(const Packet& packet)
{
  m_pKobuki->m_InertialAngle     = packet.ushort(2);
  m_pKobuki->m_InertialAngleRate = packet.ushort(4);
}

void Translator::onCliff(const Packet& packet)
{
}

void Translator::onCurrent(const Packet& packet)
{
  m_pKobuki->m_LeftMotorCurrent = packet.ushort(2) / 100.0;
  m_pKobuki->m_RightMotorCurrent = packet.ushort(4) / 100.0;
}

void Translator::onRawDataOf3AxisGyro(const Packet& packet)
{
}

void Translator::onGeneralPurposeInput(const Packet& packet)
{
  m_pKobuki->m_GPIN = packet.ushort(2);
  m_pKobuki->m_ADIN[0] = packet.ushort(4) * 3.3 / 4095.0;
  m_pKobuki->m_ADIN[1] = packet.ushort(4) * 3.3 / 4095.0;
  m_pKobuki->m_ADIN[2] = packet.ushort(4) * 3.3 / 4095.0;
  m_pKobuki->m_ADIN[3] = packet.ushort(4) * 3.3 / 4095.0;
}
