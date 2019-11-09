#include <iostream>

#include "KobukiBase.h"

using namespace rt_net;
using namespace net::ysuga;

KobukiBase::KobukiBase(const char* filename):
  m_Endflag(false),
  m_Interval(1),
  m_Bumper(0),
  m_WheelDrop(0),
  m_Cliff(0),
  m_LeftEncoder(0),
  m_RightEncoder(0),
  m_LeftPWM(0),
  m_RightPWM(0),
  m_Button(0),
  m_Charger(0),
  m_Battery(0),
  m_OverCurrentFlag(0),
  m_InertialAngle(0),
  m_InertialAngleRate(0),
  m_BaseControlSpeed(0),
  m_BaseControlRadius(0),
  m_DigitalOut(0),
  m_ExtPower(15),
  m_LED(0)
{
  m_pSerialPort = new SerialPort(filename, BAUDRATE);
  m_pTransport = new Transport(m_pSerialPort);
  m_pTranslator = new Translator(this);

  m_DockSignalBufferCount = 0;
  for(int i = 0;i < BUFFER_LENGTH;i++) {
    m_DockRightSignalBuffer[i] = 0;
    m_DockCenterSignalBuffer[i] = 0;
    m_DockLeftSignalBuffer[i] = 0;
  }
  Start();
}

KobukiBase::~KobukiBase()
{
  m_Endflag = true;
  Join();
  privateBaseControl(0, 0);
  delete m_pTranslator;
  delete m_pTransport;
  delete m_pSerialPort;
}

void KobukiBase::Run()
{
  m_Endflag = false;
  std::cout << "KobukiBase::Starting Background Job." << std::endl;
  while(!m_Endflag) {
    try {
      Packet p = m_pTransport->receive();
      m_pTranslator->translate(p);
      onUpdate();

      onPreSendCommand();
      privateBaseControl(m_BaseControlSpeed, m_BaseControlRadius);
      privateGPIOControl(m_DigitalOut, m_ExtPower, m_LED);
      onPostSendCommand();

      Thread::Sleep(m_Interval); // [ms]
    } catch (TimeOutException& e) {
      std::cerr << "Packet Receiver -- Timeout" << std::endl;
    } catch (CheckSumError& e) {
    }
  }
}

void KobukiBase::baseControl(const uint16_t speed_mm_per_sec, const uint16_t radius_mm) {
    m_BaseControlSpeed = speed_mm_per_sec;
    m_BaseControlRadius = radius_mm;
}

void KobukiBase::privateBaseControl(const uint16_t speed_mm_per_sec, const uint16_t radius_mm) {
    Packet packet(6);
    packet[0] = 1;
    packet[1] = 4;
    packet.ushort(2, speed_mm_per_sec);
    packet.ushort(4, radius_mm);
    m_pTransport->transmit(packet);
}

void KobukiBase::privateGPIOControl(const uint8_t digitalOutput, const uint8_t externalPower, const uint8_t led)
{
  Packet packet(4);
  packet[0] = 12;
  packet[1] = 2;
  packet.ushort(2, (digitalOutput | (externalPower << 4)) | (led << 8));
  m_pTransport->transmit(packet);
}
