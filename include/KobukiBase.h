#pragma once
#include <stdint.h>
#include "Kobuki.h"
#include "Thread.h"
#include "Translator.h"
#include "SerialPort.h"
#include "Transport.h"

#define BAUDRATE 115200

/*
#define GPIO_CH0 0
#define GPIO_CH1 1
#define GPIO_CH2 2
#define GPIO_CH3 3

#define POWER33V   0
#define POWER50V   1
#define POWER12V1A 2
#define POWER12V5A 3

#define RED   0x00
#define GREEN 0x01
*/
#define LED1 0
#define LED2 1


namespace rt_net {
  class Translator;

  class LIBKOBUKI_API KobukiBase : public net::ysuga::Thread {
  private:
    Translator *m_pTranslator;
    Transport  *m_pTransport;
    net::ysuga::SerialPort *m_pSerialPort;
    bool m_Endflag;
    uint8_t m_Interval;

  protected:
    uint8_t m_Bumper;
    uint8_t m_WheelDrop;
    uint8_t m_Cliff;
    uint16_t m_LeftEncoder;
    uint16_t m_RightEncoder;
    uint8_t m_LeftPWM;
    uint8_t m_RightPWM;
    uint8_t m_Button;
    uint8_t m_Charger;
    uint8_t m_Battery;
    uint8_t m_OverCurrentFlag;

    uint8_t m_DockRightSignal;
    uint8_t m_DockCenterSignal;
    uint8_t m_DockLeftSignal;

#define BUFFER_LENGTH 3
    uint8_t m_DockSignalBufferCount;
    uint8_t m_DockRightSignalBuffer[5];
    uint8_t m_DockCenterSignalBuffer[5];
    uint8_t m_DockLeftSignalBuffer[5];

    uint16_t m_InertialAngle;
    uint16_t m_InertialAngleRate;

    uint16_t m_BaseControlSpeed;
    uint16_t m_BaseControlRadius;
    
    uint8_t m_DigitalOut;
    uint8_t m_ExtPower;
    uint8_t m_LED;
   
    double m_LeftMotorCurrent;
    double m_RightMotorCurrent;
    double m_ADIN[4];
    uint16_t m_GPIN;
  public:
    KobukiBase(const char* filename);
    virtual ~KobukiBase();

  public:
    virtual void Run();

    friend class Translator;

    virtual void onUpdate() {}
    virtual void onPreSendCommand(){}
    virtual void onPostSendCommand(){}
  protected:

    void privateBaseControl(const uint16_t speed_mm_per_sec, const uint16_t radius_mm);
    void privateGPIOControl(const uint8_t digitalOutput, const uint8_t externalPower, const uint8_t led);

  protected:
    void baseControl(const uint16_t speed_mm_per_sec, const uint16_t radius_mm);


    void setGPIO(const uint8_t ch, const bool flag) {
      if(flag) {
	m_DigitalOut |= 0x01 << ch;
      } else {
	m_DigitalOut &= ~(0x01 << ch);
      }
    }
    
    void setPower(const uint8_t ch, const bool flag) {
      if(flag) {
	m_ExtPower |= 0x01 << ch;
      } else {
	m_ExtPower &= ~(0x01 << ch);
      }
    }
      
    void setLED(const uint8_t ch, const uint8_t color, const bool flag) {
      uint8_t NNNNXotemp = m_LED;
      uint8_t mask = 0x01 << (ch*2+color);
      if(flag) {
	m_LED |= mask;
      } else {
	m_LED &= ~mask;
      }
    }

  };

}
