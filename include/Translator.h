#pragma once

#include <vector>
#include "Packet.h"

namespace rt_net {
  class KobukiBase;

  class Translator {
  private:
    KobukiBase *m_pKobuki;
    enum FEEDBACK_ID {
      BASIC_SENSOR_DATA = 1,
      DOCKING_IR = 3,
      INERTIAL_SENSOR = 4,
      CLIFF = 5,
      CURRENT = 6,
      HARDWARE_VERSION = 10,
      FIRMWARE_VERSION = 11,
      RAWDATA_OF_3AXIS_GYRO = 13,
      GENERAL_PURPOSE_INPUT = 16,
      UDID = 19,
    };


  public:
    Translator(KobukiBase* pKobuki);
    virtual ~Translator();

  public:
    std::vector<Packet> split(const Packet& packet);
    void translate(const Packet& packet);

    void onBasicSensorData(const Packet& packet);
    void onDockingIR(const Packet& packet);
    void onInertialSensor(const Packet& packet);
    void onCliff(const Packet& packet);
    void onCurrent(const Packet& packet);
    void onRawDataOf3AxisGyro(const Packet& packet);
    void onGeneralPurposeInput(const Packet& packet);


  };
}
