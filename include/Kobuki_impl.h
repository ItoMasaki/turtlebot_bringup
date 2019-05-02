#pragma once 


#include "Kobuki.h"
#include "KobukiBase.h"

//#include "DockingController.h"

#include <string>

namespace rt_net {

  //  class Kobuki_impl;
  class DockingController;


  /**
   * @brief Kobukiインターフェースの実装クラス
   */
  class LIBKOBUKI_API Kobuki_impl : public KobukiBase, public Kobuki {
  private:
    bool m_Initialized;
    const static double m_LengthOfShaft;
    const static double m_PulseToRadian;
    const static double m_WheelRadius;
    uint16_t m_OldRightEncoder, m_OldLeftEncoder;
    double m_X, m_Y, m_Th;

  private:
    DOCKSTATE m_DockingMode;
    DockingController *m_pDockingController;
    friend class DockingController;
  public:
    Kobuki_impl(const KobukiStringArgument& argument);
    virtual ~Kobuki_impl();

    virtual void onUpdate();
    virtual void onPreSendCommand();

    void setTargetVelocity(const double& vx, const double& va);

    void setDigitalOut(const GPIO channel, const bool flag) {setGPIO(channel, flag);}

    void setExternalPower(const POWER channel, const bool flag) {setPower(channel, flag);}

    void setLED1(const COLOR color, const bool flag) {setLED(LED1, color, flag);}
    void setLED2(const COLOR color, const bool flag) {setLED(LED2, color, flag);}

    bool isButton0() {return m_Button & 0x01 ? true: false;}
    bool isButton1() {return m_Button & 0x02 ? true: false;}
    bool isButton2() {return m_Button & 0x04 ? true: false;}
    bool isRightBump() {return m_Bumper & 0x01 ? true : false;}
    bool isCenterBump(){return m_Bumper & 0x02 ? true : false;}
    bool isLeftBump()  {return m_Bumper & 0x04 ? true : false;}
    bool isRightWheelDrop() {return m_WheelDrop & 0x01 ? true : false;}
    bool isLeftWheelDrop()  {return m_WheelDrop & 0x02 ? true : false;}
    bool isRightCliff() {return m_Cliff & 0x01 ? true : false;}
    bool isCenterCliff(){return m_Cliff & 0x02 ? true : false;}
    bool isLeftCliff()  {return m_Cliff & 0x04 ? true : false;}

    bool isRightIRFarRight()  {return m_DockRightSignal&0x20 ? true:false;}
    bool isRightIRFarCenter() {return m_DockRightSignal&0x08 ? true:false;}
    bool isRightIRFarLeft()   {return m_DockRightSignal&0x10 ? true:false;}
    bool isRightIRNearRight() {return m_DockRightSignal&0x04 ? true:false;}
    bool isRightIRNearCenter(){return m_DockRightSignal&0x02 ? true:false;}
    bool isRightIRNearLeft()  {return m_DockRightSignal&0x01 ? true:false;}

    bool isCenterIRFarRight()  {return m_DockCenterSignal&0x20? true:false;}
    bool isCenterIRFarCenter() {return m_DockCenterSignal&0x08? true:false;}
    bool isCenterIRFarLeft()   {return m_DockCenterSignal&0x10? true:false;}
    bool isCenterIRNearRight() {return m_DockCenterSignal&0x04? true:false;}
    bool isCenterIRNearCenter(){return m_DockCenterSignal&0x02? true:false;}
    bool isCenterIRNearLeft()  {return m_DockCenterSignal&0x01? true:false;}

    bool isLeftIRFarRight()  {return m_DockLeftSignal&0x20? true:false;}
    bool isLeftIRFarCenter() {return m_DockLeftSignal&0x08? true:false;}
    bool isLeftIRFarLeft()   {return m_DockLeftSignal&0x10? true:false;}
    bool isLeftIRNearRight() {return m_DockLeftSignal&0x04? true:false;}
    bool isLeftIRNearCenter(){return m_DockLeftSignal&0x02? true:false;}
    bool isLeftIRNearLeft()  {return m_DockLeftSignal&0x01? true:false;}

    bool isCharging() {return m_Charger == 6 || m_Charger == 22;}
    bool isCharged() {return m_Charger == 2 || m_Charger == 18;}
    bool isDischarging() {return m_Charger == 0;}
    bool isDocked() {return !isDischarging();}
    
    double getBatteryVoltage() {return m_Battery * 0.1;}

    double getPoseX() {return m_X;}
    double getPoseY() {return m_Y;}
    double getPoseTh() {return m_Th;}
    uint16_t getRightMotorEncoder() {return m_RightEncoder;}
    uint16_t getLeftMotorEncoder() {return m_LeftEncoder;}

	double getInertialAngle() {return m_InertialAngle;}
	double getInertialAngleRate() {return m_InertialAngleRate;}

    void getPose(double* x, double* y, double* th) {*x=m_X, *y=m_Y, *th = m_Th;}
    void setPose(double x, double y, double th) {m_X = x, m_Y = y, m_Th = th;}

    double getRightMotorCurrent() {return m_RightMotorCurrent;}
    double getLeftMotorCurrent() {return m_LeftMotorCurrent;}

    bool getDigitalIn(const uint8_t ch) {return m_GPIN & (0x01 << ch) ? true: false;}
    double getAnalogIn(const uint8_t ch) {return m_ADIN[ch];}

    DOCKSTATE dock(const bool block = true);
	DOCKSTATE getDockState() {return m_DockingMode;}
  private:
    void processOdometry();
  };
}
