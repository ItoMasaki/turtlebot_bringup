#pragma once

#include <stdint.h>
#include "Kobuki_impl.h"

namespace rt_net {

  class Kobuki_impl;

  /**
   * 
   */
  class DockingController {
    enum MODE{
      MODE_IDLE,
      MODE_SCAN,
      MODE_MOVE_TO_CENTERLINE,
      MODE_TURN_ON_CENTERLINE,
      MODE_APPROACH,
      MODE_BUMPSTOP,
      MODE_LOSTSTOP,
      MODE_DROPSTOP,
      MODE_CLIFFSTOP,
    };

  private:
    int32_t m_BumpCount;
    int32_t m_LostCount;
    MODE m_Mode;
	double m_RotationStartAngle;
    int32_t m_ApproachSide;
    
    Kobuki_impl *m_pKobuki;

#define BUMP_MODE_THRESHOLD 10
#define LOST_THRESHOLD 100

    enum SIDE{
      SIDE_CENTER,
      SIDE_RIGHT,
      SIDE_LEFT
    };

  public:
    DockingController(Kobuki_impl* pKobuki);
    virtual ~DockingController();
  public:

    bool start();
    void process();
    
    bool isDocking() {
      if(m_Mode == MODE_IDLE || m_Mode == MODE_BUMPSTOP || m_Mode == MODE_LOSTSTOP || m_Mode == MODE_CLIFFSTOP || m_Mode == MODE_DROPSTOP) {
	return false;
      } 
      return true;
    }

    bool isBumpStop() {
      return m_Mode == MODE_BUMPSTOP;
    }

    bool isDropStop() {
      return m_Mode == MODE_DROPSTOP;
    }

    bool isCliffStop() {
      return m_Mode == MODE_CLIFFSTOP;
    }
    
    bool isLostStop() {
	  return m_Mode == MODE_LOSTSTOP;
	}

    void stop() {
      m_Mode = MODE_IDLE;
      m_pKobuki->setTargetVelocity(0, 0);
    }
    
  private:
    void approachingBehavior();
    
    void onBump();
    void onModeScan();
    void onModeMoveToCenterLine();
    void onModeTurnOnCenterLine();
    void onModeApproach();
  };
}
