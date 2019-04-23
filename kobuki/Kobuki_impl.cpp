
#define _USE_MATH_DEFINES

#include <cmath>

#include "Kobuki_impl.h"
#include "DockingController.h"

using namespace rt_net;

const double Kobuki_impl::m_LengthOfShaft = 0.23; // [m]
const double Kobuki_impl::m_PulseToRadian = 0.002436916871363930187454;
const double Kobuki_impl::m_WheelRadius = 0.035;

Kobuki_impl::Kobuki_impl(const KobukiStringArgument& arg) : 
  KobukiBase(arg.toString().c_str()),
  m_Initialized(false),
  m_DockingMode(IDLE)
{
  m_pDockingController = new DockingController(this);
}

Kobuki_impl::~Kobuki_impl()
{
  delete m_pDockingController;
}

void Kobuki_impl::setTargetVelocity(const double& vx_m_per_sec, 
				    const double& va_rad_per_sec)
{
  const double epsilon = 0.0001;

  int16_t radius = 0;
  int16_t speed = 0;
  if (std::fabs(va_rad_per_sec) < epsilon) {
    radius = 0;
  } else if (std::fabs(vx_m_per_sec) < epsilon && 
	     va_rad_per_sec > epsilon) {
    radius = 1;
  } else if (std::fabs(vx_m_per_sec) < epsilon &&
	     va_rad_per_sec < -epsilon) {
    radius = -1;
  } else {
    radius = (int16_t)(vx_m_per_sec * 1000.0 / va_rad_per_sec);
  }

  if (vx_m_per_sec < 0.0) {
    speed = (int16_t)(1000.0 * 
		      std::min<double>(vx_m_per_sec + m_LengthOfShaft 
			       * va_rad_per_sec / 2.0,
			       vx_m_per_sec - m_LengthOfShaft 
			       * va_rad_per_sec / 2.0));
  } else {
    speed = (int16_t)(1000.0 * 
		      std::max<double>(vx_m_per_sec + m_LengthOfShaft 
			       * va_rad_per_sec / 2.0,
			       vx_m_per_sec - m_LengthOfShaft 
			       * va_rad_per_sec / 2.0));
  }
  
  baseControl(speed, radius);
}

void Kobuki_impl::onUpdate()
{
  processOdometry();
}

void Kobuki_impl::onPreSendCommand()
{
  if(m_DockingMode == DOCKING) {
	  m_pDockingController->process();
	  if(isDocked()) {
		  m_DockingMode = DOCKED;
	  } else if(m_pDockingController->isBumpStop()) {
		  m_DockingMode = BUMPSTOP;
	  } else if(m_pDockingController->isCliffStop()) {
		  m_DockingMode = CLIFFSTOP;
	  } else if(m_pDockingController->isDropStop()) {
		  m_DockingMode = DROPSTOP;
	  } else if(m_pDockingController->isLostStop()) {
		  m_DockingMode = LOSTSTOP;
	  }
  }
}

void Kobuki_impl::processOdometry() {
  if(!m_Initialized) {
    m_X = m_Y = m_Th = 0;
    m_OldRightEncoder = m_RightEncoder;
    m_OldLeftEncoder  = m_LeftEncoder;
    m_Initialized = true;
  }

  int16_t deltaRightEncoder = (int16_t)((m_RightEncoder - m_OldRightEncoder) & 0xFFFF);
  int16_t deltaLeftEncoder  = (int16_t)((m_LeftEncoder  - m_OldLeftEncoder) & 0xFFFF);

  double forward = (double)(deltaRightEncoder + deltaLeftEncoder) 
    * m_WheelRadius * m_PulseToRadian / 2.0;
  double rotate  = (double)(deltaRightEncoder - deltaLeftEncoder) 
    * m_WheelRadius * m_PulseToRadian / m_LengthOfShaft;
  
  m_X += forward * cos( m_Th + rotate / 2.0);
  m_Y += forward * sin( m_Th + rotate / 2.0);
  m_Th += rotate;
  if (m_Th < -M_PI) {
    m_Th += 2 * M_PI;
  } else if (m_Th > M_PI) {
    m_Th -= 2 * M_PI;
  }

  m_OldRightEncoder = m_RightEncoder;
  m_OldLeftEncoder  = m_LeftEncoder;
}

DOCKSTATE Kobuki_impl::dock(const bool block)
{
	if(m_DockingMode != DOCKING) {
		m_pDockingController->start();
		m_DockingMode = DOCKING;
	}
	while(block) {
		if(!isDocked()) {
			return DOCKED;
		}  else if(m_pDockingController->isBumpStop()) {
			return BUMPSTOP;
		} else if(m_pDockingController->isCliffStop()) {
			return CLIFFSTOP;
		} else if(m_pDockingController->isDropStop()) {
			return DROPSTOP;
		} else if(m_pDockingController->isLostStop()) {
			return LOSTSTOP;
		}
		Thread::Sleep(5);
	}
	return DOCKING;
}
