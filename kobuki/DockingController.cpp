#include "DockingController.h"


using namespace rt_net;


DockingController::DockingController(Kobuki_impl *pKobuki) :
m_pKobuki(pKobuki), m_Mode(MODE_IDLE), m_ApproachSide(0), m_BumpCount(0), m_LostCount(0), m_RotationStartAngle(0)
{
}

DockingController::~DockingController()
{
}



bool DockingController::start() {
	if((m_Mode != MODE_IDLE) && (m_Mode != MODE_BUMPSTOP) && (m_Mode != MODE_LOSTSTOP) 
		&& (m_Mode != MODE_DROPSTOP) && (m_Mode != MODE_CLIFFSTOP)) {
		return false;
	}
	m_ApproachSide = SIDE_CENTER;
	m_BumpCount = m_LostCount = 0;
	m_Mode = MODE_SCAN;
	m_RotationStartAngle = m_pKobuki->getPoseTh();
	return true;
}

void DockingController::process() {
	approachingBehavior();
}

void DockingController::approachingBehavior()
{
	if(m_pKobuki->isCharging() || m_pKobuki->isCharged()) {
		m_pKobuki->setTargetVelocity(0, 0);
		m_Mode = MODE_IDLE;
		return;
	}

	if (m_pKobuki->isRightWheelDrop() || m_pKobuki->isLeftWheelDrop()) {
		m_pKobuki->setTargetVelocity(0, 0);
		m_Mode = MODE_DROPSTOP;
		return;
	}

	if (m_pKobuki->isRightCliff() || m_pKobuki->isCenterCliff() || m_pKobuki->isLeftCliff()) {
		m_pKobuki->setTargetVelocity(0, 0);
		m_Mode = MODE_CLIFFSTOP;
		return;
	}

	if(m_pKobuki->isRightBump() || m_pKobuki->isCenterBump() || m_pKobuki->isLeftBump()) {
		onBump();
		return;
	} 

	if(m_Mode == MODE_SCAN) {
		onModeScan();
	} else if(m_Mode == MODE_MOVE_TO_CENTERLINE) {
		onModeMoveToCenterLine();
	} else if (m_Mode == MODE_TURN_ON_CENTERLINE) {
		onModeTurnOnCenterLine();
	} else if (m_Mode == MODE_APPROACH) {
		onModeApproach();
	}
}


void DockingController::onBump() {
	if(m_Mode == MODE_APPROACH) { // Final Behavior
		if(m_pKobuki->isCenterIRNearLeft() || m_pKobuki->isCenterIRFarLeft()) {
			m_pKobuki->setTargetVelocity(0, +0.2);
		} else if(m_pKobuki->isCenterIRNearRight() || m_pKobuki->isCenterIRFarRight()) {
			m_pKobuki->setTargetVelocity(0, -0.2);
		} else if(m_pKobuki->isRightIRNearLeft() || m_pKobuki->isRightIRFarLeft()) {
			m_pKobuki->setTargetVelocity(0, -0.2);
		} else if(m_pKobuki->isLeftIRNearRight() || m_pKobuki->isLeftIRFarRight()) {
			m_pKobuki->setTargetVelocity(0, +0.2);
		} 
		// Unknown State
		else {
			m_BumpCount++;
		}

	} else { // Just Bumped
		m_BumpCount++;
	}

	// Bump Stop
	if (m_BumpCount > BUMP_MODE_THRESHOLD) {
		m_Mode = MODE_BUMPSTOP;
		m_pKobuki->setTargetVelocity(0,0);
	}
}

void DockingController::onModeScan() {
	// Kobuki is toward the dock
	if (m_pKobuki->isCenterIRNearRight()) { 
		// In Right Area
		m_pKobuki->setTargetVelocity(-0.01, -0.33);
	} else if (m_pKobuki->isCenterIRFarRight()) {
		// In Right Area
		m_pKobuki->setTargetVelocity(0.0, -0.33);
	} else if (m_pKobuki->isCenterIRNearLeft()) {
		// In Left Area
		m_pKobuki->setTargetVelocity(-0.01, 0.33);
	} else if (m_pKobuki->isCenterIRFarLeft()) {
		// In Left Area
		m_pKobuki->setTargetVelocity(0.0, 0.33);
	} else if (m_pKobuki->isCenterIRFarCenter() || m_pKobuki->isCenterIRNearCenter()) {
		// Kobuki is in Center Area and faces toward Dock.
		m_pKobuki->setTargetVelocity(0.03, 0);
		m_ApproachSide = SIDE_CENTER;
		m_Mode = MODE_APPROACH;
		m_LostCount = 0;
	}

	// Kobuki is in Left Area but faces to right
	else if ( m_pKobuki->isLeftIRNearLeft()) {
		m_pKobuki->setTargetVelocity(-0.01, 0.55);
	} else if (m_pKobuki->isLeftIRFarLeft()) {
		m_pKobuki->setTargetVelocity(0, 0.66);
	} 
	// Kobuki is in Right Area but faces to left
	else if (m_pKobuki->isRightIRNearRight()) {
		m_pKobuki->setTargetVelocity(-0.01, -0.55);
	} else if (m_pKobuki->isRightIRFarRight()) {
		m_pKobuki->setTargetVelocity(0, -0.66);
	}

	// Kobuki is in Right Area and faces to Left
	else if (m_pKobuki->isLeftIRNearRight()) {
		m_pKobuki->setTargetVelocity(0.03, -0.01);    
		m_ApproachSide = SIDE_RIGHT;
		m_Mode = MODE_MOVE_TO_CENTERLINE;
		m_LostCount = 0;
	}
	else if (m_pKobuki->isLeftIRFarRight() ) {
		m_pKobuki->setTargetVelocity(0.03, 0.01);    
		m_ApproachSide = SIDE_RIGHT;
		m_Mode = MODE_MOVE_TO_CENTERLINE;
		m_LostCount = 0;
	}
	// Kobuki is in Left Area and faces to Right

	else if (m_pKobuki->isRightIRNearLeft()) {
		m_pKobuki->setTargetVelocity(0.02, +0.01);
		m_ApproachSide = SIDE_LEFT;
		m_Mode = MODE_MOVE_TO_CENTERLINE;
		m_LostCount = 0;
	}
	else if (m_pKobuki->isRightIRFarLeft()) {
		m_pKobuki->setTargetVelocity(0.03, -0.01);
		m_ApproachSide = SIDE_LEFT;
		m_Mode = MODE_MOVE_TO_CENTERLINE;
		m_LostCount = 0;
	}

	else {
		m_LostCount++;
		if(m_LostCount > LOST_THRESHOLD) {
			m_Mode = MODE_LOSTSTOP;
			m_pKobuki->setTargetVelocity(0, 0);
		}
	}

}


void DockingController::onModeMoveToCenterLine()
{
	if (m_ApproachSide == SIDE_LEFT) {
		if (m_pKobuki->isRightIRNearLeft()) {
			m_pKobuki->setTargetVelocity(0.02, +0.005);
		} else if (m_pKobuki->isRightIRFarLeft()) {
			m_pKobuki->setTargetVelocity(0.05, -0.01);
		} else if (m_pKobuki->isRightIRFarRight() || m_pKobuki->isRightIRNearRight() 
			|| m_pKobuki->isRightIRFarCenter() || m_pKobuki->isRightIRNearCenter()) {
				m_pKobuki->setTargetVelocity(0.01, -0.33);
				m_Mode =MODE_TURN_ON_CENTERLINE;
			m_LostCount = 0;
		} else {
			m_LostCount++;
			if(m_LostCount > LOST_THRESHOLD) {
				m_Mode = MODE_LOSTSTOP;
				m_pKobuki->setTargetVelocity(0, 0);
			}
		}
	} else if (m_ApproachSide == SIDE_RIGHT) {
		if (m_pKobuki->isLeftIRNearRight()) {
			m_pKobuki->setTargetVelocity(0.02, -0.005);    
		} else if (m_pKobuki->isLeftIRFarRight()) {
			m_pKobuki->setTargetVelocity(0.05, 0.01);
		} else if (m_pKobuki->isLeftIRFarLeft() || m_pKobuki->isLeftIRNearLeft() 
			|| m_pKobuki->isLeftIRFarCenter() || m_pKobuki->isLeftIRNearCenter()) {
				m_pKobuki->setTargetVelocity(0.01, +0.33);
				m_Mode = MODE_TURN_ON_CENTERLINE;
				m_LostCount = 0;
		} else {
			m_LostCount++;
			if(m_LostCount > LOST_THRESHOLD) {
				m_Mode = MODE_LOSTSTOP;
				m_pKobuki->setTargetVelocity(0, 0);
			}
		}
	} else if (m_pKobuki->isCenterIRFarCenter() || m_pKobuki->isCenterIRNearCenter()) {
		m_pKobuki->setTargetVelocity(0.05,0);
		m_Mode = MODE_APPROACH;
		m_LostCount = 0;

	} else {
		m_LostCount++;
		if(m_LostCount > LOST_THRESHOLD) {
			m_Mode = MODE_LOSTSTOP;
			m_pKobuki->setTargetVelocity(0, 0);
		}
	}
}


void DockingController::onModeTurnOnCenterLine()
{
	if (m_pKobuki->isCenterIRFarCenter() || m_pKobuki->isCenterIRNearCenter()) {
		m_pKobuki->setTargetVelocity(0.05,0);
		m_Mode = MODE_APPROACH;
		m_LostCount = 0;
	} else {
		if(m_ApproachSide == SIDE_LEFT) {
			if (m_pKobuki->isRightIRNearRight() || m_pKobuki->isRightIRFarRight()) {
				m_pKobuki->setTargetVelocity(-0.01, -0.16);
			} else if (m_pKobuki->isRightIRFarCenter() || m_pKobuki->isRightIRNearCenter()) {
				m_pKobuki->setTargetVelocity(0.01, -0.16);
			} else if (m_pKobuki->isRightIRFarLeft() || m_pKobuki->isRightIRNearLeft()) {
				m_pKobuki->setTargetVelocity(0.01, -0.16);
			} else if (m_pKobuki->isCenterIRFarLeft() || m_pKobuki->isCenterIRNearLeft()) {
				m_pKobuki->setTargetVelocity(0.0, 0.08);
			} else if (m_pKobuki->isCenterIRFarRight() || m_pKobuki->isCenterIRNearRight()) {
				m_pKobuki->setTargetVelocity(0.0, -0.08);
			} else {
				m_LostCount++;
				if(m_LostCount > LOST_THRESHOLD) {
					m_Mode = MODE_LOSTSTOP;
					m_pKobuki->setTargetVelocity(0, 0);
				}
			}
		} else if(m_ApproachSide == SIDE_RIGHT) {
			if (m_pKobuki->isLeftIRNearLeft() || m_pKobuki->isLeftIRFarLeft()) {
				m_pKobuki->setTargetVelocity(-0.01, 0.16);
			} else if (m_pKobuki->isLeftIRFarCenter() || m_pKobuki->isLeftIRNearCenter()) {
				m_pKobuki->setTargetVelocity(0.01, 0.16);
			} else if (m_pKobuki->isLeftIRFarRight() || m_pKobuki->isLeftIRNearRight()) {
				m_pKobuki->setTargetVelocity(0.01, 0.16);
			} else if (m_pKobuki->isCenterIRFarRight() || m_pKobuki->isCenterIRNearRight()) {
				m_pKobuki->setTargetVelocity(0.0, -0.08);
			} else if (m_pKobuki->isCenterIRFarLeft() || m_pKobuki->isCenterIRNearLeft()) {
				m_pKobuki->setTargetVelocity(0.0, 0.08);
			} else {
				m_LostCount++;
				if(m_LostCount > LOST_THRESHOLD) {
					m_Mode = MODE_LOSTSTOP;
					m_pKobuki->setTargetVelocity(0, 0);
				}
			}
		}
	}
}


void DockingController::onModeApproach()
{
	if(m_pKobuki->isCenterIRNearLeft()) {
		m_pKobuki->setTargetVelocity(0.01, +0.1);
	} else if(m_pKobuki->isCenterIRNearRight()) {
		m_pKobuki->setTargetVelocity(0.01, -0.1);
	} else if (m_pKobuki->isCenterIRNearCenter()) {
		m_pKobuki->setTargetVelocity(0.02, 0);
	} else if (m_pKobuki->isCenterIRFarCenter()) {
		m_pKobuki->setTargetVelocity(0.04, 0);
	} else if(m_pKobuki->isCenterIRFarRight()) {
		m_pKobuki->setTargetVelocity(0.04, -0.2);
	} else if(m_pKobuki->isCenterIRFarLeft()) {
		m_pKobuki->setTargetVelocity(0.04, +0.2);
	} else {
		m_LostCount++;
		if(m_LostCount > LOST_THRESHOLD) {
			m_Mode = MODE_LOSTSTOP;
			m_pKobuki->setTargetVelocity(0, 0);
		}
	}
}
