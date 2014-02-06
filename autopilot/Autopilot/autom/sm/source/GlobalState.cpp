/*
 * GlobalState.cpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#include <autom/sm/include/GlobalState.hpp>
#include <system/system/include/System.hpp>

namespace autom {

GlobalState::GlobalState()
: Process(),
  _state(E_STATE_OFF)
{
}

GlobalState::~GlobalState()
{
}

/** @brief Init the process */
void GlobalState::initialize()
{
	/* Start in off */
	_state = E_STATE_OFF;
}

/** @brief Execute the process */
void GlobalState::execute()
{
	switch (_state)
	{
	case E_STATE_OFF:
		processOff();
		break;
	case E_STATE_INITIALIZING_IMU:
		processInitImu();
		break;
	case E_STATE_INITIALIZING_COMPASS:
		processInitCompass();
		break;
	case E_STATE_READY:
		processReady();
		break;
	}
}


/** @brief Process the off state */
void GlobalState::processOff()
{
	system::System::system.ancs.procImuCalib.reset();
}

/** @brief Process the off state */
void GlobalState::processInitImu()
{
	system::System::system.ancs.procImuCalib.onTick();

	if (system::System::system.ancs.procImuCalib.getState() == ProcCalibGyroBias::E_PROCCALIBIMU_ENDED)
	{
		system::System::system.ancs.procCompDec.reset();
		_state = E_STATE_INITIALIZING_COMPASS;
	}
	else if (system::System::system.ancs.procImuCalib.getState() == ProcCalibGyroBias::E_PROCCALIBIMU_FAILED)
	{
		system::System::system.ancs.procImuCalib.reset();
	}
}

/** @brief Process the off state */
void GlobalState::processInitCompass()
{
	system::System::system.ancs.procCompDec.onTick();

	if (system::System::system.ancs.procCompDec.getState() == ProcCompassDeclination::E_STATE_ENDED)
	{
		_state = E_STATE_READY;
	}
}

/** @brief Process the off state */
void GlobalState::processReady()
{
	/* Nothing to do */
}


} /* namespace autom */
