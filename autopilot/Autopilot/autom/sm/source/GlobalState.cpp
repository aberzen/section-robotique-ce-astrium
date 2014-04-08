/*
 * GlobalState.cpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <autom/sm/include/GlobalState.hpp>
#include <system/system/include/System.hpp>

namespace autom {

GlobalState::GlobalState()
: FrameScheduling(&_frames[0]),
  _state(E_STATE_OFF)
{
	_frame0Processes[0] = &system::System::system.board.imu;
	_frame0Processes[1] = &system::System::system.board.compass;
	_frame0Processes[2] = &system::System::system.board.baro;
	_frame0Processes[3] = &system::System::system.board.gps;
	_frame0Processes[4] = &system::System::system.ancs.est;
	_frame0Processes[5] = &system::System::system.board.pwm;
	_frame0Processes[6] = &system::System::system.ancs.smGroundContact;
	_frame0Processes[7] = &system::System::system.ancs.smFlyingState;
	_frame0Processes[8] = &system::System::system.ancs.modeMgt;

	_frame1Processes[0] = &system::System::system.board.imu;
	_frame1Processes[1] = &system::System::system.board.compass;
	_frame1Processes[2] = &system::System::system.board.baro;
	_frame1Processes[3] = &system::System::system.board.gps;
	_frame1Processes[4] = &system::System::system.ancs.est;
	_frame1Processes[5] = &system::System::system.ancs.modulator;
	_frame1Processes[6] = &system::System::system.ancs.gimbal;
	_frame1Processes[7] = &system::System::system.board.pwm;

	/* Set frame 1 */
	_frames[0].nb = SCHED_FRAME0_NB_PROC;
	_frames[0].processes = &_frame0Processes[0];

	/* Set frame 1 */
	_frames[1].nb = SCHED_FRAME1_NB_PROC;
	_frames[1].processes = &_frame1Processes[0];

	/* Set execution order */
	_frames[0].next = &_frames[1];
	_frames[1].next = &_frames[0];
}

GlobalState::~GlobalState()
{
}

/** @brief Init the process */
void GlobalState::initialize()
{
	/* Super */
	FrameScheduling::initialize();

	system::System::system.ancs.est.initialize();
	system::System::system.ancs.modulator.initialize();
	system::System::system.ancs.gimbal.initialize();
	system::System::system.ancs.smGroundContact.initialize();
	system::System::system.ancs.smFlyingState.initialize();
	system::System::system.ancs.procImuCalib.reset();
	system::System::system.ancs.procCompDec.stop();
	system::System::system.ancs.modeMgt.initialize();
	system::System::system.ancs.attCtrl.initialize();

	/* Start in off */
	_state = E_STATE_INITIALIZING_IMU;
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
	case E_STATE_FAILSAFE:
		processFailsafe();
		break;
	}
}


/** @brief Process the off state */
void GlobalState::processOff()
{
	/* Nothing to do */
}

/** @brief Process the off state */
void GlobalState::processInitImu()
{
	system::System::system.board.imu.execute();
	system::System::system.ancs.procImuCalib.onTick();

	if (system::System::system.ancs.procImuCalib.getState() == ProcCalibGyroBias::E_PROCCALIBIMU_ENDED)
	{
		system::System::system.ancs.procCompDec.reset();
		_state = E_STATE_INITIALIZING_COMPASS;
		Serial.print("INITIALIZING_COMPASS\n");
	}
	else if (system::System::system.ancs.procImuCalib.getState() == ProcCalibGyroBias::E_PROCCALIBIMU_FAILED)
	{
		system::System::system.ancs.procImuCalib.reset();
	}
}

/** @brief Process the off state */
void GlobalState::processInitCompass()
{
	system::System::system.board.imu.execute();
	system::System::system.board.compass.execute();
	system::System::system.ancs.procCompDec.onTick();

	if (system::System::system.ancs.procCompDec.getState() == ProcCompassDeclination::E_STATE_ENDED)
	{
		/* Initialize additional functions */
		system::System::system.ancs.smGroundContact.start();
		system::System::system.ancs.smFlyingState.start();

		_state = E_STATE_READY;
		Serial.print("READY\n");
	}
}

/** @brief Process the off state */
void GlobalState::processReady()
{
	/* Call to super / execute the scheduling frame */
	FrameScheduling::execute();
}

/** @brief Process the failsafe state */
void GlobalState::processFailsafe()
{
	/* Nothing to do */
}


/** @brief Command failsafe */
void GlobalState::cmdFailsafe()
{
	/* Disarm */
	system::System::system.ancs.smFlyingState.disarm();
	_state = E_STATE_FAILSAFE;
}


} /* namespace autom */
