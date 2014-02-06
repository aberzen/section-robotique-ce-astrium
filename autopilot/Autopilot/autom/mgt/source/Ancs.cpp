/*
 * Ancs.cpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#include <autom/mgt/include/Ancs.hpp>
#include <system/system/include/System.hpp>

#include <infra/include/Task.hpp>

namespace autom {

Ancs::Ancs(
		const float& dt_HF,
		const float& dt_LF,
		const Ancs::Param& param) :
_param(param),
_torque_B(0.,0.,0.),
_force_B(0.,0.,0.),
torqueReal_B(0.,0.,0.),
forceReal_B(0.,0.,0.),
procImuCalib(
		::system::System::system.board.meas.imu,
		estimations,
		_param.procCalibImu
		),
procCompDec(
		::system::System::system.board.meas,
		estimations,
		_param.est.declination,
		_param.procCompDec
		),
smGroundContact(
		_param.smGroundContact,
		_param.gen
		),
_est(
		::system::System::system.board.meas,
		estimations,
		dt_HF,
		_param.est
 	 	),
_attCtrl(
		_attGuid,
		estimations,
		_torque_B,
		dt_HF,
		_param.attCtrl
		),
modulator(
		 _torque_B,
		 _force_B,
		 system::System::system.board.pwmVal,
		 torqueReal_B,
		 forceReal_B,
		 _param.modGen,
		 _param.modPinv,
		 system::System::system.board.pwm
		 ),
 _modeStabilitized(
		estimations,
		_attGuid,
		_force_B,
		dt_LF,
		_param.modeStabilized,
		_param.gen
		 ),
_gimbal(
		estimations,
		param.gimbal
		)
{
	uint8_t iChannel;
	for (iChannel=0 ; iChannel<PWM_OUT_NUM_CHANNELS ; iChannel++)
	{
		radioChannels[iChannel] = new RadioChannel(
				system::System::system.board.radio.channels[iChannel],
				_param.radioChannel[iChannel]);
	}
}

Ancs::~Ancs() {
}

/** @brief Init the process */
void Ancs::initialize()
{
	smGroundContact.reset();
	modulator.initialize();
	_gimbal.initialize();
	execTransToInitCalibImu();
}

/** @brief Execute the process */
void Ancs::execute()
{
	/* Assert transition to flying mode */
	switch (_state)
	{
	case E_STATE_INITIALIZATION_CALIB_IMU:
		stepInitCalibImu();
		evalTransFromInitCalibImu();
		break;
	case E_STATE_INITIALIZATION_CALIB_COMPASS:
		stepInitCalibCompass();
		evalTransFromInitCalibCompass();
		break;
	case E_STATE_READY:
		stepReady();
		evalTransFromReady();
		break;
	case E_STATE_FLYING:
		stepFlying();
		evalTransFromFlying();
		break;
	case E_STATE_UNDEFINED:
	case E_STATE_FAILSAFE:
	default: /* Unexpected */

		/* Execute failsafe */
		stepFailsafe();
		evalTransFromFailSafe();
		break;
	}
}


/** @brief Execute the transition to E_STATE_INITIALIZATION_CALIB_IMU */
void Ancs::execTransToInitCalibImu()
{
 	/* Initialize the procedure */
	procImuCalib.reset();

	/* Change state to calib Imu */
	_state = E_STATE_INITIALIZATION_CALIB_IMU;

	Serial.printf("E_STATE_INITIALIZATION_CALIB_IMU\n");
}



/** @brief Step initialization */
void Ancs::stepInitCalibImu()
{
	switch (procImuCalib.getState())
	{
	case ProcCalibGyroBias::E_PROCCALIBIMU_OFF:
		/* Start calibration procedure */
		procImuCalib.start();
		break;

	case ProcCalibGyroBias::E_PROCCALIBIMU_ENDED:
		/* Calibration is over (and successfull), switch to ready state */
		break;

	case ProcCalibGyroBias::E_PROCCALIBIMU_FAILED:
		/* Someone move the sensor during calibration procedure */
		procImuCalib.reset();
		break;
	case ProcCalibGyroBias::E_PROCCALIBIMU_INIT:
	case ProcCalibGyroBias::E_PROCCALIBIMU_COMP_BIAS:
	case ProcCalibGyroBias::E_PROCCALIBIMU_COMP_VAR:
	default:
		/* Process board */
		system::System::system.board.execute();
		/* Execute calibration procedure */
		procImuCalib.onTick();
		break;
	}
}

/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_IMU */
void Ancs::evalTransFromInitCalibImu()
{
	if (ProcCalibGyroBias::E_PROCCALIBIMU_ENDED == procImuCalib.getState())
	{
		/* Procedure is over, command transition to compass calibration */
		execTransToInitCalibCompass();
	}
}

/** @brief Execute transitions to E_STATE_INITIALIZATION_CALIB_COMPASS */
void Ancs::execTransToInitCalibCompass()
{
	/* Initialize the procedure */
	procCompDec.reset();

	/* Set new state to compass calibration */
	_state = E_STATE_INITIALIZATION_CALIB_COMPASS;

	Serial.printf("E_STATE_INITIALIZATION_CALIB_COMPASS\n");
}

/** @brief Step Compass calibration state */
void Ancs::stepInitCalibCompass()
{
	switch (procCompDec.getState())
	{
	case ProcCompassDeclination::E_STATE_OFF:
		/* Start calibration procedure */
		procCompDec.start();
		break;

	case ProcCompassDeclination::E_STATE_INIT:
	case ProcCompassDeclination::E_STATE_RUNNING:
		/* Process board */
		system::System::system.board.execute();
		/* Execute calibration procedure */
		procCompDec.onTick();
		break;

	case ProcCompassDeclination::E_STATE_ENDED:
		/* Calibration is over (and successfull), switch to ready state */
		break;

	default:
		/* or unexpected */
		procCompDec.reset();
		break;
	}
}

/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_COMPASS */
void Ancs::evalTransFromInitCalibCompass()
{
	if (ProcCompassDeclination::E_STATE_ENDED == procCompDec.getState())
	{
		/* Procedure is over, command transition to ready */
		execTransToReady();
	}
}

/** @brief Execute the transition to E_STATE_READY */
void Ancs::execTransToReady()
{
	uint8_t idxMotor;

	/* Set demanded force and torque to null*/
	_force_B(0., 0., 0.);
	_torque_B(0., 0., 0.);
	forceReal_B(0., 0., 0.);
	torqueReal_B(0., 0., 0.);

	/* Process PWM */
	for (idxMotor=0 ; idxMotor<CONFIG_NB_MOTOR ; idxMotor++)
	{
		system::System::system.board.pwmVal.channels[idxMotor] = MIN_PULSEWIDTH;
		system::System::system.board.pwm.force_out(idxMotor);
		system::System::system.board.pwm.disable_out(idxMotor);
	}

	/* Initialization of the estimator */
	_est.initialize();

	/* Set new state to compass calibration */
	_state = E_STATE_READY;

	Serial.printf("E_STATE_READY\n");
}

/** @brief Step ready */
void Ancs::stepReady()
{
//	/* Process sensors */
//	system::System::system.board.getImu().execute();
//	system::System::system.board.getBaro().execute();
//	system::System::system.board.getCompass().execute();
//	system::System::system.board.getGps().execute();
//
//	/* Process PWM */
//	system::System::system.board.pwm.execute();
	system::System::system.board.execute();

	/* Compute estimation */
	_est.execute();

	/* Compute camera pan tilt mount orientation */
	_gimbal.execute();

	/* Compute ground detection procedure */
//	smGroundContact.onTick();
}

/** @brief Assert transitions from E_STATE_READY */
void Ancs::evalTransFromReady()
{
}

/** @brief Execute the transition to E_STATE_READY */
void Ancs::execTransToFlying()
{
	/* Set demanded force and torque to null*/
	_force_B(0., 0., 0.);
	_torque_B(0., 0., 0.);
	forceReal_B(0., 0., 0.);
	torqueReal_B(0., 0., 0.);

	/* Arm motors */
	modulator.arm();

	/* Set new state to compass calibration */
	_state = E_STATE_FLYING;

//	Serial.printf("E_STATE_FLYING\n");
}

/** @brief Step flying */
void Ancs::stepFlying()
{
	/* Process board */
	system::System::system.board.execute();

	/* Compute estimation */
	_est.execute();

	/* Process stabilized mode */
	_modeStabilitized.execute();

	/* Process PWM */
	modulator.execute();

//	Serial.printf("qDem_IB = [%.5f %.5f %.5f %.5f]\n", this->.qDem_IB.scalar, _demAttData.qDem_IB.vector.x, _demAttData.qDem_IB.vector.y, _demAttData.qDem_IB.vector.z);
//	Serial.printf("rateDem_B = [%.5f %.5f %.5f]\n", _demAttData.angRateDem_B.x, _demAttData.angRateDem_B.y, _demAttData.angRateDem_B.z);
//	Serial.printf("torque_B = [%.5f %.5f %.5f]\n", _torque_B.x, _torque_B.y, _torque_B.z);
//	Serial.printf("force_B = [%.5f %.5f %.5f]\n", _force_B.x, _force_B.y, _force_B.z);
//	Serial.printf("%d %d %d %d %d\n", system::System::system.board.radio.isAvailable, system::System::system.board.radio.channels[0], system::System::system.board.radio.channels[1], system::System::system.board.radio.channels[2], system::System::system.board.radio.channels[3]);
//	Serial.printf("%.5f;%.5f;%.5f;%.5f;%.5f;%.5f\n", _torque_B.x, _torque_B.y, _torque_B.z, _force_B.x, _force_B.y, _force_B.z);
//	Serial.printf("%d %d %d %d\n", system::System::system.board.pwmVal.channels[0], system::System::system.board.pwmVal.channels[1], system::System::system.board.pwmVal.channels[2], system::System::system.board.pwmVal.channels[3]);
}

/** @brief Assert transitions from E_STATE_FLYING */
void Ancs::evalTransFromFlying()
{

}

/** @brief Execute the transition to E_STATE_FAILSAFE */
void Ancs::execTransToFailsafe()
{
	/* Set demanded force and torque to null*/
	_force_B(0., 0., 0.);
	_torque_B(0., 0., 0.);
	forceReal_B(0., 0., 0.);
	torqueReal_B(0., 0., 0.);

	/* Disarm motors */
	modulator.disarm();

	_state = E_STATE_FAILSAFE;

//	Serial.printf("E_STATE_FAILSAFE\n");

}

/** @brief Step fail safe */
void Ancs::stepFailsafe()
{
	/* Do nothing */
}

/** @brief Assert transitions from E_STATE_FAILSAFE */
void Ancs::evalTransFromFailSafe()
{
	/* Do nothing */
}

} /* namespace autom */
