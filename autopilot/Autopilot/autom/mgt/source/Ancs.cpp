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

const ControllerPid3Axes::Param dftAttCtrlParam =
{
		{ /* x */
			0., /* Kp */
			0., /* Kd */
			0., /* Ki */
			false, /* useOfRb */
			0., /* Krb */
			0., /* rbThd */
			0., /* rb */
			0. /* maxI */
		}, /* x */
		{ /* y */
			0., /* Kp */
			0., /* Kd */
			0., /* Ki */
			false, /* useOfRb */
			0., /* Krb */
			0., /* rbThd */
			0., /* rb */
			0. /* maxI */
		}, /* y */
		{ /* z */
			0., /* Kp */
			0., /* Kd */
			0., /* Ki */
			false, /* useOfRb */
			0., /* Krb */
			0., /* rbThd */
			0., /* rb */
			0. /* maxI */
		} /* z */
};

Ancs::Ancs(
		const float& dt_HF,
		const float& dt_LF,
		const Ancs::Param& param) :
_param(param),
_torque_B(0.,0.,0.),
_force_B(0.,0.,0.),
_torqueReal_B(0.,0.,0.),
_forceReal_B(0.,0.,0.),
_procImuCalib(
		::system::System::system.board.meas.imu,
		_estVal,
		_param.procCalibImu
		),
_procCompDec(
		::system::System::system.board.meas,
		_estVal,
		_param.est.declination,
		_param.procCompDec
		),
_procDetectGround(
		_estVal,
		system::System::system.board.meas.imu,
		_forceReal_B,
		_groundDetectOutput,
		_param.procGrdDetect,
		_param.gen
		),
_est(
		::system::System::system.board.meas,
		_estVal,
		dt_HF,
		_param.est
 	 	),
_attCtrl(
		_attGuid,
		_estVal,
		_torque_B,
		dt_HF,
		dftAttCtrlParam
		),
_mod(
		 _torque_B,
		 _force_B,
		 system::System::system.board.pwmVal,
		 _torqueReal_B,
		 _forceReal_B,
		 _param.modGen,
		 _param.modPinv,
		 system::System::system.board.pwm
		 ),
 _modeStabilitized(
		_estVal,
		_groundDetectOutput,
		_attGuid,
		_force_B,
		dt_LF,
		_param.modeStabilized,
		_param.gen,
		_attCtrl
		 ),
_gimbal(
		_estVal,
		param.gimbal
		)
{
}

Ancs::~Ancs() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
infra::status Ancs::initialize()
{
	_procCompDec.initialize();
	_procDetectGround.initialize();
	_mod.initialize();
	_est.initialize();
	_gimbal.initialize();
	execTransToInitCalibImu();
	return 0;
}

/** @brief Execute the process */
infra::status Ancs::execute()
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
	return 0;
}


/** @brief Execute the transition to E_STATE_INITIALIZATION_CALIB_IMU */
void Ancs::execTransToInitCalibImu()
{
 	/* Initialize the procedure */
	_procImuCalib.start();

	/* Change state to calib Imu */
	_state = E_STATE_INITIALIZATION_CALIB_IMU;

//	Serial.printf("E_STATE_INITIALIZATION_CALIB_IMU\n");
}



/** @brief Step initialization */
void Ancs::stepInitCalibImu()
{
	switch (_procImuCalib.getState())
	{
	case ProcCalibGyroBias::E_PROCCALIBIMU_OFF:
		/* Start calibration procedure */
		_procImuCalib.start();
		break;

	case ProcCalibGyroBias::E_PROCCALIBIMU_ENDED:
		/* Calibration is over (and successfull), switch to ready state */
		break;

	case ProcCalibGyroBias::E_PROCCALIBIMU_FAILED:
		/* Someone move the sensor during calibration procedure */
		_procImuCalib.stop();
		_procImuCalib.start();
		break;
	default:
		/* Process board */
		system::System::system.board.execute();
		/* Execute calibration procedure */
		_procImuCalib.onTick();
		break;
	}
}

/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_IMU */
void Ancs::evalTransFromInitCalibImu()
{
	if (ProcCalibGyroBias::E_PROCCALIBIMU_ENDED == _procImuCalib.getState())
	{
		/* Procedure is over, command transition to compass calibration */
		execTransToInitCalibCompass();
	}
}

/** @brief Execute transitions to E_STATE_INITIALIZATION_CALIB_COMPASS */
void Ancs::execTransToInitCalibCompass()
{
	/* Initialize the procedure */
	_procCompDec.initialize();

	/* Set new state to compass calibration */
	_state = E_STATE_INITIALIZATION_CALIB_COMPASS;

//	Serial.printf("E_STATE_INITIALIZATION_CALIB_COMPASS\n");
}

/** @brief Step Compass calibration state */
void Ancs::stepInitCalibCompass()
{
	switch (_procCompDec.getStatus())
	{
	case infra::Procedure::E_PROC_STATUS_OFF:
		/* Start calibration procedure */
		_procCompDec.start();
		break;

	case infra::Procedure::E_PROC_STATUS_RUNNING:
		/* Process board */
		system::System::system.board.execute();
		/* Execute calibration procedure */
		_procCompDec.execute();
		break;

	case infra::Procedure::E_PROC_STATUS_TERMINATED:
		/* Calibration is over (and successfull), switch to ready state */
		break;

	case infra::Procedure::E_PROC_STATUS_FAILED:
		/* Someone move the sensor during calibration procedure */
	default:
		/* or unexpected */
		_procCompDec.reset();
		break;
	}
}

/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_COMPASS */
void Ancs::evalTransFromInitCalibCompass()
{
	if (infra::Procedure::E_PROC_STATUS_TERMINATED == _procCompDec.getStatus())
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
	_forceReal_B(0., 0., 0.);
	_torqueReal_B(0., 0., 0.);

	/* Process PWM */
	for (idxMotor=0 ; idxMotor<CONFIG_NB_MOTOR ; idxMotor++)
	{
		system::System::system.board.pwmVal.channels[idxMotor] = MIN_PULSEWIDTH;
		system::System::system.board.pwm.force_out(idxMotor);
		system::System::system.board.pwm.disable_out(idxMotor);
	}

	/* Set new state to compass calibration */
	_state = E_STATE_READY;

//	Serial.printf("E_STATE_READY\n");
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
	_procDetectGround.execute();
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
	_forceReal_B(0., 0., 0.);
	_torqueReal_B(0., 0., 0.);

	/* Arm motors */
	_mod.arm();

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
	_mod.execute();

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
	_forceReal_B(0., 0., 0.);
	_torqueReal_B(0., 0., 0.);

	/* Disarm motors */
	_mod.disarm();

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
