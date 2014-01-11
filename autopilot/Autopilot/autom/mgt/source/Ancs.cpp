/*
 * Ancs.cpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#include <autom/mgt/include/Ancs.hpp>
#include <board/gen/include/Board.hpp>

#include <infra/include/Task.hpp>

namespace autom {

Ancs::Ancs(
		const float& dt,
		const Ancs::Param& param) :
_param(param),
_procImuCalib(
		::board::Board::board.meas.imu,
		_estVal,
		_param.procCalibImu
		),
_procCompDec(
		::board::Board::board.meas,
		_estVal,
		_param.est.declination,
		_param.procCompDec
		),
 _est(
		::board::Board::board.meas,
		_estVal,
		dt,
		_param.est
 	 	),
 _mod(
		 _torque_B,
		 _force_B,
		 board::Board::board.pwmVal,
		 _torqueReal_B,
		 _forceReal_B,
		 _param.modGen,
		 _param.modPinv
		 ),
 _modeStabilitized(
		_estVal,
		_torque_B,
		_force_B,
		dt,
		_param.modeStabilized,
		_param.gen
		 )
{
}

Ancs::~Ancs() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
infra::status Ancs::initialize()
{
	_state = E_STATE_INITIALIZATION_CALIB_IMU;
	_procImuCalib.initialize();
	_procCompDec.initialize();
	_mod.initialize();
	_est.initialize();
	return 0;
}

/** @brief Execute the process */
infra::status Ancs::execute()
{
	/* Assert transition to flying mode */
	assertTransitions();
	switch (_state)
	{
	case E_STATE_INITIALIZATION_CALIB_IMU:
		return stepInitializationCalibImu();
		break;
	case E_STATE_INITIALIZATION_CALIB_COMPASS:
		return stepInitializationCalibCompass();
		break;
	case E_STATE_READY:
		return stepReady();
		break;
	case E_STATE_FLYING:
		return stepFlying();
		break;
	case E_STATE_UNDEFINED:
	case E_STATE_FAILSAFE:
	default: /* Unexpected */

		/* Execute failsafe */
		return stepFailsafe();
		break;
	}
	return 0;
}

/** @brief Assert if a transition is crossable */
infra::status Ancs::assertTransitions()
{
	switch (_state)
	{
	case E_STATE_INITIALIZATION_CALIB_IMU:
		if (_procImuCalib.getStatus() == infra::Procedure::E_PROC_STATUS_TERMINATED)
		{
			_procCompDec.initialize();
			_state = E_STATE_INITIALIZATION_CALIB_COMPASS;
		}
		break;
	case E_STATE_INITIALIZATION_CALIB_COMPASS:
		if (_procCompDec.getStatus() == infra::Procedure::E_PROC_STATUS_TERMINATED)
		{
			_state = E_STATE_READY;
		}
		break;
	case E_STATE_READY:
		if (true)
		{
			board::Board::board.getPwm().enable_out(0);
			board::Board::board.getPwm().enable_out(1);
			board::Board::board.getPwm().enable_out(2);
			board::Board::board.getPwm().enable_out(3);

			_modeStabilitized.initialize();
			_state = E_STATE_FLYING;
		}
		break;
	case E_STATE_FLYING:
		/* todo: process surveillance and landing */
		break;
	case E_STATE_UNDEFINED:
		_state = E_STATE_FAILSAFE;
		break;
	case E_STATE_FAILSAFE:
		/* stay in failsafe */
		break;
	default:
		/* Unexpected thus failsafe */
		_state = E_STATE_FAILSAFE;
		break;
	}
	return 0;
}

/** @brief Step initialization */
infra::status Ancs::stepInitializationCalibImu()
{
	switch (_procImuCalib.getStatus())
	{
	case infra::Procedure::E_PROC_STATUS_OFF:
		/* Start calibration procedure */
		_procImuCalib.start();
		break;

	case infra::Procedure::E_PROC_STATUS_RUNNING:
		/* Process board */
		board::Board::board.execute();
		/* Execute calibration procedure */
		_procImuCalib.execute();
		break;

	case infra::Procedure::E_PROC_STATUS_TERMINATED:
		/* Calibration is over (and successfull), switch to ready state */
		break;

	case infra::Procedure::E_PROC_STATUS_FAILED:
		/* Someone move the sensor during calibration procedure */
	default:
		/* or unexpected */
		_procImuCalib.reset();
		return -1;
		break;
	}

	return 0;
}

/** @brief Step initialization */
infra::status Ancs::stepInitializationCalibCompass()
{
	switch (_procCompDec.getStatus())
	{
	case infra::Procedure::E_PROC_STATUS_OFF:
		/* Start calibration procedure */
		_procCompDec.start();
		break;

	case infra::Procedure::E_PROC_STATUS_RUNNING:
		/* Process board */
		board::Board::board.execute();
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
		return -1;
		break;
	}

	return 0;
}

/** @brief Step ready */
infra::status Ancs::stepReady()
{
	/* Set demanded force and torque to null*/
	_force_B(0., 0., 0.);
	_torque_B(0., 0., 0.);

	/* Process board */
	board::Board::board.execute();

	/* Compute estimation */
	_est.execute();

	/* Process PWM */
	_mod.execute();

	return 0;
}

/** @brief Step flying */
infra::status Ancs::stepFlying()
{
	/* Process board */
	board::Board::board.execute();

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
//	Serial.printf("%d %d %d %d %d\n", board::Board::board.radio.isAvailable, board::Board::board.radio.channels[0], board::Board::board.radio.channels[1], board::Board::board.radio.channels[2], board::Board::board.radio.channels[3]);
//	Serial.printf("%.5f;%.5f;%.5f;%.5f;%.5f;%.5f\n", _torque_B.x, _torque_B.y, _torque_B.z, _force_B.x, _force_B.y, _force_B.z);
//	Serial.printf("%d %d %d %d\n", board::Board::board.pwmVal.channels[0], board::Board::board.pwmVal.channels[1], board::Board::board.pwmVal.channels[2], board::Board::board.pwmVal.channels[3]);

	return 0;
}

/** @brief Step fail safe */
infra::status Ancs::stepFailsafe()
{
	return 0;
}

} /* namespace autom */
