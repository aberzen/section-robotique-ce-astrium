/*
 * Ancs.hpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#ifndef ANCS_HPP_
#define ANCS_HPP_

#include <infra/app/include/Process.hpp>
#include <autom/est/include/SimpleAttitudeKalmanFilter.hpp>
#include <autom/proc/include/ProcCalibGyroBias.hpp>
#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <autom/mod/include/ModulatorPinv.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>
#include <autom/sm/include/GroundContactState.hpp>
#include <autom/gimbal/include/GimbalMgt.hpp>
#include <autom/radio/include/RadioChannel.hpp>
#include <autom/ctrl/include/AttitudeController.hpp>
#include <autom/mgt/include/ModeManagement.hpp>
#include <autom/sm/include/FlyingState.hpp>
#include <autom/sm/include/GlobalState.hpp>

namespace autom {

#define CONFIG_NB_MOTOR	4

class Ancs {
public:

	typedef struct
	{
		GenericParam gen;
		SimpleAttitudeKalmanFilter::Param est;
		ProcCalibGyroBias::Param procCalibImu;
		ProcCompassDeclination::Param procCompDec;
		GroundContactState::Param smGroundContact;
		FlyingState::Param smFLying;
		Modulator<CONFIG_NB_MOTOR>::ParamGen modGen;
		ModulatorPinv<CONFIG_NB_MOTOR>::ParamPinv modPinv;
		ControllerPid3Axes::Param attCtrl;
		ModeManagement::Param modeMgt;
		GimbalMgt::Param gimbal;
		RadioChannel::Param radioChannel[PWM_OUT_NUM_CHANNELS];
	} Param ;
public:
	Ancs(
			const float& dt_HF,
			const float& dt_LF,
			const Param& param);
	virtual ~Ancs();

public:

	/** @brief Calibration of IMU bias procedure */
	ProcCalibGyroBias procImuCalib;

	/** @brief Calibration of IMU bias procedure */
	ProcCompassDeclination procCompDec;

	/** @brief Modulator */
	ModulatorPinv<CONFIG_NB_MOTOR> modulator;

	/** @brief Realized torque */
	math::Vector3f torqueReal_B;

	/** @brief Realized force */
	math::Vector3f forceReal_B;

	/** @brief Estimation values */
	Estimator::Estimations estimations;

	/** @brief Ground detection */
	GroundContactState smGroundContact;

	/** @brief Arming / disarming */
	FlyingState smFlyingState;

	/** @brief Global state machine */
	GlobalState smGlobal;

	/** @brief Calibration of IMU bias procedure */
	RadioChannel* radioChannels[PWM_OUT_NUM_CHANNELS];

	/** @brief Simple estimator (for test purpose) */
	SimpleAttitudeKalmanFilter est;

	/** @brief Attitude controller */
	AttitudeController attCtrl;

	/** @brief Gimbal management */
	GimbalMgt gimbal;

	/** @brief Demanded attitude guidance */
	autom::AttitudeController::Input attGuid;

	/** @brief Demanded torque */
	math::Vector3f torque_B;

	/** @brief Demanded force */
	math::Vector3f force_B;

	/** @brief Demanded force */
	ModeManagement modeMgt;

protected:
	/** @brief Setting */
	Param _param;
};

} /* namespace autom */

#endif /* ANCS_HPP_ */
