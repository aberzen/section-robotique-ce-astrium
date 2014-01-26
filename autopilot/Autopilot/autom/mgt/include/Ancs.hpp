/*
 * Ancs.hpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#ifndef ANCS_HPP_
#define ANCS_HPP_

#include <hw/serial/include/FastSerial.hpp>
#include <infra/app/include/Process.hpp>
#include <autom/est/include/SimpleAttitudeKalmanFilter.hpp>
#include <autom/proc/include/ProcCalibGyroBias.hpp>
#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <autom/mod/include/ModulatorPinv.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>
#include <autom/proc/include/ProcDetectContact.hpp>

namespace autom {

#define CONFIG_NB_MOTOR	4

class Ancs : public infra::Process {
public:
	typedef enum
	{
		E_STATE_UNDEFINED = 0,
		E_STATE_INITIALIZATION_CALIB_IMU,
		E_STATE_INITIALIZATION_CALIB_COMPASS,
		E_STATE_READY,
		E_STATE_FLYING,
		E_STATE_FAILSAFE
	} State;

	typedef struct
	{
		ControllerPid3Axes::Param att;
		ControllerPid3Axes::Param nav;
	} ModeParam;

	typedef struct
	{
		GenericParam gen;
		SimpleAttitudeKalmanFilter::Param est;
		ProcCalibGyroBias::Param procCalibImu;
		ProcCompassDeclination::Param procCompDec;
		ProcDetectContact::Param procGrdDetect;
		Modulator<CONFIG_NB_MOTOR>::ParamGen modGen;
		ModulatorPinv<CONFIG_NB_MOTOR>::ParamPinv modPinv;
		ModeStabilized::Param modeStabilized;
	} Param ;
public:
	Ancs(
			const float& dt_HF,
			const float& dt_LF,
			const Param& param);
	virtual ~Ancs();

	/** @brief Init the process */
	virtual ::infra::status initialize() ;

	/** @brief Execute the process */
	virtual ::infra::status execute() ;

	/** @brief Getter for estimation values */
	inline const Estimator::Estimations& getEstimationValues();

protected:
	/** @brief Execute the transition to E_STATE_INITIALIZATION_CALIB_IMU */
	void execTransToInitCalibImu();

	/** @brief Step initialization to E_STATE_INITIALIZATION_CALIB_IMU */
	void stepInitCalibImu();

	/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_IMU */
	void evalTransFromInitCalibImu();

	/** @brief Execute the transition to E_STATE_INITIALIZATION_CALIB_COMPASS */
	void execTransToInitCalibCompass();

	/** @brief Step initialization to E_STATE_INITIALIZATION_CALIB_COMPASS */
	void stepInitCalibCompass();

	/** @brief Assert transitions from E_STATE_INITIALIZATION_CALIB_COMPASS */
	void evalTransFromInitCalibCompass();

	/** @brief Execute the transition to E_STATE_READY */
	void execTransToReady();

	/** @brief Step to E_STATE_READY */
	void stepReady();

	/** @brief Assert transitions from E_STATE_READY */
	void evalTransFromReady();

	/** @brief Execute the transition to E_STATE_FLYING */
	void execTransToFlying();

	/** @brief Step to E_STATE_FLYING */
	void stepFlying();

	/** @brief Assert transitions from E_STATE_FLYING */
	void evalTransFromFlying();

	/** @brief Execute the transition to E_STATE_FAILSAFE */
	void execTransToFailsafe();

	/** @brief Step to E_STATE_FAILSAFE */
	void stepFailsafe();

	/** @brief Assert transitions from E_STATE_FAILSAFE */
	void evalTransFromFailSafe();

protected:
	/** @brief Setting */
	Param _param;

	/** @brief Demanded attitude guidance */
	autom::AttGuid::Output _attGuid;

	/** @brief Demanded torque */
	math::Vector3f _torque_B;

	/** @brief Demanded force */
	math::Vector3f _force_B;

	/** @brief Realized torque */
	math::Vector3f _torqueReal_B;

	/** @brief Realized force */
	math::Vector3f _forceReal_B;

	/** @brief Ground detection output */
	ProcDetectContact::Output _groundDetectOutput;

	/** @brief Estimation values */
	Estimator::Estimations _estVal;

	/** @brief Calibration of IMU bias procedure */
	ProcCalibGyroBias _procImuCalib;

	/** @brief Calibration of IMU bias procedure */
	ProcCompassDeclination _procCompDec;

	/** @brief Calibration of IMU bias procedure */
	ProcDetectContact _procDetectGround;


	/** @brief Simple estimator (for test purpose) */
	SimpleAttitudeKalmanFilter _est;

	/** @brief Attitude controller */
	AttitudeController _attCtrl;

	/** @brief Modulator */
	ModulatorPinv<CONFIG_NB_MOTOR> _mod;

	/** @brief Mode stabilized */
	ModeStabilized _modeStabilitized;

	/** @brief Current state */
	State _state;
};

/** @brief Getter for estimation values */
const Estimator::Estimations& Ancs::getEstimationValues()
{
	return _estVal;
}

} /* namespace autom */

#endif /* ANCS_HPP_ */
