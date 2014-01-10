/*
 * Ancs.hpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#ifndef ANCS_HPP_
#define ANCS_HPP_

#include <hw/serial/include/FastSerial.hpp>
#include <arch/app/include/Process.hpp>
#include <autom/est/include/SimpleAttitudeKalmanFilter.hpp>
#include <autom/proc/include/ProcCalibGyroBias.hpp>
#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <autom/mod/include/ModulatorPinv.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>

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
		SimpleAttitudeKalmanFilter::Param est;
		ProcCalibGyroBias::Param procCalibImu;
		ProcCompassDeclination::Param procCompDec;
		ModulatorPinv<CONFIG_NB_MOTOR>::Param mod;
		autom::ModeStabilized::Param modeStabilized;
	} Param ;
public:
	Ancs(
			const float& dt,
			const Param& param);
	virtual ~Ancs();

	/** @brief Init the process */
	virtual ::infra::status initialize() ;

	/** @brief Execute the process */
	virtual ::infra::status execute() ;

	/** @brief Getter for estimation values */
	inline const Estimator::Estimations& getEstimationValues();

protected:
	/** @brief Step initialization IMU calibration */
	infra::status stepInitializationCalibImu();

	/** @brief Step initialization IMU calibration */
	infra::status stepInitializationCalibCompass();

	/** @brief Step ready */
	infra::status stepReady();

	/** @brief Step flying */
	infra::status stepFlying();

	/** @brief Step fail safe */
	infra::status stepFailsafe();

	/** @brief Assert transitions */
	infra::status assertTransitions();

protected:
	/** @brief Setting */
	Param _param;

	/** @brief Demanded torque */
	math::Vector3f _torque_B;

	/** @brief Demanded force */
	math::Vector3f _force_B;

	/** @brief */
	Estimator::Estimations _estVal;

	/** @brief Calibration of IMU bias procedure */
	ProcCalibGyroBias _procImuCalib;

	/** @brief Calibration of IMU bias procedure */
	ProcCompassDeclination _procCompDec;

	/** @brief Simple estimator (for test purpose) */
	SimpleAttitudeKalmanFilter _est;

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
