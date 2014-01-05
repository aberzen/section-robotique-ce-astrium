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
#include <autom/ctrl/include/AttitudeController.hpp>
#include <autom/ctrl/include/NavigationController.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/est/include/SimpleAttitudeKalmanFilter.hpp>
#include <autom/guid/include/AttGuid.hpp>
#include <autom/guid/include/NavGuid.hpp>
#include <autom/proc/include/ProcCalibGyroBias.hpp>
#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <autom/mod/include/Modulator.hpp>
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
		ModeParam stabilized;
		SimpleAttitudeKalmanFilter::Param est;
		ProcCalibGyroBias::Param procCalibImu;
		ProcCompassDeclination::Param procCompDec;
		Modulator<CONFIG_NB_MOTOR>::Param mod;
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

	/** @brief Getter for attitude guidance values */
	inline AttGuid::Output& getAttitudeGuidanceValues();

	/** @brief Getter for navigation guidance values */
	inline NavGuid::Output& getNavigationGuidanceValues();

	/** @brief Getter for the demanded torque in body frame */
	inline ::math::Vector3f& getDemandedTorqueB();

	/** @brief Getter for the demanded force in inertial frame */
	inline const ::math::Vector3f& getDemandedForceI();

	/** @brief Getter for the demanded force in body frame */
	inline ::math::Vector3f& getDemandedForceB();

	/** @brief Getter for the attitude controller */
	inline AttitudeController& getAttitudeController();

	/** @brief Getter for the navigation controller */
	inline NavigationController& getNavigationController();

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

	/** @brief Demanded force */
	math::Vector3f _force_I;

	/** @brief Demanded attitude data */
	AttGuid::Output _demAttData;

	/** @brief Demanded navigation data */
	NavGuid::Output _demNavData;

	/** @brief */
	Estimator::Estimations _estVal;

	/** @brief Attitude controller */
	AttitudeController _attCtrl;

	/** @brief Navigation controller */
	NavigationController _navCtrl;

	/** @brief Calibration of IMU bias procedure */
	ProcCalibGyroBias _procImuCalib;

	/** @brief Calibration of IMU bias procedure */
	ProcCompassDeclination _procCompDec;

	/** @brief Simple estimator (for test purpose) */
	SimpleAttitudeKalmanFilter _est;

	/** @brief Modulator */
	Modulator<CONFIG_NB_MOTOR> _mod;

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

/** @brief Getter for attitude guidance values */
AttGuid::Output& Ancs::getAttitudeGuidanceValues()
{
	return _demAttData;
}

/** @brief Getter for navigation guidance values */
NavGuid::Output& Ancs::getNavigationGuidanceValues()
{
	return _demNavData;
}

/** @brief Getter for the demanded torque in body frame */
::math::Vector3f& Ancs::getDemandedTorqueB()
{
	return _torque_B;
}

/** @brief Getter for the demanded force in inertial frame */
const ::math::Vector3f& Ancs::getDemandedForceI()
{
	return _force_I;
}

/** @brief Getter for the demanded force in body frame */
::math::Vector3f& Ancs::getDemandedForceB()
{
	return _force_B;
}

/** @brief Getter for the attitude controller */
AttitudeController& Ancs::getAttitudeController()
{
	return _attCtrl;
}

/** @brief Getter for the navigation controller */
NavigationController& Ancs::getNavigationController()
{
	return _navCtrl;
}


} /* namespace autom */

#endif /* ANCS_HPP_ */
