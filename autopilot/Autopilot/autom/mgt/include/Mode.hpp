/*
 * Mode.hpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#ifndef MODE_HPP_
#define MODE_HPP_

#include <autom/est/include/Estimator.hpp>
#include <autom/ctrl/include/AttitudeController.hpp>
#include <autom/ctrl/include/NavigationController.hpp>
#include <arch/app/include/Process.hpp>

namespace autom {

class Mode : public infra::Process {
public:
	Mode(
			/* Input */
			const Estimator::Estimations& est,
			const ::math::Vector3f& force_I,
			/* Outputs */
			AttGuid::Output& attCtrlIn,
			NavGuid::Output& navCtrlIn,
			::math::Vector3f& force_B,
			/* Parameters */
			const ControllerPid3Axes::Param& paramAttCtrl,
			const ControllerPid3Axes::Param& paramNavCtrl,
			/* Dependencies */
			AttitudeController& attCtrl,
			NavigationController& navCtrl
			);
	virtual ~Mode();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute() = 0;

protected:

	/** @brief Estimation */
	const Estimator::Estimations& _est;

	/** @brief Output of navigation controller */
	const ::math::Vector3f& _force_I;

	/** @brief Input of attitude controller */
	AttGuid::Output& _attCtrlIn;

	/** @brief Input of navigation controller */
	NavGuid::Output& _navCtrlIn;

	/** @brief Input of modulator controller */
	::math::Vector3f& _force_B;

	/** @brief Attitude controller */
	AttitudeController& _attCtrl;

	/** @brief Navigation controller */
	NavigationController& _navCtrl;

	/** @brief Parameters for the attitude controller */
	const ControllerPid3Axes::Param& _paramAttCtrl;

	/** @brief Parameters for the navigation controller */
	const ControllerPid3Axes::Param& _paramNavCtrl;

};

} /* namespace autom */

#endif /* MODE_HPP_ */
