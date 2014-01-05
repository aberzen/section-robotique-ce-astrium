/*
 * AttitudeController.hpp
 *
 *  Created on: 3 déc. 2013
 *      Author: Robotique
 */

#ifndef ATTITUDECONTROLLER_HPP_
#define ATTITUDECONTROLLER_HPP_

#include <autom/guid/include/AttGuid.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/ctrl/include/ControllerPid3Axes.hpp>

namespace autom {

class AttitudeController: public ControllerPid3Axes {
public:
	AttitudeController(
			/* Inputs */
			const AttGuid::Output& guid,
			const Estimator::Estimations& est,
			/* Outputs */
			::math::Vector3f& torque_B,
			/* Parameters */
			const float& dt,
			const ControllerPid3Axes::Param& param);

	virtual ~AttitudeController() ;

	/** @brief Update the control error */
	virtual void updateCtrlErr();

protected:

	/** @brief Guidance */
	const AttGuid::Output& _guid;

	/** @brief Estimations */
	const Estimator::Estimations& _est;
};

} /* namespace autom */

#endif /* ATTITUDECONTROLLER_HPP_ */
