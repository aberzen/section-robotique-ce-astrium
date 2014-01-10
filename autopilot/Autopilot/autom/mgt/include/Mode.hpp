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
			/* Outputs */
			::math::Vector3f& torque_B,
			::math::Vector3f& force_B
			/* Parameters */
			);
	virtual ~Mode();

	/** @brief Init the process */
	virtual ::infra::status initialize() = 0;

	/** @brief Execute the process */
	virtual ::infra::status execute() = 0;

protected:

	/** @brief Estimation */
	const Estimator::Estimations& _est;

	/** @brief Force in body frame (input of modulator) */
	::math::Vector3f& _force_B;

	/** @brief Torque in body frame (input of modulator) */
	::math::Vector3f& _torque_B;
};

} /* namespace autom */

#endif /* MODE_HPP_ */
