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
#include <infra/app/include/Process.hpp>

namespace autom {

class Mode : public infra::Process {
public:
	Mode(
			/* Input */
			const Estimator::Estimations& est,
			/* Outputs */
			AttGuid::Output& attGuid,
			::math::Vector3f& force_B
			/* Parameters */
			);
	virtual ~Mode();

	/** @brief Init the process */
	virtual void initialize() = 0;

	/** @brief Execute the process */
	virtual void execute() = 0;

protected:

	/** @brief Estimation */
	const Estimator::Estimations& _est;

	/** @brief Attitude guidance */
	AttGuid::Output& _attGuid;

	/** @brief Force in body frame (input of modulator) */
	::math::Vector3f& _force_B;
};

} /* namespace autom */

#endif /* MODE_HPP_ */
