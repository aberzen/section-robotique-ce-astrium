/*
 * GimbalMgt.hpp
 *
 *  Created on: 29 janv. 2014
 *      Author: Robotique
 */

#ifndef GIMBALMGT_HPP_
#define GIMBALMGT_HPP_

#include <infra/app/include/Process.hpp>
#include <hw/servo/include/Servo.hpp>
#include <autom/est/include/Estimator.hpp>

namespace autom {

class GimbalMgt : public infra::Process {
public:
	typedef struct
	{
		hw::Servo::Param pitch;
		hw::Servo::Param roll;
	} Param;
public:
	GimbalMgt(
			/* Input */
			const autom::Estimator::Estimations& estVal,
			/* Output */
			/* Param */
			const Param& param
			);
	virtual ~GimbalMgt();

	/** @brief Init the process */
	virtual ::infra::status initialize() ;

	/** @brief Execute the process */
	virtual ::infra::status execute() ;

protected:
	/** @brief Estimated values */
	const autom::Estimator::Estimations& _estVal;

	/** @brief Servo for pitch */
	hw::Servo _pitch;

	/** @brief Servo for roll */
	hw::Servo _roll;
};

} /* namespace autom */

#endif /* GIMBALMGT_HPP_ */
