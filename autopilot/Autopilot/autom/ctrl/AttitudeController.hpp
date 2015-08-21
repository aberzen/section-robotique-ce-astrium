/*
 * AttitudeController.hpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef AUTOM_CTRL_ATTITUDECONTROLLER_HPP_
#define AUTOM_CTRL_ATTITUDECONTROLLER_HPP_

#include <math/Quaternion.hpp>
#include "ControllerPid3Axes.hpp"

namespace autom {

class AttitudeController {
public:
	AttitudeController(
			const math::Vector3<int32_t>& Kp,
			const math::Vector3<int32_t>& Kd,
			const math::Vector3<int32_t>& Ki,
			const math::Vector3<int32_t>& maxI);
	virtual ~AttitudeController();

	/** @brief Compute torque */
	void computeTorque(
			math::Quaternion qDem_IB,
			math::Quaternion qEst_IB,
			math::Vector3f rateDem_B,
			math::Vector3f rateEst_B);

protected:

	/** @brief 3 Axes PID controller */
	ControllerPid3Axes<int32_t> _ctrl;

};

} /* namespace autom */

#endif /* AUTOM_CTRL_ATTITUDECONTROLLER_HPP_ */
