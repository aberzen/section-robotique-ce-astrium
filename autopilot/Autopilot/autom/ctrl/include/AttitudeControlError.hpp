/*
 * AttitudeControlError.hpp
 *
 *  Created on: 3 déc. 2013
 *      Author: Robotique
 */

#ifndef ATTITUDECONTROLERROR_HPP_
#define ATTITUDECONTROLERROR_HPP_

#include <math/include/Quaternion.hpp>

namespace autom {

class AttitudeControlError {
public:
	/** @brief Compute the attitude control error (small angle approximation) */
	static void compControlError(
			const math::Quaternion& qDem_IB,
			const math::Quaternion& qEst_IB,
			math::Vector3f& error);
};

} /* namespace autom */

#endif /* ATTITUDECONTROLERROR_HPP_ */
