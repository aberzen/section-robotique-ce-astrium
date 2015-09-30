/*
 * AttitudeGuidance.hpp
 *
 *  Created on: 21 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCE_HPP_
#define ATT_GUID_ATTITUDEGUIDANCE_HPP_

#include <math/Quaternion.hpp>
#include <hw/pwm/Pwm.hpp>

#define ATTITUDE_GUIDANCE_IDX_ROLL   (0)
#define ATTITUDE_GUIDANCE_IDX_PITCH  (1)
#define ATTITUDE_GUIDANCE_IDX_YAW    (2)

namespace attitude {

class AttitudeGuidance {
public:

	AttitudeGuidance();
	virtual ~AttitudeGuidance();

	/** @brief Initialize internal state */
	virtual void initialize(
			const math::Quaternion& quat_IB,
			const math::Vector3f& rate_B) = 0;
};


} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCE_HPP_ */
