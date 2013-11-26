/*
 * Estimator.h
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include <math/include/Vector3.hpp>
#include <math/include/Quaternion.hpp>
#include <arch/include/Process.hpp>

namespace autom {

class Estimator : public arch::Process {
public:
	Estimator();
	virtual ~Estimator();

	/** @brief Execute the process */
	virtual status execute() = 0;

	/** @brief Get position in reference frame */
	inline const math::Vector3f& getPosition();

	/** @brief Get velocity in reference frame */
	inline const math::Vector3f& getVelocity();

	/** @brief Get angular rate of body frame w.r.t. reference frame in body frame */
	inline const math::Vector3f& getAngRate();

	/** @brief Get rotation from inertial to body frame */
	inline const math::Quaternion& getAttitude();

protected:

	/** @brief Position in reference frame */
	math::Vector3f _position_I;

	/** @brief Velocity in reference frame */
	math::Vector3f _velocity_I;

	/** @brief Angular rate of body frame w.r.t. reference frame in body frame */
	math::Vector3f _rate_B;

	/** @brief Rotation from inertial to body frame */
	math::Quaternion _q_BI;
};

/** @brief Get position in reference frame */
inline const math::Vector3f& Estimator::getPosition()
{
	return _position_I;
}

/** @brief Get velocity in reference frame */
inline const math::Vector3f& Estimator::getVelocity()
{
	return _velocity_I;
}

/** @brief Get angular rate of body frame w.r.t. reference frame in body frame */
inline const math::Vector3f& Estimator::getAngRate()
{
	return _rate_B;
}

/** @brief Get rotation from inertial to body frame */
inline const math::Quaternion& Estimator::getAttitude()
{
	return _q_BI;
}

} /* namespace autom */
#endif /* ESTIMATOR_H_ */
