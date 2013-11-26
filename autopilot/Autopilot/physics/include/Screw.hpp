/*
 * Screw.hpp
 *
 *  Created on: 24 sept. 2013
 *      Author: Aberzen
 */

#ifndef SCREW_HPP_
#define SCREW_HPP_

#include "../../math/include/Matrix3.hpp"

namespace physics {

class Screw {
public:
	Screw(
			math::Vector3f position,
			math::Vector3f field,
			math::Vector3f moment
	);
	virtual ~Screw();

	/** Change position */
	Screw changePosition(math::Vector3f newPosition);

	/** Change position */
	Screw rotate(math::Matrix3f dcm);

	/** Get position */
	inline const math::Vector3f& getPosition();

	/** Get field */
	inline const math::Vector3f& getField();

	/** Get moment */
	inline const math::Vector3f& getMoment();

protected:
	math::Vector3f _position;
	math::Vector3f _field;
	math::Vector3f _moment;

};

/** Get position */
inline const math::Vector3f& Screw::getPosition()
{
	return _position;
}

/** Get field */
inline const math::Vector3f& Screw::getField()
{
	return _field;
}

/** Get moment */
inline const math::Vector3f& Screw::getMoment()
{
	return _moment;
}

} /* namespace physics */
#endif /* SCREW_HPP_ */
