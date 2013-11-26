/*
 * Screw.cpp
 *
 *  Created on: 24 sept. 2013
 *      Author: Aberzen
 */

#include "../include/Screw.hpp"

namespace physics {

Screw::Screw(
		math::Vector3f position,
		math::Vector3f field,
		math::Vector3f moment
		) :
	_position(position),
	_field(field),
	_moment(moment)
{
}

Screw::~Screw() {
}

/** Change position */
Screw Screw::changePosition(math::Vector3f newPosition)
{
	return Screw(
			newPosition,
			_field,
			_moment + (_position-newPosition)%_field
	);
}

/** Change position */
Screw Screw::rotate(math::Matrix3f dcm)
{
	return Screw(
			dcm * _position,
			dcm * _field,
			dcm * _moment
	);
}



} /* namespace physics */

