/*
 * Dynamics.cpp
 *
 *  Created on: 3 oct. 2015
 *      Author: AdministrateurLocal
 */

#include <system/params/Dynamics.hpp>

namespace system {

Dynamics::Dynamics(const Parameter& param)
: _param(param)
{
}

Dynamics::~Dynamics() {
	// TODO Auto-generated destructor stub
}

/** @brief Get diagonal inertia */
void Dynamics::getDiagInertia(math::Vector3f& inertia) const
{
	inertia(
			_param.diagInertia[0],
			_param.diagInertia[1],
			_param.diagInertia[2]);
}

/** @brief Get Center of Mass position in body frame */
void Dynamics::getCenterOfMass(math::Vector3f& position_B) const
{
	position_B(
			_param.comPositionB[0],
			_param.comPositionB[1],
			_param.comPositionB[2]);
}


} /* namespace system */
