/*
 * Estimator.cpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Estimator.hpp"

namespace autom {

Estimator::Estimator() :
		_position_I(0.,0.,0.),
		_velocity_I(0.,0.,0.),
		_rate_B(0.,0.,0.),
		_q_BI(1.,0.,0.,0.)
{
}

Estimator::~Estimator() {
}


} /* namespace autom */
