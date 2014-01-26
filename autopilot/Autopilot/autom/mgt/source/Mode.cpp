/*
 * Mode.cpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#include <autom/mgt/include/Mode.hpp>

namespace autom {

Mode::Mode(
		/* Input */
		const Estimator::Estimations& est,
		/* Outputs */
		AttGuid::Output& attGuid,
		::math::Vector3f& force_B
		/* Parameters */
		)
: _est(est),
  _force_B(force_B),
  _attGuid(attGuid)
{
}

Mode::~Mode() {
}

} /* namespace autom */
