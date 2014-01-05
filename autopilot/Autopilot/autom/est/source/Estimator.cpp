/*
 * Estimator.cpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Estimator.hpp"

namespace autom {

Estimator::Estimator(
		/* Inputs */
		const board::Board::Measurements& meas,
		/* Outputs */
		Estimator::Estimations& est,
		/* Parameters */
		const float& dt
		) :
		_meas(meas),
		_est(est),
		_dt(dt)
{
}

Estimator::~Estimator() {
}

/** @brief Init the process */
infra::status Estimator::initialize()
{
	this->_est.isConverged = false;
	return 0;
}


} /* namespace autom */
