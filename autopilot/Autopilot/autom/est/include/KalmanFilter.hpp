/*
 * KalmanFilter.hpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#ifndef AHRSESTIMATOR_HPP_
#define AHRSESTIMATOR_HPP_

#include <autom/est/include/Estimator.hpp>

namespace autom {

class KalmanFilter : public autom::Estimator  {
public:
	KalmanFilter(
			/* Inputs */
			const board::Board::Measurements& meas,
			/* Outputs */
			Estimator::Estimations& est,
			/* Parameters */
			const float& dt
			);
	virtual ~KalmanFilter();

	/** @brief Execute the process */
	virtual void execute();

protected:

	/** @brief Predict the state knowing the past */
	virtual infra::status predict() = 0;

	/** @brief Compute innovation */
	virtual infra::status computeInnov() = 0;

	/** @brief Update the state knowing */
	virtual infra::status update() = 0;

};

} /* namespace autom */
#endif /* AHRSESTIMATOR_HPP_ */
