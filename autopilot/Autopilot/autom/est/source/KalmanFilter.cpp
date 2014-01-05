/*
 * KalmanFilter.cpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#include <autom/est/include/KalmanFilter.hpp>

namespace autom {

KalmanFilter::KalmanFilter(
		/* Inputs */
		const board::Board::Measurements& meas,
		/* Outputs */
		Estimator::Estimations& est,
		/* Parameters */
		const float& dt
		) :
		Estimator(meas, est, dt)
{
}

KalmanFilter::~KalmanFilter() {
}

/** @brief Execute the process */
infra::status KalmanFilter::execute() {
	/* Predict the state */
	this->predict();

	/* Compute innovation */
	this->computeInnov();

	/* Correct and update the state */
	this->update();

	return 0;
}

///** @brief Predict the state knowing the past */
//template <typename int32_t>
//status KalmanFilter::predict()
//{
//	return 0;
//}
//
///** @brief Compute innovation */
//template <typename int32_t>
//status KalmanFilter::computeInnov()
//{
//	return 0;
//}
//
///** @brief Update the state knowing */
//template <typename int32_t>
//status KalmanFilter::update()
//{
//	return 0;
//}

} /* namespace autom */
