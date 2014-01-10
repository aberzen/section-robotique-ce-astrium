/*
 * SimpleAttitudeKalmanFilter.hpp
 *
 *  Created on: 9 déc. 2013
 *      Author: Robotique
 */

#ifndef SIMPLEATTITUDEKALMANFILTER_HPP_
#define SIMPLEATTITUDEKALMANFILTER_HPP_

#include <autom/est/include/KalmanFilter.hpp>
#include <autom/proc/include/ProcCompassDeclination.hpp>

namespace autom {

class SimpleAttitudeKalmanFilter : public KalmanFilter {
public:
	typedef struct
	{
		float gainAcco;
		float gainCompass;
		ProcCompassDeclination::Output declination;
	} Param;
public:
	SimpleAttitudeKalmanFilter(
			/* Inputs */
			const board::Board::Measurements& meas,
			/* Outputs */
			Estimator::Estimations& est,
			/* Parameters */
			const float& dt,
			const Param& param
			);
	virtual ~SimpleAttitudeKalmanFilter();

	/** @brief Init the process */
	virtual infra::status initialize() ;

protected:

	/** @brief Predict the state knowing the past */
	virtual infra::status predict();

	/** @brief Compute innovation */
	virtual infra::status computeInnov();

	/** @brief Update the state knowing */
	virtual infra::status update();

protected:

	/** @brief Predicted rate */
	const Param& _param;

	/** @brief Predicted rate */
	math::Vector3f _ratePred_B;

	/** @brief Predicted velocity */
	math::Vector3f _velPred_I;

	/** @brief Predicted position */
	math::Vector3f _posPred_I;

	/** @brief Predicted attitude */
	math::Quaternion _attitudePred_IB;

	/** @brief Predicted rate */
	math::Vector3f _attitudeInnov_B;

	/** @brief Predicted rate */
	float _attEstInvNrmPrev;

	/** @brief Previous inverse of the norm of the projection of compass*/
	float _invNrmMagProjPrev;
};

} /* namespace autom */

#endif /* SIMPLEATTITUDEKALMANFILTER_HPP_ */
