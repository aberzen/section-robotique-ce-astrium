/*
 * Estimator.h
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#ifndef ESTIMATOR_HPP_
#define ESTIMATOR_HPP_

#include <board/gen/include/Board.hpp>
#include <math/include/Quaternion.hpp>
#include <infra/app/include/Process.hpp>

namespace autom {

class Estimator : public infra::Process {
public:

	typedef struct {
		bool isConverged;
		::math::Vector3<float> imuGyroBias_B;
		::math::Vector3<float> imuAccoBias_B;
		::math::Vector3<float> rate_B;
		::math::Quaternion attitude_IB;
		::math::Vector3<float> position_I;
		::math::Vector3<float> velocity_I;
	} Estimations;

public:
	Estimator(
			/* Inputs */
			const ::board::Board::Measurements& meas,
			/* Outputs */
			Estimations& est,
			/* Parameters */
			const float& dt
			);
	virtual ~Estimator();

	/** @brief Init the process */
	virtual infra::status initialize() ;

	/** @brief Execute the process */
	virtual infra::status execute() = 0;

protected:

	/** @brief Measurements */
	const ::board::Board::Measurements& _meas;

	/** @brief Estimations */
	Estimations& _est;

	/** @brief Period of the fsw */
	const float& _dt;
};

} /* namespace autom */
#endif /* ESTIMATOR_H_ */
