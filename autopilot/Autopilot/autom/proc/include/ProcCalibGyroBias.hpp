/*
 * ProcCalibGyroBias.hpp
 *
 *  Created on: 26 déc. 2013
 *      Author: Robotique
 */

#ifndef PROCCALIBGYROBIAS_HPP_
#define PROCCALIBGYROBIAS_HPP_

#include <infra/app/include/Procedure.hpp>
#include <hw/imu/include/HalImu.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/filt/include/DiscreteFilter.hpp>

namespace autom {

class ProcCalibGyroBias : public infra::Procedure {
protected:
	typedef enum
	{
		E_PROCCALIBIMU_COMP_INIT_FILTERS, // Init filters
		E_PROCCALIBIMU_COMP_BIAS, // compute bias
		E_PROCCALIBIMU_COMP_VAR // verify variance
	} State ;
public:
	typedef struct {
		float filtCoeffNum[2];
		float filtCoeffDen[2];
		uint16_t biasNbMeas;
		uint16_t varNbMeas;
		float gyroVarianceThd;
		float accoVarianceThd;
	} Param ;
public:
	ProcCalibGyroBias(
		/* Input */
		const hw::HalImu::Output& imu,
		/* Output */
		autom::Estimator::Estimations& est,
		/* Param */
		const Param& param
	);
	virtual ~ProcCalibGyroBias();

	/** @brief Init the process */
	virtual infra::status initialize() ;

protected:
	/** @brief Execute current procedure step */
	virtual infra::status step();

	/** @brief Initialize the filters from first measurements */
	infra::status stepInitFilters();

	/** @brief Compute bias */
	infra::status stepCompBias();

	/** @brief Compute variance */
	infra::status stepCompVar();

protected:
	/** @brief Parameters */
	const Param& _param;

	/** @brief Imu measurements */
	const hw::HalImu::Output& _imu;

	/** @brief Estimation */
	Estimator::Estimations& _est;

	/** @brief Filters on gyro measurements */
	DiscreteFilter<float, float, 2, 2> _filtGyroX;
	DiscreteFilter<float, float, 2, 2> _filtGyroY;
	DiscreteFilter<float, float, 2, 2> _filtGyroZ;

	/** @brief Filter on acco measurements */
	DiscreteFilter<float, float, 2, 2> _filtAccoX;
	DiscreteFilter<float, float, 2, 2> _filtAccoY;
	DiscreteFilter<float, float, 2, 2> _filtAccoZ;

	/** @brief Sum of gyro measurements */
	::math::Vector3f _sumGyroMeas;

	/** @brief Sum of acco measurements */
	::math::Vector3f _sumAccoMeas;

	/** @brief Counter for mean computation */
	uint8_t _count;

	/** @brief State of the procedure */
	State _state;
};

} /* namespace autom */

#endif /* PROCCALIBGYROBIAS_HPP_ */
