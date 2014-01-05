/*
 * ProcCompassDeclination.hpp
 *
 *  Created on: 30 déc. 2013
 *      Author: Robotique
 */

#ifndef PROCCOMPASSDECLINATION_HPP_
#define PROCCOMPASSDECLINATION_HPP_

#include <arch/app/include/Procedure.hpp>
#include <board/gen/include/Board.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/filt/include/DiscreteFilter.hpp>

namespace autom {

class ProcCompassDeclination : public infra::Procedure {
public:
	typedef struct
	{
		float filtCoeffNum[2];
		float filtCoeffDen[2];
		uint16_t nbMeas;
	} Param ;

	typedef struct
	{
		float invNrm;
	} Output ;
public:
	ProcCompassDeclination(
			/* Input */
			const board::Board::Measurements& meas,
			/* Input / Output */
			autom::Estimator::Estimations& est,
			/* Output */
			Output& out,
			/* Param */
			const Param& param
			);
	virtual ~ProcCompassDeclination();

	/** @brief Init the process */
	virtual infra::status initialize() ;

protected:
	/** @brief Execute current procedure step */
	virtual infra::status step();

protected:

	/** @brief Measurements */
	const board::Board::Measurements& _meas;

	/** @brief Estimation */
	autom::Estimator::Estimations& _est;

	/** @brief Output */
	Output& _out;

	/** @brief Param */
	const Param& _param;

	/** @brief Filter (low pass) to estimate the declination (X axis) */
	autom::DiscreteFilter<float, float, 2, 2> _filtX;

	/** @brief Count */
	uint16_t _count;

};

} /* namespace autom */

#endif /* PROCCOMPASSDECLINATION_HPP_ */
