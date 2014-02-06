/*
 * ProcCompassDeclination.hpp
 *
 *  Created on: 30 déc. 2013
 *      Author: Robotique
 */

#ifndef PROCCOMPASSDECLINATION_HPP_
#define PROCCOMPASSDECLINATION_HPP_

#include <board/gen/include/Board.hpp>
#include <autom/est/include/Estimator.hpp>
#include <autom/filt/include/DiscreteFilter.hpp>

namespace autom {

class ProcCompassDeclination {
public:
	typedef enum
	{
		E_STATE_OFF = 0,
		E_STATE_INIT,
		E_STATE_RUNNING,
		E_STATE_ENDED
	} State ;

	typedef struct
	{
		autom::DiscreteFilter<float, float, 2, 2>::Param filt;
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

	/** @brief Start the procedure */
	void start() ;

	/** @brief Stop the procedure */
	void stop() ;

	/** @brief Reset the procedure */
	void reset() ;

	/** @brief Handle one tick */
	void onTick() ;

	/** @brief Getter method for current state */
	inline State getState();

protected:

	/** @brief Current state */
	State _state;

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

/** @brief Getter method for current state */
inline ProcCompassDeclination::State ProcCompassDeclination::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* PROCCOMPASSDECLINATION_HPP_ */
