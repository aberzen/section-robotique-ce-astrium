/*
 * Mode.hpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#ifndef MODE_HPP_
#define MODE_HPP_

#include <arch/include/State.hpp>
#include <autom/ctrl/include/TrajCtrl.hpp>
#include <autom/guid/include/TrajGuid.hpp>
#include <autom/ctrl/include/AttCtrl.hpp>
#include <autom/guid/include/AttGuid.hpp>
#include <autom/est/include/Estimator.hpp>

namespace autom {

class Mode: public arch::State {
public:
	virtual ~Mode();

	/** @brief Initialize by executing the initialization procedure */
	virtual void initialize();

	/** @brief Execute the internal procedure */
	virtual void execute();

protected:
	Mode(
			autom::AttGuid* attGuid,
			autom::AttCtrl* attCtrl,
			autom::TrajGuid* trajGuid,
			autom::TrajCtrl* trajCtrl,
			autom::Estimator* est,
			arch::Transition** transitions = NULL,
			uint8_t nbTransitions = 0
			);

protected:
	autom::AttGuid* _attGuid;
	autom::AttCtrl* _attCtrl;
	autom::TrajGuid* _trajGuid;
	autom::TrajCtrl* _trajCtrl;
	autom::Estimator* _est;
};

} /* namespace autom */
#endif /* MODE_HPP_ */
