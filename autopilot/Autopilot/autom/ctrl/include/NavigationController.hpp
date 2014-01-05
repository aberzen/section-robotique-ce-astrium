/*
 * NavigationController.hpp
 *
 *  Created on: 3 déc. 2013
 *      Author: Robotique
 */

#ifndef NAVIGATIONCONTROLLER_HPP_
#define NAVIGATIONCONTROLLER_HPP_

#include <autom/ctrl/include/ControllerPid3Axes.hpp>
#include <autom/guid/include/NavGuid.hpp>
#include <autom/est/include/Estimator.hpp>

namespace autom {


class NavigationController: public autom::ControllerPid3Axes {
public:
	NavigationController(
			/* Inputs */
			const NavGuid::Output& guid,
			const Estimator::Estimations& est,
			/* Outputs */
			::math::Vector3f& force_I,
			/* Parameters */
			 const float& dt,
			 const ControllerPid3Axes::Param& param
			);
	virtual ~NavigationController() ;

	/** @brief Update the control error */
	virtual void updateCtrlErr();

protected:
	/** @brief Guidance */
	const NavGuid::Output& _guid;

	/** @brief Estimations */
	const Estimator::Estimations& _est;
};

} /* namespace autom */

#endif /* NAVIGATIONCONTROLLER_HPP_ */
