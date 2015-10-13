/*
 * AttitudeController.hpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_CTRL_ATTITUDECONTROLLER_HPP_
#define ATT_CTRL_ATTITUDECONTROLLER_HPP_

#include <math/Quaternion.hpp>
#include <autom/ctrl/ControllerPid3Axes.hpp>
#include <autom/filt/DiscreteFilter.hpp>

namespace attitude {

class AttitudeController {
public:
	typedef struct
	{
		autom::ControllerPid3Axes<float>::Parameter ctrl;
		float maxCosAngOverTwoErr;
		float maxSinAngOverTwoErr;
		autom::DiscreteFilter<float, float, 3, 3>::Parameter filterX;
		autom::DiscreteFilter<float, float, 3, 3>::Parameter filterY;
		autom::DiscreteFilter<float, float, 3, 3>::Parameter filterZ;
	} Parameter;

public:
	AttitudeController(
			const Parameter& param);
	virtual ~AttitudeController();

	/** @brief */
	void reset();

	/** @brief Execute the service */
	void execute();

protected:

	/** @brief 3 Axes PID controller */
	autom::ControllerPid3Axes<float> _ctrl;

	/** @brief Previous rate */
	math::Vector3f _rateDem_B_prev;

	/** @brief Parameter */
	const AttitudeController::Parameter& _param;

	/** @brief Filter for X axis */
	autom::DiscreteFilter<float, float, 3, 3> _filterX;

	/** @brief Filter for Y axis */
	autom::DiscreteFilter<float, float, 3, 3> _filterY;

	/** @brief Filter for Z axis */
	autom::DiscreteFilter<float, float, 3, 3> _filterZ;
};

} /* namespace attitude */

#endif /* ATT_CTRL_ATTITUDECONTROLLER_HPP_ */
