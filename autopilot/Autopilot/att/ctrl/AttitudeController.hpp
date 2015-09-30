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

namespace attitude {

class AttitudeController {
public:
	typedef struct
	{
		autom::ControllerPid3Axes<int32_t>::Parameter ctrl;
		float maxCosAngOverTwoErr;
		float maxSinAngOverTwoErr;
	} Parameter;

public:
	AttitudeController(
			const Parameter& param);
	virtual ~AttitudeController();

	/** @brief Execute the service */
	void execute();

protected:

	/** @brief 3 Axes PID controller */
	autom::ControllerPid3Axes<int32_t> _ctrl;

	/** @brief Parameter */
	const AttitudeController::Parameter& _param;

};

} /* namespace attitude */

#endif /* ATT_CTRL_ATTITUDECONTROLLER_HPP_ */
