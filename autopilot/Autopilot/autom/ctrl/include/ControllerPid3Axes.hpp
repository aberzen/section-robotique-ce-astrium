/*
 * ControllerPid3Axes.hpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPIS3AXES_HPP_
#define CONTROLLERPIS3AXES_HPP_

#include <infra/include/Types.hpp>
#include <math/include/Vector3.hpp>
#include <autom/ctrl/include/ControllerPid.hpp>
#include <infra/app/include/Process.hpp>

namespace autom {

class ControllerPid3Axes {
public:
	typedef struct ControllerPid3AxesParam {
		ControllerPid::Param x;
		ControllerPid::Param y;
		ControllerPid::Param z;
	} Param;
public:
	ControllerPid3Axes(
			/* Parameters */
			const float& dt,
			const Param& param
			) ;
	virtual ~ControllerPid3Axes();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Setter method of controller parameters */
	inline void setParam(const Param& param);

	/** @brief Compute the command */
	virtual void compCommand(
			const math::Vector3f& ctrlErr,
			const math::Vector3f& derivCtrlErr,
			math::Vector3f& command);

protected:

	/** @brief Output */
	const ControllerPid3Axes::Param& _param;

	/** @brief PID controller for axis X */
	ControllerPid _ctrl_x;

	/** @brief PID controller for axis Y */
	ControllerPid _ctrl_y;

	/** @brief PID controller for axis Z */
	ControllerPid _ctrl_z;
};

/** @brief Setter method of controller parameters */
void ControllerPid3Axes::setParam(const ControllerPid3Axes::Param& param)
{
	(ControllerPid3Axes::Param&)_param = param;
	_ctrl_x.setParam(_param.x);
	_ctrl_y.setParam(_param.y);
	_ctrl_z.setParam(_param.z);
}

} /* namespace autom */
#endif /* CONTROLLERPIS3AXES_HPP_ */
