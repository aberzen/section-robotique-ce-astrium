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

class ControllerPid3Axes : public infra::Process {
public:
	typedef struct ControllerPid3AxesParam {
		ControllerPid::Param x;
		ControllerPid::Param y;
		ControllerPid::Param z;
	} Param;
public:
	ControllerPid3Axes(
			/* Outputs */
			::math::Vector3f& out,
			/* Parameters */
			const float& dt,
			const Param& param
			) ;
	virtual ~ControllerPid3Axes();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute();

	/** @brief Execute the process */
	virtual void updateCtrlErr() = 0;

	/** @brief Setter method of controller parameters */
	inline void setParam(const Param& param);

protected:

	/** @brief Output */
	::math::Vector3f& _out;

	/** @brief Output */
	const ControllerPid3Axes::Param& _param;

	/** @brief Control error */
	::math::Vector3f _ctrlErr;

	/** @brief Control error derivative */
	::math::Vector3f _ctrlErrDeriv;

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
