/*
 * ControllerPid3Axes.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */
#include <autom/ctrl/include/ControllerPid3Axes.hpp>

namespace autom {
/**
 * \brief Constructor for the PID 3 axes controller class
 *
 * \details Instantiate an object for the class ControllerPid3Axes
 *
 * \param[out] out		Reference to the produced output
 * \param[in]  dt		Constant reference to the sampling period
 * \param[in]  param	Constant rReference to the parameter structure
 */
ControllerPid3Axes::ControllerPid3Axes(
		/* Outputs */
		::math::Vector3f& out,
		/* Parameters */
		const float& dt,
		const Param& param
		)
: infra::Process(),
  _out(out),
  _param(param),
  _ctrlErr(0., 0., 0.),
  _ctrlErrDeriv(0., 0., 0.),
  _ctrl_x(_ctrlErr.x, _ctrlErrDeriv.x, out.x, dt, param.x),
  _ctrl_y(_ctrlErr.y, _ctrlErrDeriv.y, out.y, dt, param.y),
  _ctrl_z(_ctrlErr.z, _ctrlErrDeriv.z, out.z, dt, param.z) {
	} ;

ControllerPid3Axes::~ControllerPid3Axes() {
}

/** @brief Init the process */
void ControllerPid3Axes::initialize()
{
	/* Initialize per axis controllers */
	_ctrl_x.initialize();
	_ctrl_y.initialize();
	_ctrl_z.initialize();
}

/** @brief Execute the process */
void ControllerPid3Axes::execute()
{
	/* Compute contol error */
	updateCtrlErr();

	/* Execute the contollers */
	_ctrl_x.execute();
	_ctrl_y.execute();
	_ctrl_z.execute();
}

} /* namespace autom */
