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
 * \param[in]  dt		Constant reference to the sampling period
 * \param[in]  param	Constant rReference to the parameter structure
 */
ControllerPid3Axes::ControllerPid3Axes (
		/* Parameters */
		const float& dt,
		const Param& param
		)
: _param(param),
  _ctrl_x(dt, param.x),
  _ctrl_y(dt, param.y),
  _ctrl_z(dt, param.z)
{
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
/** @brief Compute the command */
void ControllerPid3Axes::compCommand(
		const math::Vector3f& ctrlErr,
		const math::Vector3f& derivCtrlErr,
		math::Vector3f& command)
{
	/* Execute the contollers */
	_ctrl_x.compCommand(ctrlErr.x, derivCtrlErr.x, command.x);
	_ctrl_y.compCommand(ctrlErr.y, derivCtrlErr.y, command.y);
	_ctrl_z.compCommand(ctrlErr.z, derivCtrlErr.z, command.z);
}

} /* namespace autom */
