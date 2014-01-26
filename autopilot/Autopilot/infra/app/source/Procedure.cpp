/*
 * Procedure.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Procedure.hpp"

namespace infra {

Procedure::Procedure() :
	Process(),
	_status(E_PROC_STATUS_OFF)
	{
}

Procedure::~Procedure() {
}

/** @brief Start current procedure*/
infra::status Procedure::start() {
	_status = E_PROC_STATUS_RUNNING;
	return 0;
}

/** @brief Stop current procedure*/
infra::status Procedure::stop() {
	_status = E_PROC_STATUS_OFF;
	return 0;
}

/** @brief Start current procedure*/
infra::status Procedure::reset() {
	_status = E_PROC_STATUS_OFF;
	initialize();
	return 0;
}

/** @brief Execute the process */
infra::status Procedure::execute(void) {
	if (_status == E_PROC_STATUS_RUNNING)
		return this->step();
	return -1;
}

} /* namespace infra */
