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
void Procedure::start() {
	_status = E_PROC_STATUS_RUNNING;
}

/** @brief Stop current procedure*/
void Procedure::stop() {
	_status = E_PROC_STATUS_OFF;
}

/** @brief Start current procedure*/
void Procedure::reset() {
	_status = E_PROC_STATUS_OFF;
	initialize();
}

/** @brief Execute the process */
void Procedure::execute(void) {
	if (_status == E_PROC_STATUS_RUNNING)
		this->step();
}

} /* namespace infra */
