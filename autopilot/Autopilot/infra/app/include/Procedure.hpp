/*
 * Procedure.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef PROCEDURE_HPP_
#define PROCEDURE_HPP_

#include "Process.hpp"

namespace infra {

class Procedure : public Process{
public:
	typedef enum {
		E_PROC_STATUS_OFF = 0,
		E_PROC_STATUS_RUNNING,
		E_PROC_STATUS_TERMINATED,
		E_PROC_STATUS_FAILED
	} TStatus;
public:
	Procedure();
	virtual ~Procedure();

	/** @brief Start current procedure*/
	virtual void start();

	/** @brief Stop current procedure */
	virtual void stop();

	/** @brief Execute the process */
	virtual void execute();

	/** @brief Reset procedure step */
	virtual void reset();

	/** @brief Is Running ? */
	inline TStatus getStatus();

protected:
	/** @brief Execute current procedure step */
	virtual infra::status step() = 0;

	/** @brief Mark procedure as failed */
	inline void signalFailure();

protected:
	TStatus _status;
};

/** @brief Is Running ? */
Procedure::TStatus Procedure::getStatus() {
	return _status;
}

/** @brief Mark procedure as failed */
void Procedure::signalFailure() {
	_status = E_PROC_STATUS_FAILED;
}

} /* namespace infra */
#endif /* PROCEDURE_HPP_ */
