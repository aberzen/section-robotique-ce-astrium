/*
 * Process.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef PROCESS432_H_
#define PROCESS432_H_

#include <infra/include/Types.hpp>

namespace infra {

class Process {

public:
	Process();
	virtual ~Process();

	/** @brief Init the process */
	virtual void initialize() = 0;

	/** @brief Execute the process */
	virtual void execute() = 0;
};

} /* namespace infra */
#endif /* PROCESS_H_ */
