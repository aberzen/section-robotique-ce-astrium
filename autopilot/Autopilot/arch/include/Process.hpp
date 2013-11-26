/*
 * Process.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#include <infra/include/Types.hpp>

namespace arch {

class Process {
public:
	Process();
	virtual ~Process();

	/** @brief Init the process */
	virtual status initialize() = 0;

	/** @brief Execute the process */
	virtual status execute() = 0;
};

} /* namespace arch */
#endif /* PROCESS_H_ */
