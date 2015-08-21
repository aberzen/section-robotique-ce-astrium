/*
 * ControlCyclicTask.hpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#ifndef CONTROLCYCLICTASK_HPP_
#define CONTROLCYCLICTASK_HPP_
#include <math.h>
#include <infra/rtos/CyclicTask.hpp>
#include <hw/Screen/Screen.hpp>
#include <gcs/gcs/Gcs.hpp>
#include <gcs/channel/SerialChannel.hpp>


namespace system {

class ControlCyclicTask: public infra::CyclicTask {
public:
	ControlCyclicTask(
			const signed char * pcName,
	        unsigned portSHORT usStackDepth,
	        unsigned portBASE_TYPE uxPriority,
	        uint16_t period,
	        uint16_t delay);

	virtual ~ControlCyclicTask();

	/** @brief Non returning function executed as the task body function. */
	virtual void init(void);

protected:
	virtual void runCycle(void);

};

} /* namespace test */
#endif /* CONTROLCYCLICTASK_HPP_ */
