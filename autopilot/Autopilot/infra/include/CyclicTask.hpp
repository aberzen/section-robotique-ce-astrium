/*
 * CyclicTask.hpp
 *
 *  Created on: 18 mai 2013
 *      Author: Aberzen
 */

#ifndef CYCLICTASK_HPP_
#define CYCLICTASK_HPP_


#include "Task.hpp"
#include <semphr.h>

namespace infra {

class CyclicTask : public Task {
public:
	CyclicTask(
			const signed char * pcName,
	        unsigned portSHORT usStackDepth,
	        unsigned portBASE_TYPE uxPriority,
	        uint16_t period,
	        uint16_t delay);

	virtual ~CyclicTask();

	virtual void run(void);

	void tick();
protected:
	virtual void runCycle(void) = 0;

	void waitNextPeriod();

protected:

	/** @brief Execution period (in Cyclic Manager ticks) */
	uint16_t _periodCnf;

	/** @brief Execution period (in Cyclic Manager ticks) */
	uint16_t _periodTicks;

	/** @brief Delay before first execution (in Cyclic Manager ticks) */
	uint16_t _delayCnf;

	/** @brief Delay before first execution (in Cyclic Manager ticks) */
	uint16_t _delayTicks;

	/** @brief Number of missed deadlines (in Cyclic Manager ticks) */
	uint8_t _missed;

	/** @brief Number of missed deadlines (in Cyclic Manager ticks) */
	uint8_t _isWaiting;

	/** @brief Semaphore to synchronize on Cyclic manager */
	xSemaphoreHandle _synchSem;
	//xQueueHandle _synchSem;

};

} /* namespace infra */



#endif /* CYCLICTASK_HPP_ */
