/*
 * CyclicMgr.hpp
 *
 *  Created on: 18 mai 2013
 *      Author: Aberzen
 */

#ifndef CYCLICMGR_HPP_
#define CYCLICMGR_HPP_

#include <FreeRTOS.h>
#include <semphr.h>

#include "CyclicTask.hpp"

namespace infra {

#define CYCLIC_MGR_MAX_TASK		(4)

class CyclicMgr {
public:
	static void tickHook(void);
	static bool registerTask(CyclicTask* task);


public:
	static bool _areAllTasksInitialized;
	static uint8_t _nbRegisteredTasks;
	static CyclicTask* _registeredTasks[CYCLIC_MGR_MAX_TASK];

};

} /* namespace hw */
#endif /* CYCLICMGR_HPP_ */
