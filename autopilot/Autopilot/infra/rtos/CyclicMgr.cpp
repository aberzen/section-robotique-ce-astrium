/*
 * CyclicMgr.cpp
 *
 *  Created on: 18 mai 2013
 *      Author: Aberzen
 */

#include "CyclicMgr.hpp"

namespace infra {


bool CyclicMgr::_areAllTasksInitialized = false;
uint8_t CyclicMgr::_nbRegisteredTasks = 0;
CyclicTask* CyclicMgr::_registeredTasks[CYCLIC_MGR_MAX_TASK] = {NULL, NULL, NULL, NULL};

void CyclicMgr::tickHook(void)
{
	uint8_t iRegisteredTasks;
	uint8_t nbReadyTasks = 0;
	if (_areAllTasksInitialized)
	{
		/* Tick all tasks */
		for (iRegisteredTasks=0 ; iRegisteredTasks<_nbRegisteredTasks ; iRegisteredTasks++)
		{
			_registeredTasks[iRegisteredTasks]->tick();
		}
	}
	else
	{
		/* Wait all tasks to initialize */
		for (iRegisteredTasks=0 ; iRegisteredTasks<_nbRegisteredTasks ; iRegisteredTasks++)
		{
			if (_registeredTasks[iRegisteredTasks]->isInitialized())
				nbReadyTasks++;
			//isAtLeastOneTaskNotInitialized = isAtLeastOneTaskNotInitialized || (! _registeredTasks[iRegisteredTasks]->isInitialized());
		}
		//_areAllTasksInitialized = ! isAtLeastOneTaskNotInitialized;
		_areAllTasksInitialized = (nbReadyTasks == _nbRegisteredTasks);
	}
}

bool CyclicMgr::registerTask(CyclicTask* task)
{
	if (_nbRegisteredTasks==CYCLIC_MGR_MAX_TASK)
	{
		return false;
	}

	_registeredTasks[_nbRegisteredTasks++] = task;
	return true;
}

extern "C" void vApplicationTickHook(){
		CyclicMgr::tickHook();
}

} /* namespace hw */
