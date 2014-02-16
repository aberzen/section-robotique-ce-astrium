/*
 * FrameScheduling.cpp
 *
 *  Created on: 10 févr. 2014
 *      Author: Robotique
 */

#include <stddef.h>
#include <infra/sched/include/FrameScheduling.hpp>

namespace infra {

FrameScheduling::FrameScheduling()
: Process(),
  _first(NULL),
  _current(NULL)
{
}

FrameScheduling::FrameScheduling(const FrameScheduling::Frame* frame)
: Process(),
  _first(frame),
  _current(frame)
{
}

FrameScheduling::~FrameScheduling() {
	// TODO Auto-generated destructor stub
}


/** @brief Init the process */
void FrameScheduling::initialize()
{

}

/** @brief Execute the process */
void FrameScheduling::execute()
{
	if (_current != NULL)
	{
		uint8_t idxProc;
		for (idxProc=0 ; idxProc<_current->nb ; idxProc++)
		{
			_current->processes[idxProc]->execute();
		}
		if (_current != NULL)
		{
			_current = _current->next;
		}
		else
		{
			_current = _first;
		}
	}
}

} /* namespace infra */
