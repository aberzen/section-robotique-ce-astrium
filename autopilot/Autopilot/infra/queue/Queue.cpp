/*
 * Queue.cpp
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Queue.hpp"

#include <stddef.h>

namespace infra {


template<typename T>
Queue<T>::~Queue(){
}

template<typename T>
Queue<T>::Queue(List *pool) :
	_list(),
	_pool(pool)
{
}

template<typename T>
QueueStatus Queue<T>::enqueue(T* data)
{
	ListNode *node;
	QueueStatus result = E_QUEUE_OK;
	ListStatus status;

	if (data == NULL)
	{
		result = E_QUEUE_BAD_ARG;
	}
	else if (_list.isFull())
	{
		/* Should not happen in case a unique pool is shared among several queue */
		result = E_QUEUE_FULL;
	}
	else
	{
		/* Get a node from pool */
		status = _pool->pop(&node);

		/* Check status */
		switch (status)
		{
		case E_LIST_OK:

			/* Attach data */
			node->set(data);

			/* Push node to list */
			status = _list.pushBack(node);

			switch (status)
			{
			case E_LIST_OK:
				/* Nothing to do */
				break;
			case E_LIST_FULL:
				/* Should not happen in case a unique pool is shared among several queue */
				result = E_QUEUE_FULL;
				break;
			case E_LIST_EMPTY:
			case E_LIST_BAD_ARG:
			default:
				result = E_QUEUE_UNEXPECTED;
				break;
			}
			break;
		case E_LIST_EMPTY:
			/* Pool is empty */
			result = E_QUEUE_POOL_EMPTY;
			break;
		case E_LIST_BAD_ARG:
		case E_LIST_FULL:
		default:
			result = E_QUEUE_UNEXPECTED;
			break;
		}
	}

	/* Return status */
	return result;
}

template<typename T>
QueueStatus Queue<T>::dequeue(T** data)
{
	ListNode *node;
	QueueStatus result = E_QUEUE_OK;
	ListStatus status;

	if (data == NULL)
	{
		result = E_QUEUE_BAD_ARG;
	}
	else if (_pool->isFull())
	{
		/* Should not happen in case a unique pool is shared among several queue */
		result = E_QUEUE_POOL_FULL;
	}
	else
	{
		/* Get a node from list */
		status = _list.pop(&node);

		/* Check status */
		switch (status)
		{
		case E_LIST_OK:

			/* Detach data */
			*data = node->get();

			/* Push back node to pool */
			status = _pool->pushBack(node);

			switch (status)
			{
			case E_LIST_OK:
				/* Nothing to do */
				break;
			case E_LIST_FULL:
				/* Should not happen in case a unique pool is shared among several queue */
				result = E_QUEUE_POOL_FULL;
				break;
			case E_LIST_EMPTY:
			case E_LIST_BAD_ARG:
			default:
				result = E_QUEUE_UNEXPECTED;
				break;
			}
			break;
		case E_LIST_EMPTY:
			/* Queue is empty */
			result = E_QUEUE_EMPTY;
			break;
		case E_LIST_BAD_ARG:
		case E_LIST_FULL:
		default:
			result = E_QUEUE_UNEXPECTED;
			break;
		}
	}

	/* Return status */
	return result;
}

} /* namespace arducopter */
