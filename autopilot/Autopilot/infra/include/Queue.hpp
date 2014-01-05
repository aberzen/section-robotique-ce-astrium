/*
 * Queue.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include "List.hpp"

namespace infra {

typedef enum EQueueStatus {
	E_QUEUE_OK = 0,
	E_QUEUE_FULL,
	E_QUEUE_EMPTY,
	E_QUEUE_POOL_FULL,
	E_QUEUE_POOL_EMPTY,
	E_QUEUE_UNEXPECTED,
	E_QUEUE_BAD_ARG
} QueueStatus;

template<typename T>
class Queue {
public:
	virtual ~Queue() ;
	Queue(List *pool);

	virtual QueueStatus enqueue(T* data);
	virtual QueueStatus dequeue(T** data);

protected:
	List _list;
	List *_pool;
};

} /* namespace arducopter */
#endif /* QUEUE_H_ */
