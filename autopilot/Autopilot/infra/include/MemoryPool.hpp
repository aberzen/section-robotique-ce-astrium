/*
 * MemoryPool.h
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#ifndef MEMORYPOOL_H_
#define MEMORYPOOL_H_

#include "../include/Queue.hpp"
#include "../include/List.hpp"

#include <stdint.h>

namespace infra {

typedef enum EMemoryPoolStatus {
	E_MEMORYPOOL_STATUS_OK=E_QUEUE_OK,
	E_MEMORYPOOL_STATUS_EMPTY=E_QUEUE_EMPTY,
	E_MEMORYPOOL_STATUS_FULL=E_QUEUE_FULL,
	E_MEMORYPOOL_STATUS_BAD_ARG=E_QUEUE_BAD_ARG
} MemoryPoolStatus;

/**
 * @brief Memory Pool
 * Use this pool to avoid memory fragmentation.
 */
template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
class MemoryPool {
public:
	MemoryPool();
	virtual ~MemoryPool() ;

	inline uint16_t getBufferSize(void);
	inline uint16_t getTotBufferCount(void);
	inline uint16_t getFreeBufferCount(void);

	inline MemoryPoolStatus allocBuffer(void** buffer);
	inline MemoryPoolStatus freeBuffer(void* buffer);

protected:
	/** @brief Start address of the allocated buffers */
	const uint32_t _mem[nbBuffers*((bufferSizeInBytes+3)/4)];

	/** @brief Node used to store the buffers in the list */
	ListNode _nodeList[nbBuffers];

	/** @brief Pool of nodes */
	List _pool;

	/** @brief Queue of available buffers */
	Queue<uint8_t*> _queue;

};

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
inline uint16_t MemoryPool<nbBuffers, bufferSizeInBytes>::getBufferSize(void){
	return bufferSizeInBytes;
}

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
inline uint16_t MemoryPool<nbBuffers, bufferSizeInBytes>::getTotBufferCount(void){
	return nbBuffers;
}

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
inline uint16_t MemoryPool<nbBuffers, bufferSizeInBytes>::getFreeBufferCount(void){
	return _pool.getCount();
}

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
inline MemoryPoolStatus MemoryPool<nbBuffers, bufferSizeInBytes>::allocBuffer(void** buffer){
	return (MemoryPoolStatus) this->_queue.dequeue(buffer);
}

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
inline MemoryPoolStatus MemoryPool<nbBuffers, bufferSizeInBytes>::freeBuffer(void* buffer){
	return (MemoryPoolStatus) this->_queue.enqueue(buffer);
}

} /* namespace arducopter */
#endif /* MEMORYPOOL_H_ */
