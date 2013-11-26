/*
 * MemoryPool.cpp
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#include "../include/MemoryPool.hpp"

namespace infra {

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
MemoryPool<nbBuffers, bufferSizeInBytes>::MemoryPool() :
	_pool(_nodeList,nbBuffers),
	_queue(_pool)
{
}

template<uint16_t nbBuffers, uint16_t bufferSizeInBytes>
MemoryPool<nbBuffers, bufferSizeInBytes>::~MemoryPool() {
}

} /* namespace arducopter */
