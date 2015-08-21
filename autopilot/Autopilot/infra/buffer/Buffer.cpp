/*
 * Buffer.cpp
 *
 *  Created on: 7 oct. 2014
 *      Author: Aberzen
 */

#include "Buffer.hpp"
#include <infra/rtos/Task.hpp>


namespace infra {

Buffer::Buffer(uint8_t *buffer, size_t len)
: _buffer(buffer),
  _mask(1),
  _idxHead(0),
  _idxTail(0)
{
	if (len == 0)
	{
		_mask = 0;
	}
	else if (len == 0xFFFF)
	{
		_mask = 0xFFFF;
	}
	/* Find the greatest 2^n smaller or equal to the len */
	while(_mask<=len)
	{
		_mask <<= 1;
	}
	_mask >>= 1;
	_mask -- ;
}

Buffer::~Buffer() {
}


/** @brief Read a buffer */
size_t Buffer::read(uint8_t *value, size_t len) {
	size_t nRead = 0;

	/* Bound elements to available elements */
	size_t avail = available();
	if (len > avail)
		len = avail;

	/* Copy data */
	for (nRead = 0 ; nRead < len ; nRead ++)
	{
		value[nRead] = _buffer[_idxTail];
		infra::Task::disableInterrupt();
		_idxTail = (_idxTail+1) & _mask;
		infra::Task::enableInterrupt();
	}

	return nRead;
}

/** @brief Read one char */
size_t Buffer::read(uint8_t *value) {
	size_t nRead = 0;
	if (_idxHead != _idxTail)
	{
		/* Copy data */
		*value = _buffer[_idxTail];
		infra::Task::disableInterrupt();
		_idxTail = (_idxTail+1) & _mask;
		infra::Task::enableInterrupt();
		nRead = 1;
	}
	return nRead;
}

/** @brief Write a buffer */
size_t Buffer::write(const uint8_t *value, size_t len) {
	size_t iWrite = 0;
	size_t nWrite = len;

	/* Bound elements to available elements */
	size_t free = Buffer::freeSpace();
	if (len > free)
		nWrite = free;

	/* Copy data */
	for (iWrite = 0 ; iWrite < nWrite ; iWrite ++)
	{
		_buffer[_idxHead] = value[iWrite];
		infra::Task::disableInterrupt();
		_idxHead = (_idxHead+1) & _mask;
		infra::Task::enableInterrupt();
	}

	return nWrite;
}

/** @brief Write one char */
size_t Buffer::write(uint8_t value) {
	size_t nWrite = 0;
	uint16_t idxHeadNext = (_idxHead+1) & _mask;

	if (idxHeadNext != _idxTail)
	{
		/* Copy data */
		_buffer[_idxHead] = value;
		_idxHead = idxHeadNext;
		nWrite = 1;
	}
	return nWrite;
}
/** @brief Get number of elements in the buffer */
size_t Buffer::available() {
	return (_idxHead - _idxTail) & _mask;
}


/** @brief Get number of free space in the buffer */
size_t Buffer::freeSpace() {
	return (_idxTail - 1 - _idxHead) & _mask;
}


/** @brief Discard bytes */
size_t Buffer::discard(size_t len) {
	/* Bound elements to available elements */
	size_t avail = available();
	if (len > avail)
		len = avail;

	/* Remove elements */
	_idxTail = (_idxTail + len) & _mask ;

	return len;
}


/** @brief Reset the buffer (free all space) */
void Buffer::reset() {
	_idxHead = 0;
	_idxTail = 0;
}


} /* namespace infra */
