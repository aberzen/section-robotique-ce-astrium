/*
 * SpiSlave.cpp
 *
 *  Created on: 25 mars 2013
 *      Author: Aberzen
 */

#include "../include/SpiSlave.hpp"

namespace hw {

SpiSlave::SpiSlave(SpiBus& bus, uint8_t slaveId) :
	_id(slaveId),
	_isSelected(false),
	_bus(bus)
	{
	/* Set SS as OUTPUT */
	pinMode(_id, OUTPUT);
	/* Set SS to deselect the slave */
    digitalWrite(_id, HIGH);
}

SpiSlave::~SpiSlave() {
}

/** @brief Select the bus to access the slave. */
bool SpiSlave::select(portTickType ticks) {
	if (!_isSelected) {
		/* Reserve the mutex*/
		_isSelected = xSemaphoreTake(_bus._mutex,ticks);
		if (_isSelected)
		{
			/* Activate the device */
			digitalWrite(_id, LOW);
		}
	}
	return _isSelected;
}

/** @brief Release the bus. */
void SpiSlave::release() {
	if (_isSelected) {
		/* Release the mutex*/
		xSemaphoreGive(_bus._mutex);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
	    /* Now, slave is deselected */
		_isSelected = false;
	}
}


/** @brief Write / transfer any size. */
bool SpiSlave::read(uint8_t addr, uint8_t cnt, uint8_t* dataOut)
{
	bool result = false;
	uint8_t idx;
	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
		digitalWrite(_id, LOW);

		/* Discard */
		_bus.transfer(addr);
		for (idx=0 ; idx<cnt ; idx++)
		{
			/* Discard */
			*(dataOut++) = _bus.transfer(0);
		}
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		result = true;
	}
	return result;
}

/** @brief Write / transfer any size. */
bool SpiSlave::read(uint8_t addr, uint8_t cnt, uint16_t* dataOut)
{
	bool result = false;
	uint8_t idx;
	uint16_t dataL = 0, dataH = 0;
	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
		digitalWrite(_id, LOW);

		/* Discard */
		_bus.transfer(addr);
		for (idx=0 ; idx<cnt ; idx++)
		{
			dataH = _bus.transfer(0);
			dataL = _bus.transfer(0);
			*(dataOut++) = (dataH<<8) | dataL;
		}
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		result = true;
	}
	return result;
}


/** @brief Transfer arbitrary count. */
bool SpiSlave::transfer(uint8_t* dataIn, uint8_t cnt, uint8_t* dataOut)
{
	bool result = false;
	uint8_t idx;
	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
		digitalWrite(_id, LOW);

		if (dataOut == NULL)
		{
			for (idx=0 ; idx<cnt ; idx++)
				/* Discard */
				_bus.transfer(*(dataIn++));
		}
		else
		{
			for (idx=0 ; idx<cnt ; idx++)
				/* Discard */
				*(dataOut++) = _bus.transfer(*(dataIn++));
		}
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		result = true;
	}
	return result;
}

/** @brief Read 8bits. */
bool SpiSlave::transfer0(uint8_t cmd) {
	bool result = false;

	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
	    digitalWrite(_id, LOW);
	    /* Discard */
		_bus.transfer(cmd);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		result = true;
	}
	return result;
}

/** @brief Read 8bits. */
bool SpiSlave::transfer8(uint8_t cmd, uint8_t& dataOut, bool discardFirst) {
	bool result = false;

	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
	    digitalWrite(_id, LOW);
	    dataOut = _bus.transfer(cmd); /* Discard */
	    if (discardFirst)
	    	dataOut = _bus.transfer(0);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		result = true;
	}
	return result;
}

/** @brief Read 16bits. */
bool SpiSlave::transfer16(uint8_t cmd, uint16_t& dataOut, bool discardFirst) {
	bool result = false;
	uint16_t dataL, dataH;

	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
	    digitalWrite(_id, LOW);
	    dataH = _bus.transfer(cmd);
	    if (discardFirst)
			dataH = _bus.transfer(0);
		dataL = _bus.transfer(0);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		dataOut = (dataH<<8) | dataL;
		result = true;
	}
	return result;
}

/** @brief Transfer 24bits. */
bool SpiSlave::transfer24(uint8_t cmd, uint32_t& dataOut, bool discardFirst) {
	bool result = false;
	uint32_t dataL, dataM, dataH;

	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
	    digitalWrite(_id, LOW);
	    dataH = _bus.transfer(cmd);
	    if (discardFirst)
			dataH = _bus.transfer(0);
		dataM = _bus.transfer(0);
		dataL = _bus.transfer(0);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		dataOut = (dataH<<16) | (dataM<<8) | dataL;
		result = true;
	}
	return result;
}

/** @brief Transfer 32bits. */
bool SpiSlave::transfer32(uint8_t cmd, uint32_t& dataOut, bool discardFirst) {
	bool result = false;
	uint8_t dataL;
	uint16_t dataML;
	uint32_t dataMH, dataH;

	/* Check if current slave is selected */
	if (_isSelected)
	{
		/* Set SS to deselect the slave */
	    digitalWrite(_id, LOW);
	    dataH = _bus.transfer(cmd);
	    if (discardFirst)
			dataH = _bus.transfer(0);
		dataMH = _bus.transfer(0);
		dataML = _bus.transfer(0);
		dataL = _bus.transfer(0);
		/* Set SS to deselect the slave */
	    digitalWrite(_id, HIGH);
		dataOut = (dataH<<24) | (dataMH<<16) | (dataML<<8) | dataL;
		result = true;
	}
	return result;
}

} /* namespace hw */
