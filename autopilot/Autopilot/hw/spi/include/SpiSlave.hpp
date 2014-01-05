/*
 * SpiSlave.hpp
 *
 *  Created on: 25 mars 2013
 *      Author: Aberzen
 */

#ifndef SPISLAVE_HPP_
#define SPISLAVE_HPP_

#include "SpiBus.hpp"

namespace hw {

class SpiSlave {
public:
	SpiSlave(SpiBus& bus, uint8_t slaveId);
	virtual ~SpiSlave();

	/** @brief Select the bus to access the slave. */
	bool select(portTickType ticks);

	/** @brief Release the bus. */
	void release();

	/** @brief Write / transfer any size. */
	bool read(uint8_t addr, uint8_t cnt, uint8_t* dataOut);

	/** @brief Write / transfer any size. */
	bool read(uint8_t addr, uint8_t cnt, uint16_t* dataOut);

	/** @brief Write / transfer any size. */
	bool transfer(uint8_t* dataIn, uint8_t cnt, uint8_t* dataOut = NULL);

	/** @brief Read 8bits. */
	bool transfer0(uint8_t dataIn);

	/** @brief Read 8bits. */
	bool transfer8(uint8_t addr, uint8_t& dataOut, bool discardFirst=true);

	/** @brief Read 16bits. */
	bool transfer16(uint8_t addr, uint16_t& dataOut, bool discardFirst=true);

	/** @brief Read 24bits. */
	bool transfer24(uint8_t addr, uint32_t& dataOut, bool discardFirst=true);

	/** @brief Read 32bits. */
	bool transfer32(uint8_t addr, uint32_t& dataOut, bool discardFirst=true);

protected:
	/** @brief ID of this slave. */
	uint8_t _id;

	/** @brief Is the current slave selected. */
	bool _isSelected;

	/** @brief SPI bus to which the current slave is attached. */
	SpiBus& _bus;
};

} /* namespace hw */
#endif /* SPISLAVE_HPP_ */
