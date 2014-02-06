/*
 * HalMagHMC5883L.hpp
 *
 *  Created on: 6 août 2013
 *      Author: Aberzen
 */

#ifndef HALMAGHMC5883L_HPP_
#define HALMAGHMC5883L_HPP_

#include "HalMagnetometer.hpp"
#include <hw/i2c/include/I2C.hpp>

#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03

namespace hw {


class HalMagHMC5883L : public HalMagnetometer {
public:
	HalMagHMC5883L(
			/* Dependencies */
			I2C& bus,
			/* Outputs */
			HalMagnetometer::Output& out,
			HalMagnetometer::RawOutput& rawOut);
	virtual ~HalMagHMC5883L();

	/** @brief Initialize the HW */
	virtual void initialize();

	/** @brief Reset the HW */
	virtual void reset();

	/** @brief Execute the driver */
	virtual void execute();

protected:
	/** @brief Read register value */
	bool read_register(uint8_t address, uint8_t* value);

	/** @brief Write a value in register */
	bool write_register(uint8_t address, uint8_t value);

	/** @brief Program a measurement */
	infra::status commandSample();

	/** @brief Read raw measurement */
	infra::status readRaw();

	/** @brief Scale raw measurement into SI */
	void scaleRaw();

protected:
	/** @brief Bus I2C which the HMC5883L is connected to */
	I2C& _bus;
};

} /* namespace hw */
#endif /* HALMAGHMC5883L_HPP_ */
