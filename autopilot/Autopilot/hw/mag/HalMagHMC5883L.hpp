/*
 * HalMagHMC5883L.hpp
 *
 *  Created on: 6 août 2013
 *      Author: Aberzen
 */

#ifndef HALMAGHMC5883L_HPP_
#define HALMAGHMC5883L_HPP_

#include "HalMagnetometer.hpp"
#include <hw/bus/I2C.hpp>

#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03

#define HALMAGHMC5883L_LSB_0_88GA  (0.73)
#define HALMAGHMC5883L_LSB_1_30GA  (0.92)
#define HALMAGHMC5883L_LSB_1_90GA  (1.22)
#define HALMAGHMC5883L_LSB_2_50GA  (1.52)
#define HALMAGHMC5883L_LSB_4_00GA  (2.27)
#define HALMAGHMC5883L_LSB_4_70GA  (2.56 )
#define HALMAGHMC5883L_LSB_5_60GA  (3.03)
#define HALMAGHMC5883L_LSB_8_10GA  (4.35)

namespace hw {


class HalMagHMC5883L : public HalMagnetometer {
public:
	HalMagHMC5883L(I2C& bus);
	virtual ~HalMagHMC5883L();

	/** @brief Initialize the HW */
	virtual bool initialize();

	/** @brief Reset the HW */
	virtual void reset();

	/** @brief Execute the driver */
	virtual bool sample (
			math::Vector3i& compassMeas_U);

protected:
	/** @brief Read register value */
	bool read_register(uint8_t address, uint8_t* value) const ;

	/** @brief Write a value in register */
	bool write_register(uint8_t address, uint8_t value) const ;

	/** @brief Program a measurement */
	int8_t commandSample();

	/** @brief Read raw measurement */
	int8_t readRaw(math::Vector3i& compassMeas_U);

protected:
	/** @brief Bus I2C which the HMC5883L is connected to */
	I2C& _bus;
};

} /* namespace hw */
#endif /* HALMAGHMC5883L_HPP_ */
