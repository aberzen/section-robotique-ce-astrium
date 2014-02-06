/*
 * Apm25.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef APM25_HPP_
#define APM25_HPP_

#include <board/gen/include/Board.hpp>

#include <hw/spi/include/SpiBus.hpp>
#include <hw/i2c/include/I2C.hpp>
#include <hw/imu/include/HalImuMpu6000.hpp>
#include <hw/baro/include/HalBarometerMs5611OverSpi.hpp>
#include <hw/gps/include/Gps.hpp>
#include <hw/mag/include/HalMagHMC5883L.hpp>
#include <hw/pwm/include/PwmApm25.hpp>

#include <autom/est/include/Estimator.hpp>

namespace board {

class Apm25 : public Board {
public:
	typedef struct {
		::hw::HalImuMpu6000::Param imu;
	} Param;
public:
	Apm25(
			/* Inputs */
			/* Outputs */
			/* Param */
			Param& param);
	virtual ~Apm25();

	/** @brief Init the process */
	virtual void initialize();

protected:
	/** @brief I2C Bus */
	hw::I2C _i2cBus;

	/** @brief Spi Bus */
	hw::SpiBus _spiBus;

	/** @brief Spi slave for Imu */
	hw::SpiSlave _spiSlaveImu;

	/** @brief Spi slave for Baro */
	hw::SpiSlave _spiSlaveBaro;

	/** @brief Hal for Imu */
	hw::HalImuMpu6000 _imu;

	/** @brief Hal for Imu */
	hw::HalMagHMC5883L _compass;

	/** @brief Hal for Barometer */
	hw::HalBarometerMs5611OverSpi _baro;

	/** @brief Sensor for Barometer */
	hw::Gps _gps;

	/** @brief Pwm */
	hw::PwmApm25 _pwm;
};

} /* namespace board */
#endif /* APM25_HPP_ */
