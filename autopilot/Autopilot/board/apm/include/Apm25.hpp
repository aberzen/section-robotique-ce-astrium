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
#include <hw/gps/include/HalGps.hpp>
#include <hw/mag/include/HalMagHMC5883L.hpp>
#include <hw/sonar/include/HalSonar.hpp>

#include <sensor/baro/include/Ms5611Barometer.hpp>

namespace board {

class Apm25 : public Board {
public:
	Apm25();
	virtual ~Apm25();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

	/** @brief Get IMU HAL */
	virtual inline hw::HalImu& getHalImu();

	/** @brief Get IMU HAL */
	virtual inline hw::HalMagnetometer& getHalMagnetometer();

	/** @brief Get IMU sensor */
	virtual inline sensor::Imu& getImu();

	/** @brief Get Barometer sensor */
	virtual inline sensor::Barometer& getBarometer();

	/** @brief Get Gps sensor */
	virtual inline sensor::Gps& getGps();

	/** @brief Get Sonar sensor */
	virtual inline sensor::Sonar& getSonar();

	/** @brief Get Magnetometer sensor */
	virtual inline sensor::Magnetometer& getMagnetometer();

//	/** @brief Get Led1 sensor */
//	virtual inline sensor::Led& getLed1();

//protected:
	/** @brief Spi Bus */
	hw::SpiBus _spiBus;

	/** @brief I2C Bus */
	hw::I2C _i2cBus;
	/** @brief Spi slave for Imu */
	hw::SpiSlave _spiSlaveImu;

	/** @brief Spi slave for Baro */
	hw::SpiSlave _spiSlaveBaro;

	/** @brief Hal for Imu */
	hw::HalImuMpu6000 _halImu;

	/** @brief Hal for Imu */
	hw::HalMagHMC5883L _halMag;

	/** @brief Hal for Barometer */
	hw::HalBarometerMs5611OverSpi _halBaro;

	/** @brief Sensor for Imu */
	sensor::Imu _sensImu;

	/** @brief Sensor for Barometer */
	sensor::Ms5611Barometer _sensBaro;

	/** @brief Sensor for Barometer */
	sensor::Magnetometer _sensMagnetometer;

	/** @brief Sensor for Barometer */
	sensor::Gps _sensGps;

	/** @brief Sensor for Barometer */
	sensor::Sonar _sensSonar;

};

/** @brief Get IMU HAL */
inline hw::HalImu& Apm25::getHalImu()
{
	return _halImu;
}

/** @brief Get Mag HAL */
inline hw::HalMagnetometer& Apm25::getHalMagnetometer()
{
	return _halMag;
}

/** @brief Get IMU sensor */
inline sensor::Imu& Apm25::getImu()
{
	return _sensImu;
}

/** @brief Get Barometer sensor */
inline sensor::Barometer& Apm25::getBarometer()
{
	return _sensBaro;
}

/** @brief Get Gps sensor */
inline sensor::Gps& Apm25::getGps()
{
	return _sensGps;
}

/** @brief Get Sonar sensor */
inline sensor::Sonar& Apm25::getSonar()
{
	return _sensSonar;
}

/** @brief Get Magnetometer sensor */
inline sensor::Magnetometer& Apm25::getMagnetometer()
{
	return _sensMagnetometer;
}

} /* namespace board */
#endif /* APM25_HPP_ */
