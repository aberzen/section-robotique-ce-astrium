/*
 * Apm25.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include <Arduino.h>

#include <infra/include/Task.hpp>
#include "../include/Apm25.hpp"
#include <config/include/nrd.h>


FastSerialPort0(Serial);        // FTDI/console
//FastSerialPort1(Serial1);       // GPS port
//FastSerialPort3(Serial3);       // Telemetry port



namespace board {

Apm25::Apm25() :
		Board(),
		_spiBus(),
		_i2cBus(),
		_spiSlaveImu(_spiBus, IMU_MPU6000_SPI_CS_PIN),
		_spiSlaveBaro(_spiBus, BARO_MS5611_SPI_CS_PIN),
		_halImu(_spiSlaveImu, IMU_MPU6000_GYR_CNF, IMU_MPU6000_ACC_CNF, IMU_MPU6000_FREQ_CNF),
		_halBaro(_spiSlaveBaro),
		_halMag(_i2cBus),
		_sensImu((hw::HalImu&)_halImu, cnfImuRateMat_UB, cnfImuRateBias_B, cnfImuAccMat_UB, cnfImuAccBias_B),
		_sensBaro(_halBaro),
		_sensMagnetometer(_halMag, cnfMagMat_UB, cnfMagBias_B),
		_sensGps(),
		_sensSonar()
{
}

Apm25::~Apm25() {
}

/** @brief Init the process */
status Apm25::initialize()
{
	/* Initialize Serial */
	Serial.begin(115200);

	/* Disable magnetometer */
    pinMode(63, OUTPUT);
    digitalWrite(63, HIGH);

    /* Initialize SPI */
    _spiBus.init();

    /* Set clock diviser */
    _spiBus.setClockDivider(SPI_CLOCK_DIV32); // 2MHZ SPI rate
//	SPI.setDataMode(3);

    // we need to stop the barometer from holding the SPI bus
//    pinMode(40, OUTPUT);
//    digitalWrite(40, HIGH);

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	/* Init IMU Hal */
	_halImu.initialize();

	/* Init Baro Hal */
	_halBaro.initialize();

	/* Set clock diviser */
    _spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Execute Mag Hal */
	status st = _halMag.initialize();
//	if (st != 0)
//	{
//		Serial.printf("_halMag.initialize() returns %d\n",st);
//	}

	/* IMU sensor */
	_sensImu.initialize();

	/* Baro sensor */
	_sensBaro.initialize();

	/* Magnetometer sensor */
	_sensMagnetometer.initialize();

	/* GPS sensor */
	_sensGps.initialize();

	/* Sonar sensor */
	_sensSonar.initialize();

	return 0;
}

/** @brief Execute the process */
status Apm25::execute()
{
	/* Execute IMU Hal */
	_halImu.execute();

	/* Execute Baro Hal */
	_halBaro.execute();

	/* Execute Mag Hal */
	status st = _halMag.execute();
//	if (st != 0)
//	{
//		Serial.printf("_halMag.execute() returns %d\n",st);
//	}

	/* Execute IMU Sensor */
	_sensImu.execute();

	/* Execute Baro Sensor */
	_sensBaro.execute();

	/* Execute Magnetometer Sensor */
	_sensMagnetometer.execute();

	/* Execute GPS Sensor */
	_sensGps.execute();

	/* Execute Sonar Sensor */
	_sensSonar.execute();

	return 0;
}


} /* namespace board */
