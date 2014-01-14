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
#include <system/params/include/Nrd.hpp>


FastSerialPort0(Serial);        // FTDI/console
//FastSerialPort1(Serial1);       // GPS port
//FastSerialPort3(Serial3);       // Telemetry port



namespace board {

Apm25::Apm25(
		/* Inputs */
		/* Outputs */
		/* Param */
		Param& param
) :
		Board(),
		_spiBus(),
		_i2cBus(),
		_spiSlaveImu(_spiBus, IMU_MPU6000_SPI_CS_PIN),
		_spiSlaveBaro(_spiBus, BARO_MS5611_SPI_CS_PIN),
		_imu(_spiSlaveImu, meas.imu, rawMeas.imu, param.imu),
		_baro(_spiSlaveBaro, meas.baro),
		_compass(_i2cBus, meas.compass, rawMeas.compass),
		_gps(meas.gps),
		_pwm(pwmVal,radio)
//		_sensImu((hw::HalImu&)_imu, cnfImuRateMat_UB, cnfImuRateBias_B, cnfImuAccMat_UB, cnfImuAccBias_B),
//		_sensBaro(_baro),
//		_sensMagnetometer(_compass, cnfMagMat_UB, cnfMagBias_B),
//		_sensGps(),
//		_sensSonar()
{
}

Apm25::~Apm25() {
}

/** @brief Init the process */
infra::status Apm25::initialize()
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
	_imu.initialize();

	/* Init Baro Hal */
	_baro.initialize();

	/* Set clock diviser */
    _spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Execute Mag Hal */
	infra::status st = _compass.initialize();
//	if (st != 0)
//	{
//		Serial.printf("_compass.initialize() returns %d\n",st);
//	}

//	/* IMU sensor */
//	_sensImu.initialize();
//
//	/* Baro sensor */
//	_sensBaro.initialize();
//
//	/* Magnetometer sensor */
//	_sensMagnetometer.initialize();
//
//	/* GPS sensor */
//	_sensGps.initialize();
//
//	/* Sonar sensor */
//	_sensSonar.initialize();

	/* Pwm */
	_pwm.initialize();

	return 0;
}

/** @brief Execute the process */
infra::status Apm25::execute()
{
	/* Execute IMU Hal */
	_imu.execute();

	/* Execute Baro Hal */
	_baro.execute();

	/* Execute Mag Hal */
	infra::status st = _compass.execute();
//	if (st != 0)
//	{
//		Serial.printf("_compass.execute() returns %d\n",st);
//	}

//	/* Execute IMU Sensor */
//	_sensImu.execute();
//
//	/* Execute Baro Sensor */
//	_sensBaro.execute();
//
//	/* Execute Magnetometer Sensor */
//	_sensMagnetometer.execute();
//
//	/* Execute GPS Sensor */
//	_sensGps.execute();
//
//	/* Execute Sonar Sensor */
//	_sensSonar.execute();

	/* Pwm */
	_pwm.execute();

	return 0;
}


} /* namespace board */
