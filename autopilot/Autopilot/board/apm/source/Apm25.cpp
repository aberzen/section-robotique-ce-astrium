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
		Param& param)
: Board(_imu, _baro, _compass, _gps, _pwm),
  _i2cBus(),
  _spiBus(),
  _spiSlaveImu(_spiBus, IMU_MPU6000_SPI_CS_PIN),
  _spiSlaveBaro(_spiBus, BARO_MS5611_SPI_CS_PIN),
  _imu(_spiSlaveImu, meas.imu, rawMeas.imu, param.imu),
  _compass(_i2cBus, meas.compass, rawMeas.compass),
  _baro(_spiSlaveBaro, meas.baro),
  _gps(meas.gps),
  _pwm(pwmVal,radio)
{
}

Apm25::~Apm25() {
}

#define USB_MUX_PIN 23
/** @brief Init the process */
void Apm25::initialize()
{
	/* Initialize Serial */
//	Serial.begin(115200);

    // on the APM2 board we have a mux thet switches UART0 between
    // USB and the board header. If the right ArduPPM firmware is
    // installed we can detect if USB is connected using the
    // USB_MUX_PIN
    pinMode(USB_MUX_PIN, INPUT);

    bool usb_connected = !digitalRead(USB_MUX_PIN);

	Serial.begin(57600);

	//Serial.printf("usb_connected = %d\n", usb_connected);

	/* Disable magnetometer */
    pinMode(63, OUTPUT);
    digitalWrite(63, HIGH);

    /* Initialize SPI */
    _spiBus.init();

    /* Set clock diviser */
    _spiBus.setClockDivider(SPI_CLOCK_DIV32); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	/* Init IMU Hal */
	_imu.initialize();

	/* Init Baro Hal */
	_baro.initialize();

	/* Set clock diviser */
    _spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Execute Mag Hal */
	_compass.initialize();

	/* Pwm */
	_pwm.initialize();
}


} /* namespace board */
