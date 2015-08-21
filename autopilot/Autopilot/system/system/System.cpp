/*
 * System.cpp
 *
 *  Created on: 23 mai 2015
 *      Author: Modélisme
 */


#include <gcs/channel/mavlink_bridge.hpp>

#include "System.hpp"

#include <infra/rtos/Task.hpp>

#define PARAM(type, name, addr, defVal) { MAV_PARAM_TYPE_ ## type, name, addr, {type : defVal} }
#define USB_MUX_PIN 23


namespace system {

const autom::Modulator::InfluenceMatrix paramModInflMat = {
	 {   0,       0,       0,       0,     },
	 {   0,       0,       0,       0,     },
	 {   6000,    6000,    6000,    6000,  },
	 {   2546,  - 2546,    2546,  - 2546,  },
	 { - 2546,    2546,    2546,  - 2546,  },
	 {   600,     600,   - 600,   - 600,   }
};

#ifdef MODULATORPINV
autom::ModulatorPinv::DescentVector paramModModDesc = {};
autom::ModulatorPinv::PInvInflMat paramModModPInv = {};
#else
const autom::ModulatorLut::LutVectorFrc paramModLutFrc = {
	    {{-0,      -0,      -0,      -0     },
	    {  0,       0,       0,       0     }},
	    {{-0,      -0,      -0,      -0     },
	    {  0,       0,       0,       0     }},
	    {{-0,      -0,      -0,      -0     },
	    {  44,      44,      44,      44    }}
};

const autom::ModulatorLut::LutVectorTrq paramModLutTrq = {
	    {{-0,      -206,    -0,      -206   },
	    {  206,     0,       206,     0     }},
	    {{-206,    -0,      -0,      -206   },
	    {  0,       206,     206,     0     }},
	    {{-0,      -0,      -874,    -874   },
	    {  874,     874,     0,       0     }}
};
//const uint8_t paramModScaleRequest = 10;
//const uint8_t paramModScaleInflMat = 10;
const math::Vector3l paramModFrcMaxPos_B( 0L, 0L, 28125L);
const math::Vector3l paramModFrcMaxNeg_B( 0L, 0L, 0L);
const math::Vector3l paramModTrqMaxPos_B( 5967L, 5967L, 1406L);
const math::Vector3l paramModTrqMaxNeg_B(-5967L,-5967L,-1406L);
#endif

math::Vector3l attCtrlKp(0,0,0);
math::Vector3l attCtrlKd(0,0,0);
math::Vector3l attCtrlKi(0,0,0);
math::Vector3l attCtrlIntMax(0,0,0);

/** @brief Handlers for COM0 interrupts */
SerialHandler(0);


System::System()
: dataPool(),
  _buffRx0(&_buffRx0_buffer[0], SERIAL0_RX_LEN),
  _buffTx0(&_buffTx0_buffer[0], SERIAL0_TX_LEN),
  _spiBus(),
  _i2c(),
  _imu(
		  _spiBus,
		  hw::HalImuMpu6000::E_GYR_CNF_1000DPS,
		  hw::HalImuMpu6000::E_ACC_CNF_4G,
		  hw::HalImuMpu6000::E_UPT_FREQ_500HZ),
  _baro(_spiBus),
  _compass(_i2c),
  SerialPortConstParam(_com0, 0, _buffRx0, _buffTx0),
  _pwm(),
  _paramMgt(info, paramCount),
  _mavChannel(MAVLINK_COMM_0,_com0),
  _gcs(MAVLINK_COMM_0),
  _estimator(),
#ifdef MODULATORPINV
#else
  _modulator(
			paramModInflMat,
			paramModFrcMaxPos_B,
			paramModFrcMaxNeg_B,
			paramModTrqMaxPos_B,
			paramModTrqMaxNeg_B,
			paramModLutFrc,
			paramModLutTrq),
#endif
  _ctrl(attCtrlKp, attCtrlKd, attCtrlKi, attCtrlIntMax),
  _motorArmed(false),
  _navAuto(false)
  /*_radio()*/
{

}

System::~System()
{

}

void System::initialize()
{
	_paramMgt.loadAllValues();

    /* Disable magnetometer */
	pinMode(63, OUTPUT);
	digitalWrite(63, HIGH);

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_spiBus.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_i2c.begin();
	_i2c.setSpeed(false);
	// on the APM2 board we have a mux thet switches UART0 between
	// USB and the board header. If the right ArduPPM firmware is
	// installed we can detect if USB is connected using the
	// USB_MUX_PIN
	pinMode(USB_MUX_PIN, INPUT);

	bool usb_connected = !digitalRead(USB_MUX_PIN);

	if (usb_connected)
		_com0.open(115200);
	else
		_com0.open(57600);

	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV32); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_imu.initialize();
	_baro.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	_compass.initialize();

	/* pwm */
	_pwm.initialize();
	for (uint8_t idx=0 ; idx<CNF_NB_MOTORS ; idx++)
	{
		_pwm.enable_out(idx);
	}

	/* Estimator */
	_estimator.initialize();

	/* motors */
	_modulator.initialize();

	/* GCS */
	_gcs.initialize();
}

void System::execute()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Process radio */
	processRadio();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process guidance */
	processGuidance();

	/* Process Attitude controller */
	processAttitudeController();

	/* Process Navigation controller */
	processNavigationController();

	/* Process actuator */
	processActuators();

	/* Process GCS services */
	_gcs.processServices();
}

/** @brief Process sensors (raw) */
void System::processRawSensors()
{
	uint32_t now = micros();

	// Sample compass
	if (_compass.sample(dataPool.compassMagRaw_U))
	{
		dataPool.compassLastMeasDateUsec = now;
	}

	// Sample barometer
	if (_baro.sample(dataPool.baroPressRaw_U, dataPool.baroTempRaw_U))
	{
		dataPool.baroLastMeasDateUsec = now;
	}

	// Sample imu1
	if (_imu.sample(dataPool.imuRateRaw_U, dataPool.imuAccRaw_U,dataPool.imuTemp))
	{
		dataPool.imuLastMeasDateUsec = now;
	}
}

/** @brief Process RC (raw) */
void System::processRadio()
{
//	_pwm.read(&dataPool.pwm_inputs[0]);
}

/** @brief Post process sensors */
void System::postProcessSensors()
{
//	char buffer[100];

	/* IMU gyro */
	dataPool.imuRate_B(
			((float) dataPool.imuRateRaw_U.x) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.y) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.z) * GYRO_SCALE_1000DPS);

	/* IMU acco */
	dataPool.imuAcc_B(
			((float) dataPool.imuAccRaw_U.x) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.y) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.z) * ACC_SCALE_4G);

	/* Compass */
	dataPool.compassMag_B(
			((float) (dataPool.compassMagRaw_U.x - 61)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.y + 27)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.z -  3)) * HALMAGHMC5883L_LSB_1_30GA);
}

/** @brief Process estimation */
void System::processEstimation()
{
	_estimator.update();
}

/** @brief Process mode */
void System::processGuidance()
{

}

/** @brief Process actuators */
void System::processActuators()
{
	if (_motorArmed)
	{
		/* Compute motor inputs */
		_modulator.calcMotorCommand(
				dataPool.ctrlFrcDemB,
				dataPool.ctrlTrqDemB,
				&dataPool.pwm_outputs[0]);
	}
	else
	{
		for (uint8_t iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
			dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
	}

	/* write PWM outputs */
	_pwm.write(&dataPool.pwm_outputs[0]);

	/* Compute efforts produced by the modulator */
	_modulator.calcTorsor(
			&dataPool.pwm_outputs[0],
			dataPool.estForce_B,
			dataPool.estTorque_B);
}

/** @brief Process attitude controller */
void System::processAttitudeController()
{
	if (_motorArmed)
	{
		_ctrl.computeTorque(
				dataPool.guidAtt_IB,
				dataPool.estAtt_IB,
				dataPool.guidRate_B,
				dataPool.estRate_B);
	}
}

/** @brief Process navigation controller */
void System::processNavigationController()
{
	if (_motorArmed)
	{
	}
}

static int8_t test = 0;


PROGMEM const mavlink::ParameterMgt::ParamInfo info[10] = {
		PARAM(INT8,"TEST", &test, 10)
};
uint16_t paramCount = 1;


System system = System();


} /* namespace system */
