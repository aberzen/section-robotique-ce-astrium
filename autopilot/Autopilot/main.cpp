/*
 * main.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */


#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <hw/serial/include/FastSerial.hpp>

#include <stdint.h>

#include <Arduino.h>

#include <board/apm/include/Apm25.hpp>
#include <system/system/include/System.hpp>

board::Apm25::Param param = {
	{ /* imu */
		hw::HalImuMpu6000::E_GYR_CNF_500DPS, /* gyrCnf */
		hw::HalImuMpu6000::E_ACC_CNF_4G, /* accCnf */
		hw::HalImuMpu6000::E_UPT_FREQ_100HZ /* frequence */
	} /* imu */
};
static board::Apm25 boardApm25(param);
board::Board& board::Board::board = boardApm25;

static system::System sys(boardApm25);
system::System& system::System::system = sys;

//board::Board& board::Board::board = boardApm25;

#include <test/ControlCyclicTask.hpp>

//static struct test::Variables varList = {
//	154,
//	-98,
//	0xABCD,
//	-0xABCD,
//	0xCAFEDECA,
//	0xDEADC0FF,
//	1458.54365487
//};
//
//const mavlink::ParameterMgt::ParamInfo info[7] PROGMEM = {
//		{MAV_PARAM_TYPE_UINT8, "VAR1", &varList.var1, {UINT8 : 154}},
//		{MAV_PARAM_TYPE_INT8, "VAR2", &varList.var2, {INT8 : -98}},
//		{MAV_PARAM_TYPE_UINT16, "VAR3", &varList.var3, {UINT16 : 0xABCD}},
//		{MAV_PARAM_TYPE_INT16, "VAR4", &varList.var4, {INT16 : -0xABCD}},
//		{MAV_PARAM_TYPE_UINT32, "VAR5", &varList.var5, {UINT32 : 0xCAFEDECA}},
//		{MAV_PARAM_TYPE_INT32, "VAR6", &varList.var6, {INT32 : 0xDEADC0FF}},
//		{MAV_PARAM_TYPE_REAL32, "VAR7", &varList.var7, {REAL32 : 1458.54365487}}
//};

#include <test/MavTask.hpp>
#include <gcs/include/SerialChannel.hpp>

mavlink_system_t mavlink_system = {7, 1, 0, 0};
mavlink::SerialChannel mavChan0(MAVLINK_COMM_0, Serial);

test::MavTask task(
		(const signed char*)"MAVL",
		(unsigned portSHORT)720,
		(unsigned portBASE_TYPE)1
);

test::ControlCyclicTask taskControl(
		(const signed char*)"CTRL",
		(unsigned portSHORT)720,
		(unsigned portBASE_TYPE)2,
        10,
        1);


int main(int argc, char **argv)
{
	/* Initialize Arduino library */
	arduino_init();

	/* End by starting scheduler */
	vTaskStartScheduler();

}

