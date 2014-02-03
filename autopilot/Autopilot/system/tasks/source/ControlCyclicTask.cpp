/*
 * ControlCyclicTask.cpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#include "../include/ControlCyclicTask.hpp"
#include <board/gen/include/Board.hpp>
#include <system/system/include/System.hpp>
#include <autom/mgt/include/Ancs.hpp>

#include <hw/serial/include/FastSerial.hpp>

#include <Arduino.h>

#include "../include/MavTask.hpp"

namespace test {


ControlCyclicTask::ControlCyclicTask(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority,
        uint16_t period,
        uint16_t delay) :
	infra::CyclicTask(pcName,usStackDepth,uxPriority,period,delay)
{
}

ControlCyclicTask::~ControlCyclicTask() {
}

/** @brief Non returning function executed as the task body function. */
void ControlCyclicTask::init(void)
{

	/* Execute Sensors */
	system::System::system.initialize();

	Task::init();
}

uint32_t saved_dur_wcet;
//#define COUNT_MAX 0x0000FFFF
void ControlCyclicTask::runCycle(void)
{
	uint32_t now = micros();

	if (_missed != 0)
	{
		Serial.printf("ControlCyclicTask::runCycle _missed = %d\n",_missed);
		_missed = 0;
	}

//	/* Process HF sensors */
//	board::Board::board.getPwm().execute();
//	board::Board::board.getImu().execute();
//	board::Board::board.getBaro().execute();
//	board::Board::board.getCompass().execute();
//
//	/* Execute HF functions */
//	system::System::system._mgt._est.execute();
//	system::System::system._mgt._attCtrl.execute();
//	system::System::system._mgt._mod.execute();

	system::System::system.ancs.execute();
	system::System::system.getMavHouseKeeping().update();

	uint32_t dur = micros() - now;
	if (dur > saved_dur_wcet)
	{
		saved_dur_wcet = dur;
//		Serial.printf("WCET = %ld us\n", saved_dur_wcet);
	}
//	/* Execute Sensors */
//	  int32_t time_start, time_stop, duration;
//	  uint32_t count;
//	  volatile float f1 = random()*2^15, f2 = random()*2^10, f3;
//	  volatile int32_t i1 = (int32_t)f1, i2 = (int32_t)f2, i3;
//
//	  Serial.println("*************************");
//	  Serial.println("** TEST 1 (float)");
//	  Serial.println("*************************");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    f3 = f1 * f2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Mult Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    f3 = f1 / f2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Div Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    f3 = f1 + f2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Add Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    f3 = f1 - f2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Sub Duration = ");Serial.print(duration);Serial.println("mico sec");
//
//	  Serial.println("*************************");
//	  Serial.println("** TEST 2 (int)");
//	  Serial.println("*************************");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    i3 = i1 * i2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Mult Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    i3 = i1 / i2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Div Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    i3 = i1 + i2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Add Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//	    i3 = i1 - i2;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Sub Duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//		i3 = i1 >> 10;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Decalage a droite duration = ");Serial.print(duration);Serial.println("mico sec");
//  	  count = COUNT_MAX;
//	  time_start = micros();
//	  while(count-- != 0)
//	  {
//		i3 = i1 << 10;
//	  }
//	  time_stop = micros();
//	  duration = time_stop - time_start;
//	  Serial.print("Decalage a gauche duration = ");Serial.print(duration);Serial.println("mico sec");
}


} /* namespace test */
