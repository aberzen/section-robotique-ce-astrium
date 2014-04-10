/*
 * ControlCyclicTask.cpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include <system/system/include/System.hpp>
#include <hw/Screen/include/Screen.hpp>

#include "../include/ControlCyclicTask.hpp"

namespace test {

ControlCyclicTask::ControlCyclicTask(
		const signed char * pcName,
        unsigned portSHORT usStackDepth,
        unsigned portBASE_TYPE uxPriority,
        uint16_t period,
        uint16_t delay) :
	infra::CyclicTask(pcName,usStackDepth,uxPriority,period,delay),myScreen(system::System::system.board.pwmVal,system::System::system.board.pwm)
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
	nbcycle = 0;//screen demo
	nbcycle1 = 0;
	myScreen.Display(hw::S_ALL_OFF);
}

void ControlCyclicTask::runCycle(void)
{
	if (_missed != 0)
	{
		Serial.printf("ControlCyclicTask::runCycle _missed = %d\n",_missed);
		_missed = 0;
	}

	system::System::system.ancs.smGlobal.execute();
	system::System::system.getMavHouseKeeping().update();
	nbcycle = nbcycle +1;
	switch (nbcycle1){
	case 0:
		myScreen.Display(hw::S_ALL_ON);
		break;
	case 1:
		myScreen.Display(hw::S_CHENILLE_LF);
		break;
	case 2:
		myScreen.Display(hw::S_CHENILLE_RS);
		break;
	case 3:
		myScreen.Display(hw::S_CHENILLE_RF);
		break;
	case 4:
		myScreen.Display(hw::S_CHENILLE_LS);
		break;
	case 5:
		myScreen.Display(hw::S_BLUE);
		break;
	case 6:
		myScreen.Display(hw::S_YELLOW);
		break;
	case 7:
		myScreen.Display(hw::S_RED);
		break;
	case 8:
		myScreen.Display(hw::S_BLUE_BS);
		break;
	case 9:
		myScreen.Display(hw::S_BLUE_BF);
		break;
	case 10:
		myScreen.Display(hw::S_BUZZ);
		break;
	default:
		nbcycle1=0;
		break;
	}
	if (nbcycle % 500==0) nbcycle1 = nbcycle1+1;
		// execute myScreen to change screen state
		myScreen.update();}


} /* namespace test */
