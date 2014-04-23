/*
 * screen.cpp
 *
 *  Created on: 5 mars 2014
 *      Author: vincent
 */

#include <hw/Screen/include/Screen.hpp>

namespace hw {

Screen::Screen(hw::Pwm::Input& out,hw::Pwm& pwm) : _pwm(pwm),_out(out),myBlueLed(25),myYellowLed(26),myRedLed(27)
{
	ncycle=0;
	ncycle1=0;
	// initialisation du canal buzzer
	_out.channels[BUZZ_CHANNEL] = MIN_PULSEWIDTH;
	_pwm.force_out(BUZZ_CHANNEL);
	_pwm.enable_out(BUZZ_CHANNEL);

}

Screen::~Screen()
{}
void Screen::Display(ScreenState S)
{
	if(_state != S){
		// qu changement d'état il faut réinitialiser les cycles non en cours
		// on peut changer plus vite que le cycle
		ncycle=0;
		_out.channels[BUZZ_CHANNEL] = MIN_PULSEWIDTH;
		_pwm.force_out(BUZZ_CHANNEL);
		myBlueLed.switchOn();
		myRedLed.switchOn();
		myYellowLed.switchOn();
		_state = S;
	}

}
void Screen::update()
{
	ncycle = ncycle+1;
	switch (_state) {
	case S_BUZZ:
		if (ncycle % VERY_SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1==1){_out.channels[BUZZ_CHANNEL] = MAX_PULSEWIDTH;myYellowLed.switchOff();}
		if(ncycle1==2){_out.channels[BUZZ_CHANNEL] = MIN_PULSEWIDTH;myYellowLed.switchOn();}
		if(ncycle1==3){_out.channels[BUZZ_CHANNEL] = MAX_PULSEWIDTH;myYellowLed.switchOff();}
		if(ncycle1==4){_out.channels[BUZZ_CHANNEL] = MIN_PULSEWIDTH;myYellowLed.switchOn();}
		if(ncycle1==5){ncycle1=1;}
		_pwm.force_out(BUZZ_CHANNEL);
		break;
	case S_ALL_ON:
		myBlueLed.switchOff();
		myRedLed.switchOff();
		myYellowLed.switchOff();
		break;
	case S_ALL_OFF:
		myBlueLed.switchOn();
		myRedLed.switchOn();
		myYellowLed.switchOn();
		break;
	case S_CHENILLE_LF:
		if (ncycle % FAST == 0)ncycle1 = ncycle1+1;
		if(ncycle1==1){myBlueLed.switchOff();myRedLed.switchOn();myYellowLed.switchOn();}
		if(ncycle1==2){myBlueLed.switchOn();myRedLed.switchOn();myYellowLed.switchOff();}
		if(ncycle1==3){myRedLed.switchOff();myYellowLed.switchOn();myBlueLed.switchOn();}
		if(ncycle1==4){ncycle1=1;}
		break;
	case S_CHENILLE_LS:
		if (ncycle % SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1==1){myBlueLed.switchOff();myRedLed.switchOn();myYellowLed.switchOn();}
		if(ncycle1==2){myBlueLed.switchOn();myRedLed.switchOn();myYellowLed.switchOff();}
		if(ncycle1==3){myRedLed.switchOff();myYellowLed.switchOn();myBlueLed.switchOn();}
		if(ncycle1==4){ncycle1=1;}
		break;
	case S_CHENILLE_RF:
		if (ncycle % FAST == 0)ncycle1 = ncycle1+1;
		if(ncycle1==3){myBlueLed.switchOff();myRedLed.switchOn();myYellowLed.switchOn();}
		if(ncycle1==2){myBlueLed.switchOn();myRedLed.switchOn();myYellowLed.switchOff();}
		if(ncycle1==1){myRedLed.switchOff();myYellowLed.switchOn();myBlueLed.switchOn();}
		if(ncycle1==4){ncycle1=1;}
		break;
	case S_CHENILLE_RS:
		if (ncycle % SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1==3){myBlueLed.switchOff();myRedLed.switchOn();myYellowLed.switchOn();}
		if(ncycle1==2){myBlueLed.switchOn();myRedLed.switchOn();myYellowLed.switchOff();}
		if(ncycle1==1){myRedLed.switchOff();myYellowLed.switchOn();myBlueLed.switchOn();}
		if(ncycle1==4){ncycle1=1;}
		break;
	case S_BLUE:
		myBlueLed.switchOff();
		myRedLed.switchOn();
		myYellowLed.switchOn();
		break;
	case S_YELLOW:
		myBlueLed.switchOn();
		myRedLed.switchOn();
		myYellowLed.switchOff();
		break;
	case S_RED:
		myBlueLed.switchOn();
		myRedLed.switchOff();
		myYellowLed.switchOn();
		break;
	case S_BLUE_BS:
		if (ncycle % SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOff();
			myRedLed.switchOn();
			myYellowLed.switchOn();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	case S_YELLOW_BS:
		if (ncycle % SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOff();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	case S_RED_BS:
		if (ncycle % SLOW == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOn();
			myRedLed.switchOff();
			myYellowLed.switchOn();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	case S_BLUE_BF:
		if (ncycle % FAST == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOff();
			myRedLed.switchOn();
			myYellowLed.switchOn();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	case S_YELLOW_BF:
		if (ncycle % FAST == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOff();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	case S_RED_BF:
		if (ncycle % FAST == 0)ncycle1 = ncycle1+1;
		if(ncycle1 >=1){
			myBlueLed.switchOn();
			myRedLed.switchOff();
			myYellowLed.switchOn();
			ncycle1=0;
		}
		else
		{
			myBlueLed.switchOn();
			myRedLed.switchOn();
			myYellowLed.switchOn();
		}
		break;
	}

}
}
