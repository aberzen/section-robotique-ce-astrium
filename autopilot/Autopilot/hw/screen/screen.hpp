/*
 * screen.hpp
 *
 *  Created on: 5 mars 2014
 *      Author: vincent
 */

#ifndef SCREEN_HPP_
#define SCREEN_HPP_

#include <stdint.h>
#include <hw/led/Led.hpp>
#include <hw/pwm/Pwm.hpp>

namespace hw {
#define SLOW 64
#define FAST 16
#define VERY_SLOW 128
#define BUZZ_CHANNEL 5 // RC_CHANNEL -1 (6 dans ce cas)

typedef enum EScreenState{
	S_ALL_ON=0,
	S_ALL_OFF,
	S_BLUE,
	S_YELLOW,
	S_RED,
	S_BLUE_BS,
	S_YELLOW_BS,
	S_RED_BS,
	S_BLUE_BF,
	S_YELLOW_BF,
	S_RED_BF,
	S_WARNING_1,
	S_WARNING_2,
	S_WARNING_3,
	S_WARNING_4,
	S_OK_1,
	S_OK_2,
	S_OK_3,
	S_CHENILLE_LS,//left slow
	S_CHENILLE_LF,
	S_CHENILLE_RS,
	S_CHENILLE_RF,//right fast
	S_BUZZ
}ScreenState;

class Screen {
public:
	Screen();
	virtual ~Screen();

	/* Display predifined state */
	void Display(ScreenState S);

protected:
	ScreenState _state;
	uint16_t ncycle,ncycle1,slow,fast;
	hw::Led myBlueLed;
	hw::Led myYellowLed;
	hw::Led myRedLed;
private:
	/* @brief switch on the led w/o state update */
	//void _switchOn(void);

	/* @brief switch off the led w/o state update */
	//void _switchOff(void);
public:
	/* called by cyclic task for blink */
	void update();
};


} /* namespace hw */




#endif /* SCREEN_HPP_ */
