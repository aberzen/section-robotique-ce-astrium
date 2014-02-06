#ifndef PWM_H__
#define PWM_H__


#include <hw/common/include/Driver.hpp>

namespace hw {

// Radio channels

// Note channels are from 0!
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10

#define PWM_IN_NUM_CHANNELS 11
#define PWM_OUT_NUM_CHANNELS 8

#define MIN_PULSEWIDTH     900
#define MAX_PULSEWIDTH    2100

//#define NUM_CHANNELS 8
#define MIN_CHANNELS 5      // for ppm sum we allow less than 8 channels to make up a valid packet


class Pwm : public Driver
{
public:
	typedef struct {
		uint16_t channels[PWM_OUT_NUM_CHANNELS];
		bool isAvailable;
	} Output;

	typedef struct {
		uint16_t channels[PWM_IN_NUM_CHANNELS];
	} Input;

public:
	Pwm(
			/* Inputs */
			const Input& in,
			/* Outputs */
			Output& out
			);
	virtual ~Pwm() {
	}

    virtual uint8_t getState() = 0;
    virtual void setFastOutputChannels( uint32_t channelmask, uint16_t speed_hz = 400 ) = 0;
    virtual void enable_out(uint8_t) = 0;
    virtual void disable_out(uint8_t) = 0;
    virtual void force_out(uint8_t) = 0;

protected:
	/* Inputs */
    const Input& _in;
	/* Outputs */
	Output& _out;

    uint16_t _map_speed(uint16_t speed_hz) {
        return 2000000UL / speed_hz;
    }
//    static volatile uint32_t         _last_update; // Modified by interrupt

};

}  /* namespace hw */

#endif

