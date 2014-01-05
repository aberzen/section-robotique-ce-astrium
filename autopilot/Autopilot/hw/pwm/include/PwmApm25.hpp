#ifndef PWMAPM25_H__
#define PWMAPM25_H__


#include <hw/pwm/include/Pwm.hpp>

namespace hw {

class PwmApm25 : public hw::Pwm
{
private:
public:
	PwmApm25(
			/* Inputs */
			const Input& in,
			/* Outputs */
			Output& out);
	virtual ~PwmApm25();

	/** @brief Init the process */
	virtual ::infra::status initialize() ;

	/** @brief Execute the process */
	virtual ::infra::status execute() ;

	/** @brief Reset the HW */
	virtual infra::status reset() ;

    virtual uint8_t getState();
    virtual void setFastOutputChannels(uint32_t chmask, uint16_t speed_hz = 400);

    virtual void enable_out(uint8_t);
    virtual void  disable_out(uint8_t);
    virtual void force_out(uint8_t);

public:
    static void _timer5_capt_cb(void);

protected:
    void OutputCh(uint8_t ch, uint16_t pwm);

protected:
    static volatile uint16_t        _PWM_RAW[PWM_OUT_NUM_CHANNELS];
    static volatile uint8_t         _radio_status;
};

} /* namespace hw */
#endif
