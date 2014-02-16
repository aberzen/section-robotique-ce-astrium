/*
 * FrameScheduling.hpp
 *
 *  Created on: 10 févr. 2014
 *      Author: Robotique
 */

#ifndef FRAMESCHEDULING_HPP_
#define FRAMESCHEDULING_HPP_

#include <infra/app/include/Process.hpp>

namespace infra {

class FrameScheduling : public infra::Process{
protected:
	typedef struct Frame {
		infra::Process** processes;
		uint8_t nb;
		Frame* next;
	} Frame;

protected:
	FrameScheduling();

public:
	FrameScheduling(const Frame* frame);
	virtual ~FrameScheduling();

	/** @brief Init the process */
	virtual void initialize() ;

	/** @brief Execute the process */
	virtual void execute() ;

protected:

	/** @brief Set frame */
	inline void setFrame(const Frame* frame);

private:
	/** @brief First frame */
	const Frame* _first;

	/** @brief Current frame */
	const Frame* _current;
};

/** @brief Set frame */
inline void FrameScheduling::setFrame(const FrameScheduling::Frame* frame)
{
	_first = frame;
}

} /* namespace infra */

#endif /* FRAMESCHEDULING_HPP_ */
