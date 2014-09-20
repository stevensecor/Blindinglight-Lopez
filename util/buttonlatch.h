#ifndef UTIL_BUTTONLATCH_H_
#define UTIL_BUTTONLATCH_H_

#include "Joystick.h"

class ButtonLatch {
public:
	ButtonLatch(Joystick &j, int id);
	/**
	 * Returns true if joystick button value changed since
	 * the last time one of the poll() functions was called.
	 * 
	 * _Always_ sets the reference's value to be the 
	 * value of the joystick button.
	 */
	bool poll(bool &value);

	/**
	 * Returns true if joystick button value changed since
	 * the last time one of the poll() functions was called.
	 */
	bool poll();
	/**
	 * Returns the current joystick button value, but
	 * does not update ButtonLatch state.
	 */
	bool value();
private:
	Joystick &joystick;
	int jid;
	bool val;
	bool polled_yet;
};

#endif

