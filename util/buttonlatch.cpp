#include "buttonlatch.h"

ButtonLatch::ButtonLatch(Joystick &j, int id) :
	joystick(j), jid(id), val(false), polled_yet(false) {
}
bool ButtonLatch::poll(bool &value) {
	value = this->value();
	return poll();
}
bool ButtonLatch::poll() {
	bool b = joystick.GetRawButton(jid);
	if (!polled_yet || b != val) {
		val = b;
		polled_yet = true;
		return true;
	} else {
		return false;
	}
}
bool ButtonLatch::value() {
	return joystick.GetRawButton(jid);
}
