#include "lights.h"
#include "iomap.h"
#include "RobotBase.h"

const double NORM_PWMRATE = 1000.0;
DoubleConstant RANDOM_PERIOD(6.0, "LIGHTS_RANDOM_PERIOD");
DoubleConstant FLASH_PERIOD(0.21, "LIGHTS_FLASH_PERIOD");

Lights::Lights(Drive &d, Intake &i, Kicker &k) :
	drive(d), intake(i), kicker(k), indicator_R(DIG_LIGHTS_INDICATE_RED),
			indicator_G(DIG_LIGHTS_INDICATE_GREEN),
			indicator_B(DIG_LIGHTS_INDICATE_BLUE),
			camera_ring(DIG_LIGHTS_CAMERA), timer() {
	indicator_R.SetPWMRate(NORM_PWMRATE);
	indicator_B.SetPWMRate(NORM_PWMRATE);
	indicator_G.SetPWMRate(NORM_PWMRATE);
	indicator_R.EnablePWM(0.0);
	indicator_G.EnablePWM(0.0);
	indicator_B.EnablePWM(0.0);
	timer.Start();
}
void Lights::initialize() {
	setColor(0, 0, 0);
	camera_ring.Set(false);
}
void Lights::process() {
	// NOTE: may need to rethink lights due to lack of isBallPresent
	// functionality (robot sensor unattached)
	if (flash_mode) {
		flashLights(1,1,1);
		return;
	}
	if (RobotBase::getInstance().IsDisabled()) {
		if (intake.isPosOver()) {
			randomLights();// we use the cooler version for the pre-auto position
		} else if (intake.isRaised()) {
			setColor(1, 0.5, 0);
		} else {
			setColor(0, 0, 0);
		}
	} else {
		if (kicker.isReset() && intake.isRaised() && intake.isBallPresent()) {
			// LOW: Red
			// SAFE: Green
			// A TAD HIGH: Blue/Purple
			if (intake.isPosOver()) {
				setColor(0.5, 0.0, 1.0);
			} else if (intake.isRaised()) {
				setColor(0.0, 1.0, 0);
			} else {
				setColor(1.0, 0.0, 0.0);
			}
		} else if (intake.isBallPresent()) {
			setColor(0.5, 0, 0);
		} else {
			randomLights();
		}
	}
}

void Lights::setHue(double hue) {
	double h = hue * 6.0;
	double Red, Green, Blue;
	if (h >= 0 && h < 1) {
		Red = 1;
		Green = 0;
		Blue = h;
	} else if (h >= 1 && h < 2) {
		Red = 2 - h;
		Green = 0;
		Blue = 1;
	} else if (h >= 2 && h < 3) {
		Red = 0;
		Green = h - 2;
		Blue = 1;
	} else if (h >= 3 && h < 4) {
		Red = 0;
		Green = 1;
		Blue = 4 - h;
	} else if (h >= 4 && h < 5) {
		Red = h - 4;
		Green = 1;
		Blue = 0;
	} else if (h >= 5 && h < 6) {
		Red = 1;
		Green = 6 - h;
		Blue = 0;
	}
	setColor(Red, Green, Blue);
}

void Lights::randomLights() {
	if (timer.Get() > RANDOM_PERIOD) {
		timer.Reset();
	}
	setHue(timer.Get() / RANDOM_PERIOD);
}

void Lights::setCameraLight(bool on) {
	if (on) {
		camera_ring.Set(true);
	} else {
		camera_ring.Set(false);
	}
}

void Lights::setColor(float r, float g, float b) {
	indicator_B.UpdateDutyCycle(b);
	indicator_G.UpdateDutyCycle(g);
	indicator_R.UpdateDutyCycle(r);
}

void Lights::flashLights(float r, float g, float b) {
	if (timer.Get() < FLASH_PERIOD) {
		setColor(r,g,b);
	} else if (timer.Get() < 2 * FLASH_PERIOD) {
		setColor(0,0,0);
	} else {
		setColor(0,0,0);
		timer.Reset();
	}
}

void Lights::flash(bool yes) {
	flash_mode = yes;
}
