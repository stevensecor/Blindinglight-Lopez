#ifndef LIGHTS_H_
#define LIGHTS_H_

#include "drive.h"
#include "intake.h"
#include "kicker.h"
#include "DigitalOutput.h"

/**
 * Controls the lights on the robot. Its mode is
 * determined by the Controls and Auto systems;
 * what it does specifically may be dependent on the
 * Drive, Intake, and Kicker systems' states.
 */
class Lights {
public:
	Lights(Drive &d, Intake &i, Kicker &k);
	/**
	 * Reset state at start of a mode.
	 */
	void initialize();
	/**
	 * Update state every cycle.
	 */
	void process();
	
	/**
	 * Flash the lights if yes is true.
	 */
	void flash(bool yes);

	/**
	 * If true, turns on the camera ring light.
	 */
	void setCameraLight(bool on);
private:
	/**
	 * Takes a value from 0 to 1, then sets a color on a bright edge 
	 * of the RGB-color cube depending on the color.
	 */
	void setHue(double hue);
	
	/**
	 * Set the light colors as RGB according to the arguments
	 */
	void setColor(float r, float g, float b);
	/**
	 * Like the name says. No guarantees on light color.
	 */
	void randomLights();
	
	void flashLights(float r, float g, float b);
	Drive &drive;
	Intake &intake;
	Kicker &kicker;

	DigitalOutput indicator_R;
	DigitalOutput indicator_G;
	DigitalOutput indicator_B;
	DigitalOutput camera_ring;
	Timer timer;
	
	bool flash_mode;
};

#endif
