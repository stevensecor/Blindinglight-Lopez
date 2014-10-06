#ifndef CONTROLS_H_
#define CONTROLS_H_

#include "util.h"
#include "drive.h"
#include "intake.h"
#include "kicker.h"
#include "lights.h"
#include "auto.h"

/**
 * The Controls system is active during Teleop and Disabled modes.
 * (the Auto system runs everything during Autonomous).
 * 
 * It handles all driver station joystick input, including broken switches
 * and smartdashboard input (if any). This includes applying ramping and deadbands to
 * the joysticks, and polling broken switches so new values are only applied
 * on change.
 */
class Controls {
public:
	Controls(Drive &d, Intake &i, Kicker &k, Lights &l);
	/**
	 * Reset state at start of a mode.
	 */
	void initialize(bool enabled);
	/**
	 * Update state every cycle. If enabled is false,
	 * only process debug buttons, broken switches,
	 * SmartDashboard updates, etc. If enabled is true,
	 * read and act on the previous items AND joystick input,
	 */
	void process(bool enabled);

	/**
	 * Return the selected autonomous mode,
	 * for use by the Auto subsystem.
	 */
	Auto::AutoRoutine getAutoMode();
private:
	
	void processBrokenSwitches();
	void processSmartDashboard();
	void processDriveSticks();
	void processDebug();
	void processAuxSide();
	
	double getManualPower();

	Drive &drive;
	Intake &intake;
	Kicker &kicker;
	Lights &lights;

	Joystick stick_drive; //-- Blindinglight implemetation
	Joystick stick_aux;
	Joystick stick_virtual;

	ButtonLatch broken_cradle_pot;
	ButtonLatch broken_kicker_sensors;
	ButtonLatch guard_tuning;
	ButtonLatch alternate_pot;
	ButtonLatch intake_speed_tuning;
	ButtonLatch auto_test;

	ButtonLatch trussKick;
	ButtonLatch highKick;
	ButtonLatch manualKick;

	ButtonLatch resetMotors;

	double prev_power;
	double prev_angle;
	bool prev_ball;
	bool start_dash;
		
	typedef enum {kDebugDrive, kDebugIntake, kDebugKicker, kDebugClear} DebugType;
	DebugType lastDebug;
};

#endif
