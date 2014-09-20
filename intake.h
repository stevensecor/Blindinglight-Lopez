#ifndef INTAKE_H_
#define INTAKE_H_

#include "util.h"
#include "armcontroller.h"
#include "Notifier.h"
#include "AnalogChannel.h"
#include "DigitalInput.h"

class Intake;
class IntakePot {
public:
	static const double kPeriod = 0.005;

	IntakePot(uint8_t analog_channel, Intake& i);
	void StartWatching();
	void StopWatching();

	double getLocation();
	double getVoltage();
private:
	static void callUpdate(void* x);
	void update();

	SEM_ID semaphore;
	bool watching;
	MovingAverageFilter filter;
	Notifier noti;
	AnalogChannel pot;
	Intake &intake;
};

/**
 * This system manages the intake roller and the
 * linkage system that moves the roller and cradle.
 */
class Intake {
public:
	typedef enum {
		kSpinIn,
		kSpinNot,
		kSpinOut,
		kSpinOutSlow,
		kSpinCustomIn,
		kSpinCustomOut
	} SpinMode;
	typedef enum {
		kLocFeed, kLocAutoKick, kLocArcKick, kLocTeleopKick
	} Position;

	Intake();
	/**
	 * Reset state at start of a mode.
	 */
	void initialize();
	/**
	 * Update state every cycle.
	 */
	void process();
	/**
	 * Write debugging info to driver station lcd.
	 */
	void debug();

	/**
	 * Spin roller in specified direction.
	 */
	void spin(SpinMode spin);

	/**
	 * Use position control to find
	 */
	void goToLocation(Position p);

	/**
	 * Raise or lower intake arm and cradle;
	 */
	void moveArm(double down_power);

	/**
	 * isRaised returns true iff the arm and cradle are high enough to shoot.
	 * isLowered returns true iff they are fully down.
	 * isPosOver returns true iff value above target setpoint
	 */
	bool isRaised();
	bool isLowered();
	bool isPosOver();

	/**
	 * Returns true if ball beam break sensor tripped;
	 */
	bool isBallPresent();

	/**
	 * Call if the broken status of the potentiometer changes.
	 */
	void setPotBroken(bool broke);

	/**
	 * Returns normalized range scaled from potentiometer reading;
	 * 0.0 is low position, 1.0 is high position
	 */
	double getLocation();

	/**
	 * Returns the location, using alternate potentiometer.
	 */
	double getUnsloppedLocation();

	/**
	 * Return the current target for position control.
	 */
	double getTargetPos();

	/**
	 * Get last power written to the motor.
	 */
	double getLastLiftOutput();

	/**
	 * Name says it all. Sets the custom intake speed; setting persists.
	 */
	void setCustomSpinSpeed(double d);

	/**
	 * Log intake pos/time/output data
	 */
	void setModelController(bool logging);
	

	/**
	 * Set the lift motor output.
	 * Internal/Test Use only
	 */
	void setLiftDirect(double output);

	/**
	 * Replace other pot w/ the alternate.
	 */
	void useAlternatePot(bool yes_p);
private:
	/*
	 * pos is a range from 0.0 (feeding position) to 1.0
	 * (kicking position).
	 */
	void moveToPosition(double pos);


	MultiMotor motor_roller;
	MultiMotor motor_lift;
	IntakePot pot;
	IntakePot altpot;
	DigitalInput beam_break;
	// NaiveController
	BumpController pos_controller;
	ModelController alt_controller;

	typedef enum {
		kPower, kPosition
	} IntakeMode;

	double target_power;
	double target_pos;
	SpinMode direction;
	bool potbroke;
	IntakeMode mode;

	double last_output;
	double custom_spin_speed;
	bool use_alt;
};

#endif
