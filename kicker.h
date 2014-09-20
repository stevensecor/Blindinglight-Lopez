#ifndef KICKER_H_
#define KICKER_H_

#include "intake.h"
#include "util.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "Servo.h"
#include "Timer.h"

/**
 * This system controls the kicker arm, corresponding sensors,
 * and the guard that prevents the ball from rolling out of the cradle
 * when in the ready position. It depends on the Intake to know if
 * the ball is present and to know if the intake angle is high enough
 * to avoid damaging the robot when kicking.
 *
 */
class Kicker {
public:
	Kicker(Intake &i);

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
	 * Begins the kicking sequence:
	 * system will wait until the Intake has the ball
	 * in the correct position, then lower the guard
	 * and swing the kicker into the ball.
	 * 
	 * The sequence ends when cancelKick is called or
	 * the ball has been impelled.
	 * 
	 * "power", range 0 to 1, is the power at which the shot 
	 * will be performed.
	 */
	void startKick(double power);

	/**
	 * Call if the broken status of the kicker sensors changes.
	 * This puts the kicker into broken mode.
	 */
	void setSensorsBroken(bool broke);

	/**
	 * Apply power to kicker motors; positive "power" means upward rotation (kick)
	 */
	void turnKicker(double power);
	/*
	 * returns 1 if the kicker is in the process of kicking
	 */
	bool nowKicking();
	/*
	 * returns 1 if the kicker is ready to kick (does not need to reset)
	 */
	bool isReset();
	/*
	 * returns the value of high flag (1 is tripped, 0 is not)
	 */
	bool getHighFlag();
	/*
	 * returns the value of the low flag(1 is tripped, 0 is not)
	 */
	bool getLowFlag();

	/**
	 * If true, guards move out. If false, no guarantees.
	 */
	void setGuard(bool state);
	
	/**
	 * moves guards out if out is true; otherwise defers to kicker 
	 */
	void overrideGuards(double left, double right);
private:
	/**
	 * Print data after the kick: did any Jaguars experience faults?
	 */
	void kickPostMortem();

	Intake &intake;
	MultiMotor motors;
	Encoder kicker_encoder;
	DigitalInput low_flag;
	DigitalInput high_flag;
	Counter low_counter;
	Servo servo_guard_left;
	Servo servo_guard_right;

	Timer timer;

	typedef enum {
		kKickInactive,
		kKickOpenGuard,
		kKickWaitForIntake,
		kKickSpinning,
		kKickStopping,
		kKickResetting
	} KickerState;

	bool sensorsBroken;
	KickerState state;

	double startKickPower;

	bool servoTuning;
	bool guardsSet;

	void processGuard();
};

#endif
