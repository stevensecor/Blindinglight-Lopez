#ifndef DRIVE_H_
#define DRIVE_H_

#include <math.h>
#include "util.h"
#include "Counter.h"
#include "Timer.h"
#include "Gyro.h"

/**
 * This system controls the drivetrain and its motion.
 */
class Drive {
public:
	Drive();
	/**
	 * Reset state at start of a mode.
	 */
	void initialize();
	/**
	 * Needs to run after the autonomous drive function. 
	 * Run every cycle. Automaticaly straightens itself
	 * using encoders.
	 */
	void process();
	/**
	 * Writes speed (inches per second) to driver station lcd.
	 */
	void debug();
	/**
	 * Apply left and right power to drivetrain. 
	 * Used for movement throughout the class.
	 */
	void tankDrive(double left_power_percentage, double right_power_percentage);
	/**
	 * Sets up for process. Tells robot to drive in a
	 * straight line for a specified distance. Distance
	 * can be any number of inches (positive or negative).
	 * Negative makes the robot move backwards. Max power
	 * goes from zero to positive one.
	 */
	void autonDrive(double distance_inches, double max_power_percentage);
	/*
	 * Checks to see if the robot is moving. If the robot
	 * is not moving, this function returns true.
	 */
	bool autoDone();

	void finalizeGyro();
	void measureGyro();
	double getRawGyro();
private:
	typedef enum {
		kAutoDrive, kAutoTurn, kNothing
	} AutoMode;
	RollingGyro gyro;
	AutoMode mode;
	MultiMotor leftMotors;
	MultiMotor rightMotors;
	Counter leftEncoder;
	Counter rightEncoder;

	void gyroController(double,double,double,double,double,double);

	double maxPower;// range 0 to 1, max abs output for motors in autonomous
	double moveTarget;// signed value indicating the target location to go to (inches)

	Timer lastUpdate;
	double lastLeftDist;// inches
	double lastRightDist;// inches
};

#endif
