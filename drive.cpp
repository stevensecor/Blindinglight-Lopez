#include "drive.h"
#include "iomap.h"

//CONSTANTS
#define IN_PER_TICK (0.0342727272727273)

DoubleConstant DIST_LEADING_LIFT(0.2, "DRIVE_TRAP_LIFT");
DoubleConstant DIST_FALL_SLOPE(0.04, "DRIVE_FALL_SLOPE");
DoubleConstant DIST_RISE_SLOPE(0.08, "DRIVE_RISE_SLOPE");

DoubleConstant AUTO_CLOSE_ENOUGH(1.0, "DRIVE_CLOSE_ENOUGH");

double getTrapezoidalTarget(double loc, double target, double maxp) {
	//
	// To graph the profile:
	//
	// XXXXXXXXXX  /=>=>=>=>=>=>=>=\     /=<=<=<=<=<=<=<=<=<
	// XXXXXXXXXX /                 \   /
	// XXXXXXXXXX.                   \ /
	// XXXXXXXXXX|                    .
	//
	//         start                target
	//
	//      ---------------- position --------------->

	double sign = (target - loc) > 0 ? 1.0 : -1.0;

	double near = fabs(loc);
	double far = fabs(loc - target);
	if (near < ((maxp - DIST_LEADING_LIFT) / DIST_RISE_SLOPE)) {
		return DIST_LEADING_LIFT + near * sign * DIST_RISE_SLOPE;
	}

	if (far < ((maxp - DIST_LEADING_LIFT) / DIST_FALL_SLOPE)) {
		return DIST_LEADING_LIFT + far * sign * DIST_FALL_SLOPE;
	}

	return maxp * sign;
}

Drive::Drive() :
	gyro(ANALOG_GYRO), mode(kNothing),
			leftMotors(CANID_DRIVE_LEFT_FRONT, CANID_DRIVE_LEFT_REAR, true),
			rightMotors(CANID_DRIVE_RIGHT_FRONT, CANID_DRIVE_RIGHT_REAR, true),
			leftEncoder(DIG_DRIVE_ENCODER_LEFT_B),
			rightEncoder(DIG_DRIVE_ENCODER_RIGHT_A) {
	// sample averaging (over a quarter rotation, est. good)
	leftEncoder.SetSamplesToAverage(63);
	rightEncoder.SetSamplesToAverage(63);

	leftEncoder.Start();
	rightEncoder.Start();
	lastUpdate.Start();

	initialize();
}

void Drive::initialize() {
	//Set up encoders.
	leftEncoder.Reset();
	rightEncoder.Reset();
	//Stop the motors.
	tankDrive(0, 0);
	// Set variables that need to start off with a set value to appropriate placeholders.
	lastLeftDist = leftEncoder.Get();
	lastRightDist = rightEncoder.Get();
	maxPower = 0.0;
	mode = kNothing;
}

void Drive::gyroController(double sign, double d_left, double d_right,
		double v_left, double v_right, double t_step) {
	double d_avg = (d_left + d_right) / 2;
	double target_pow = getTrapezoidalTarget(d_avg, moveTarget, maxPower);

	double delta = (moveTarget - d_avg) * sign;
	if (delta < AUTO_CLOSE_ENOUGH) {
		mode = kNothing;
		tankDrive(0.0, 0.0);
	} else {
		// pPow is forward-motion power. Now gyro correct
		// negative delta means robot pointed left-of target
		double delta = gyro.GetAngle();
		delta *= 0.05; // %pow/deg err
		// Swap delta direction -- negative in driveAndTurn means turn left
		delta *= -1.0f;

		// Add/subtract turn value
		double left = target_pow - delta;
		double right = target_pow + delta;

		left = bound(left, 0.25 * sign, maxPower * sign);
		right = bound(right, 0.25 * sign, maxPower * sign);

		tankDrive(left, right);
	}
	printf("L %3.6f R %3.6f l %3.6f r %3.6f G %3.6f\n", d_left, d_right,
			v_left, v_right, gyro.GetAngle());
}

void Drive::process() {
	double sign = (moveTarget < 0) ? -1.0 : 1.0;

	//Determine total distance traveled.
	double leftDist = leftEncoder.Get() * IN_PER_TICK * sign;
	double rightDist = rightEncoder.Get() * IN_PER_TICK * sign;

	double timestep = lastUpdate.Get();
	lastUpdate.Reset();

	double leftRate = (leftDist - lastLeftDist) / timestep;
	double rightRate = (rightDist - lastRightDist) / timestep;
	lastLeftDist = leftDist;
	lastRightDist = rightDist;

	if (mode == kAutoDrive) {
		gyroController(sign, leftDist, rightDist, leftRate, rightRate, timestep);
	}
}

inline double jagCurrent(SafeCANJag* jag) {
	if (jag == NULL) {
		return 0.0;
	}
	return jag->GetOutputCurrent();
}

void Drive::debug() {
	LCDWriter l;
	l.line1("Gyro %2.4f", gyro.GetAngle());
	l.line2("LDist: %d", leftEncoder.Get());
	l.line3("RDist: %d", rightEncoder.Get());
	double time = GetTime();
	l.line4("LCurr %05.2f %05.2f",
			jagCurrent(leftMotors.GetByID(CANID_DRIVE_LEFT_FRONT)),
			jagCurrent(leftMotors.GetByID(CANID_DRIVE_LEFT_REAR)));
	l.line5("RCurr %05.2f %05.2f",
			jagCurrent(rightMotors.GetByID(CANID_DRIVE_RIGHT_FRONT)),
			jagCurrent(rightMotors.GetByID(CANID_DRIVE_RIGHT_REAR)));
	l.line6("Automode %d Rd4x %f", mode, GetTime() - time);
}

void Drive::tankDrive(double left_power_percentage,
		double right_power_percentage) {
	leftMotors.Set(left_power_percentage);
	rightMotors.Set(-right_power_percentage);
}

void Drive::autonDrive(double distance_inches, double max_power_percentage) {
	//Set the move target that process checks against and the max speed for this drive segment.
	moveTarget = distance_inches;
	leftEncoder.Reset();
	rightEncoder.Reset();
	;
	lastLeftDist = 0.0;
	lastRightDist = 0.0;
	lastUpdate.Reset();
	gyro.Reset();

	maxPower = max_power_percentage;

	if (moveTarget == 0) {
		mode = kNothing;
		tankDrive(0.0, 0.0);
		return;
	}

	mode = kAutoDrive;
}

bool Drive::autoDone() {
	//Determine whether or not the robot is moving.
	return (mode == kNothing);
}

void Drive::finalizeGyro() {
	gyro.FinishMeasurement();
}
void Drive::measureGyro() {
	gyro.BeginMeasurement();
}
double Drive::getRawGyro() {
	return gyro.GetRawLevel();
}
