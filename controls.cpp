#include "controls.h"
#include "SmartDashboard/SmartDashboard.h"

// Joystick IDs
const int STICK_DRIVE = 1;
const int STICK_AUX = 2;
const int DRIVER_STATION = 3;
const int STICK_VIRTUAL = 4;

// Driver Stick Axis - SteelSeries Joypad
const int DRIVE_LEFT_STICK = 2;
const int DRIVE_RIGHT_STICK = 5;

// Driver joystick
const int DRIVE_TURBO = 7; //Steelseries Left Bumper
const int DRIVE_SLOW = 8; //Steelseries Right Bumper
const int DRIVE_FLASH = 9; //Steelseries 'Square' button
const int RESET_MOTORS = 10; //Steelseries 'Triangle' button

// Aux driver joystick
const int AUX_INITIATE_KICK = 2;    // Kick
const int AUX_RAISE_ARM = 4;		// Raise Arm
const int AUX_RELEASE_SLOW = 6;     // Normal Out
const int AUX_INTAKE_SLOW = 5;   	// Normal in
const int AUX_RELEASE_FAST = 10;	// Fast Out
const int AUX_SHOT_CONTROL = 6;		// DPad Left=Truss (neg values), Right=High (pos values)
const int AUX_SHOT_TRUSS = -1;
const int AUX_SHOT_HIGH = 1;

// Debug Modes -- Only 1 at a time.
const int DEBUG_DRIVE = 0;
const int DEBUG_INTAKE = 0;
const int DEBUG_KICKER = 1;
const int DEBUG_CLEAR = 0;

// Auto choice -- Only 1 at a time.
const int AUTO_CHOICE_2 = 0;
const int AUTO_CHOICE_1 = 1;

// Guard Constant
const int GUARD_OPEN = 1;    // Wiskers

// Aux driver panel (virtual joystick)
const int VIRTUAL_TRUSS_SHOT = 1;
const int VIRTUAL_HIGH_SHOT = 2;
const int VIRTUAL_MANUAL_POWER_PUSH = 3;
const int VIRTUAL_AUTO_CHOICE_2 = 5;
const int VIRTUAL_AUTO_CHOICE_1 = 6;
const int BROKEN_SWITCH_1 = 7;
const int BROKEN_SWITCH_2 = 8;
const int BROKEN_SWITCH_3 = 9;
const int BROKEN_SWITCH_4 = 10;
const int BROKEN_SWITCH_5 = 11;
const int BROKEN_SWITCH_6 = 12;

// Joystick parameters
const double DRIVE_JOYSTICK_DEADBAND = 0.05;
const double INTAKE_JOYSTICK_DEADBAND = 0.05;

DoubleConstant TRUSS_KICK_POWER(0.65, "CONTROLS_KICK_POW_TRUSS");
DoubleConstant HIGH_GOAL_POWER(0.9, "CONTROLS_KICK_POW_HIGH");

// Dashboard variable names
const char* MANUAL_POWER = "manual_power";
const char* FLASH_SCREEN = "flash_screen";
const char* INTAKE_ANGLE = "intake_angle";

Controls::Controls(Drive &d, Intake &i, Kicker &k, Lights &l) :
	drive(d), intake(i), kicker(k), lights(l),
	// Controllers
	stick_drive(STICK_DRIVE),
	stick_aux(STICK_AUX),
	stick_virtual(STICK_VIRTUAL),

	// Broken 
	broken_cradle_pot(stick_virtual, BROKEN_SWITCH_1),
	broken_kicker_sensors(stick_virtual, BROKEN_SWITCH_2),
	guard_tuning(stick_virtual, BROKEN_SWITCH_3),
	alternate_pot(stick_virtual, BROKEN_SWITCH_4),
	intake_speed_tuning(stick_virtual, BROKEN_SWITCH_5),
	auto_test(stick_virtual, BROKEN_SWITCH_6),
	
	//Kick
	trussKick(stick_virtual, VIRTUAL_TRUSS_SHOT),
	highKick(stick_virtual, VIRTUAL_HIGH_SHOT),
	manualKick(stick_virtual, VIRTUAL_MANUAL_POWER_PUSH),

	// resetMotors(stick_drive_right, RESET_MOTORS) { -- 1511 Code
	resetMotors(stick_drive, RESET_MOTORS) { 
			
	intake.setPotBroken(false);
	kicker.setSensorsBroken(false);
	lastDebug = kDebugClear;
	start_dash = true;
}

void Controls::initialize(bool enabled) {
	if (start_dash) {
		start_dash = false;
		prev_ball = intake.isBallPresent();
		SmartDashboard::PutBoolean(FLASH_SCREEN, prev_ball);
		prev_power = getManualPower();
		SmartDashboard::PutNumber(MANUAL_POWER, prev_power);
		prev_angle = intake.getLocation();
		SmartDashboard::PutNumber(INTAKE_ANGLE, prev_angle);
	}
}

void Controls::process(bool enabled) {
	if (resetMotors.poll() && resetMotors.value()) {
		double t = GetTime();
		MultiMotor::ReflashAll();
		printf("\n\t\tREFLASH (%2.6f s)\n\n", GetTime() - t);
	}

	lights.flash(
		stick_drive.GetRawButton(DRIVE_FLASH));

	processBrokenSwitches();
	if (enabled) {
		processDriveSticks();
		processAuxSide();
	}
	processDebug();
	processSmartDashboard();
}

Auto::AutoRoutine Controls::getAutoMode() {
	if (AUTO_CHOICE_1) {
		return Auto::kAutoA;
	} else if (AUTO_CHOICE_2) {
		return Auto::kAutoB;
	} else {
		return Auto::kNone;
	}
}

void Controls::processBrokenSwitches() {
	if (broken_cradle_pot.poll()) {
		intake.setPotBroken(broken_cradle_pot.value());
	}
	if (broken_kicker_sensors.poll()) {
		//kicker.setSensorsBroken(broken_kicker_sensors.value());
		//kicker.setSensorsBroken(1);
	}
	if (guard_tuning.poll()) {
	}
	if (alternate_pot.poll()) {
		intake.useAlternatePot(true);
	}
	if (intake_speed_tuning.poll()) {
	}
	if (auto_test.poll()) {
	}
}

void Controls::processSmartDashboard() {
	bool b = intake.isBallPresent();
	if (b != prev_ball) {
		prev_ball = b;
		SmartDashboard::PutBoolean(FLASH_SCREEN, b);
	}

	double np = getManualPower();
	if (np != prev_power) {
		prev_power = np;
		SmartDashboard::PutNumber(MANUAL_POWER, np);
	}

	double ang = intake.getLocation();
	if (ang != prev_angle) {
		prev_angle = ang;
		SmartDashboard::PutNumber(INTAKE_ANGLE, ang);
	}
}

double deadband(double v, double radius) {
	if (v <= radius && v >= -radius) {
		return 0.0;
	} else {
		return v;
	}
}

double stick_power(double v, bool turbo, bool slow) {
	const double x = deadband(v, DRIVE_JOYSTICK_DEADBAND);
	const double cb = x * x * x;

	if (turbo) {
		return cb * 1.0;
	} else if (slow) {
		return cb * 0.3;
	} else {
		return cb * 0.7;
	}
}

void Controls::processDriveSticks() {
	 bool turbo = stick_drive.GetRawButton(DRIVE_TURBO);
	 bool slow = stick_drive.GetRawButton(DRIVE_SLOW);	

	// Blindinglight stick control using Steelseries contoller
	double left_power = stick_power(stick_drive.GetRawAxis(DRIVE_LEFT_STICK), turbo, slow);
	double right_power = stick_power(stick_drive.GetRawAxis(DRIVE_RIGHT_STICK), turbo, slow);
		
	drive.tankDrive(left_power, right_power);
}

void Controls::processDebug() {
	//
	// If several debug buttons are pressed, the most important is selected.
	//
 	if (DEBUG_DRIVE) {
		lastDebug = kDebugDrive;
	} else if (DEBUG_INTAKE) {
		lastDebug = kDebugIntake;
	} else if (DEBUG_KICKER) {
		lastDebug = kDebugKicker;
	} else if (DEBUG_CLEAR) {
		lastDebug = kDebugClear;
	}
	switch (lastDebug) {
	case kDebugDrive:
		drive.debug();
		return;
	case kDebugIntake:
		intake.debug();
		return;
	case kDebugKicker:
		kicker.debug();
		return;
	case kDebugClear:
		LCDWriter().clear();
		return;
	}
}

void Controls::processAuxSide() {
	// intake roller control
	bool icspd = intake_speed_tuning.value();
	intake.setCustomSpinSpeed(
			scaleLinear(stick_aux.GetRawAxis(4), -1.0, 1.0, 0.2, 1));
	if (stick_aux.GetRawButton(AUX_INTAKE_SLOW)) {
		intake.spin(icspd ? Intake::kSpinCustomIn : Intake::kSpinIn);
	} else if (stick_aux.GetRawButton(AUX_RELEASE_SLOW)) {
		intake.spin(icspd ? Intake::kSpinCustomOut : Intake::kSpinOutSlow);
	} else if (stick_aux.GetRawButton(AUX_RELEASE_FAST)) {
		intake.spin(icspd ? Intake::kSpinCustomOut : Intake::kSpinOut);
	} else {
		intake.spin(Intake::kSpinNot);
	}

	// intake lift arm control
	if (stick_aux.GetRawButton(AUX_RAISE_ARM)) {
		intake.goToLocation(Intake::kLocTeleopKick);
	} else {
		double auxLoc = deadband(stick_aux.GetRawAxis(2), INTAKE_JOYSTICK_DEADBAND);
		//intake.moveArm(-auxLoc);
		intake.moveArm(auxLoc);
	}

	// control guards and the broken switch enabled case
	if ((stick_aux.GetRawAxis(AUX_SHOT_CONTROL) == AUX_SHOT_TRUSS) ||
			(stick_aux.GetRawAxis(AUX_SHOT_CONTROL) == AUX_SHOT_HIGH)) {
		kicker.setGuard(GUARD_OPEN);
	}

	if ((stick_aux.GetRawAxis(AUX_SHOT_CONTROL) == AUX_SHOT_TRUSS) &&
			stick_aux.GetRawButton(AUX_INITIATE_KICK)) {
		kicker.startKick(TRUSS_KICK_POWER);
	}
	if ((stick_aux.GetRawAxis(AUX_SHOT_CONTROL) == AUX_SHOT_HIGH) && 
			stick_aux.GetRawButton(AUX_INITIATE_KICK)) {
		kicker.startKick(HIGH_GOAL_POWER);
	}
}

double Controls::getManualPower() {
	return ((-stick_virtual.GetX() + 1.0) / 2.0);
}
