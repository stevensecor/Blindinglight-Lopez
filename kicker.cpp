#include "kicker.h"
#include "iomap.h"

const double ENCODER_HIGH_LIMIT = 95; // ticks, from low limit angle to stopping angle, may include noise

DoubleConstant INTAKE_WAIT_TIME(1.00, "KICKER_INTAKE_WAIT_TIME");

DoubleConstant SAFETY_WAIT_TIME(1.75, "KICKER_PREKICK_MAXWAIT");

DoubleConstant STOP_TIME(1.0, "KICKER_STOP_TIME");

DoubleConstant KICKER_RESET_SPEED(0.1, "KICKER_RESET_SPEED");//power
DoubleConstant RESET_SAFETY_TIME(1.25, "KICKER_RESET_TIMEOUT");//power

DoubleConstant GUARD_LEFT_IN_POS(0.289, "KICKER_GUARD_LEFT_IN");// lesser is outward
DoubleConstant GUARD_LEFT_OUT_POS(0.950, "KICKER_GUARD_LEFT_OUT");// lesser is outward

DoubleConstant GUARD_RIGHT_IN_POS(0.753, "KICKER_GUARD_RIGHT_IN");// greater is outward
DoubleConstant GUARD_RIGHT_OUT_POS(0.149, "KICKER_GUARD_RIGHT_OUT");// greater is outward

DoubleConstant OPEN_GUARD_TIME(0.40, "KICKER_OPEN_GUARD_TIME"); // seconds

Kicker::Kicker(Intake &i) :
			intake(i),
			motors(CANID_KICKER_LEFT_FRONT, CANID_KICKER_LEFT_MID,
					CANID_KICKER_LEFT_BACK, CANID_KICKER_RIGHT_FRONT,
					CANID_KICKER_RIGHT_MID, CANID_KICKER_RIGHT_BACK, false),
			kicker_encoder(DIG_KICKER_ENCODER_A, DIG_KICKER_ENCODER_B, true),
			low_flag(DIG_KICKER_LOW_FLAG), high_flag(DIG_KICKER_HIGH_FLAG),
			low_counter(&low_flag), servo_guard_left(PWM_KICKER_GUARD_LEFT),
			servo_guard_right(PWM_KICKER_GUARD_RIGHT) {

	sensorsBroken = false;

	initialize();
}

void Kicker::initialize() {
	kicker_encoder.Stop();
	kicker_encoder.Reset();
	kicker_encoder.Start();
	low_counter.Reset();
	low_counter.Start();
	state = kKickInactive;
	startKickPower = 0.0;
	timer.Reset();
	timer.Start();
	guardsSet = false;
}

void Kicker::process() {
	// do sensors imply that the state should end?
	bool sensor_end;

	switch (state) {
	case kKickInactive:
		turnKicker(0.0);
		setGuard(false);
		break;
	case kKickOpenGuard:
		printf("OPENING GUARDS\n");
		turnKicker(0.0);
		// guardsSet could be true iff the controls previously set it true
		if (timer.Get() > OPEN_GUARD_TIME || guardsSet) {
			printf("Guards satisfactory at %f with guards %c\n", timer.Get(),
					guardsSet ? 'T' : 'F');
			timer.Reset();
			timer.Start();
			state = kKickWaitForIntake;
		}
		setGuard(true);
		break;
	case kKickWaitForIntake:
		printf("WAITING for INTAKE\n");
		turnKicker(0.0);
		setGuard(true);
		if (timer.Get() > INTAKE_WAIT_TIME || intake.isRaised()
				|| sensorsBroken) {
			printf("INTAKE wait was %f\n", timer.Get());
			timer.Reset();
			timer.Start();
			state = kKickSpinning;
		}
		break;
	case kKickSpinning:
		// these two can be broken
		bool d_encoder = kicker_encoder.Get() >= ENCODER_HIGH_LIMIT;

		// guaranteed to work ;-)
		bool d_highflag = getHighFlag();
		bool d_timer = timer.Get() > SAFETY_WAIT_TIME;

		sensor_end = d_encoder;

		if ((sensor_end && !sensorsBroken) || d_highflag || d_timer) {
			timer.Reset();
			timer.Start();
			turnKicker(0.0);
			state = kKickStopping;
			low_counter.Reset();
			low_counter.Start();
			setGuard(false);
			kickPostMortem();
			printf("ENC: %d >= %.0f\n", (int) kicker_encoder.Get(),
					ENCODER_HIGH_LIMIT);
			printf("END: encoder %d; high flag %d; timer %d\n", d_encoder,
					d_highflag, d_timer);
		} else {
			printf("SPINNING %f with intake at %f\n", startKickPower,
					intake.getLocation());
			turnKicker(startKickPower);
			setGuard(true);
		}
		break;
	case kKickStopping:
		turnKicker(0.0);
		setGuard(false);
		if (timer.Get() > STOP_TIME) {
			timer.Reset();
			timer.Start();
			state = kKickResetting;
		}
		break;
	case kKickResetting:
		setGuard(false);

		sensor_end = low_counter.Get() != 0;
		if (timer.Get() > RESET_SAFETY_TIME || (sensor_end && !sensorsBroken)) {
			printf("On reset conclusion: encoder at %d; beam at %c\n",
					(int) kicker_encoder.Get(), sensor_end ? 'T' : 'F');
			turnKicker(0.0); //stop the kicker
			kicker_encoder.Reset();
			kicker_encoder.Start();
			timer.Reset();
			timer.Start();
			state = kKickInactive;
		} else {
			turnKicker(-KICKER_RESET_SPEED);
		}

		break;
	}

	processGuard();
}

void Kicker::debug() {
	LCDWriter d;
	d.line1("encoder: %d", kicker_encoder.Get());
	d.line2("timer %f", timer.Get());
	d.line3("");
	d.line4("low: %d high: %d", getLowFlag(), getHighFlag());
	d.line5("broken: %d", sensorsBroken);
	d.line6("srv L %1.3f R %1.3f", servo_guard_left.Get(),
			servo_guard_right.Get());
}
void Kicker::startKick(double power) {
	printf("START KICK\n");
	startKickPower = power;
	state = kKickOpenGuard;
	timer.Reset();
	timer.Start();
}

void Kicker::setSensorsBroken(bool broke) {
	if (broke) {
		sensorsBroken = true;
	} else {
		sensorsBroken = false;
	}
	state = kKickInactive;
}

void Kicker::turnKicker(double power) {
	motors.Set(power);
}

void Kicker::setGuard(bool state) {
	guardsSet |= state;
}
bool Kicker::nowKicking() {
	switch (state) {
	case kKickInactive:
		return false;
	case kKickOpenGuard:
		return true;
	case kKickWaitForIntake:
		return true;
	case kKickSpinning:
		return true;
	case kKickStopping:
		return false;
	case kKickResetting:
		return false;
	}
	printf("BAD STATE FOR KICKER\n");
	return false;
}
bool Kicker::isReset() {
	return intake.isRaised() && state != kKickResetting;
}
bool Kicker::getLowFlag() {
	return !low_flag.Get();
}
bool Kicker::getHighFlag() {
	SafeCANJag* motor = motors.GetByID(CANID_KICKER_LEFT_BACK);
	if (motor == NULL) {
		printf("kicker left back motor dne\n");
		return true;
	}
	return !motor->GetForwardLimitOK();
}

void Kicker::kickPostMortem() {
	printf("\nKick Postmortem: time %f power %f\n", GetTime(), startKickPower);
	motors.PrintFaults();
	printf("\n");
}

void Kicker::overrideGuards(double left, double right) {
	servo_guard_left.Set(left);
	servo_guard_right.Set(right);
	servoTuning = true;
}

void Kicker::processGuard() {
	if (!servoTuning) {
		servo_guard_left.Set(guardsSet ? GUARD_LEFT_OUT_POS : GUARD_LEFT_IN_POS);
		servo_guard_right.Set(
				guardsSet ? GUARD_RIGHT_OUT_POS : GUARD_RIGHT_IN_POS);
	}
	guardsSet = false;
	servoTuning = false;
}
