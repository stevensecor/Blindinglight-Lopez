#include "intake.h"
#include "iomap.h"
#include "math.h"

DoubleConstant HIGH_LIMIT(1.33, "INTAKE_POT_HIGH"); // volts; where we kick at
DoubleConstant LOW_LIMIT(2.34, "INTAKE_POT_LOW"); // volts

DoubleConstant ALT_HIGH_LIM(2.138, "UNSLOP_POT_HIGH");
DoubleConstant ALT_LOW_LIM(2.7949, "UNSLOP_POT_LOW");

DoubleConstant SAFE_PERCENT(0.77, "INTAKE_LOC_SAFE_LEVEL");

// best these the same
DoubleConstant KICK_POSITION(0.82, "INTAKE_LOC_KICK");
DoubleConstant KICK_POS_TELE(0.82, "INTAKE_LOC_TELEKICK");

DoubleConstant ARC_KICK_POSITION(0.85, "INTAKE_LOC_ARC");
DoubleConstant HIGH_POWER_STOP(0.85, "INTAKE_LOC_POWER_STOP");

DoubleConstant SCALE_POSITION_CONTROL(1.0, "INTAKE_SCALE_POSITION");
DoubleConstant SCALE_BROKEN_POWER_CONTROL(0.50, "INTAKE_SCALE_BROKEN");

DoubleConstant SPIN_IN_POWER(1.0, "INTAKE_SPIN_POWER_IN");
DoubleConstant SPIN_OUT_POWER(1.0, "INTAKE_SPIN_POWER_OUT");
DoubleConstant SPIN_OUT_SLOW_POWER(0.69, "INTAKE_SPIN_POWER_OUT_SLOW");

DoubleConstant PFIELD_DOWN_SPLIT(0.25, "INTAKE_PF_DOWN_SPLIT");
DoubleConstant PFIELD_DOWN_NEAR(1.0, "INTAKE_PF_DOWN_NEAR");
DoubleConstant PFIELD_DOWN_FAR(1.0, "INTAKE_PF_DOWN_FAR");

DoubleConstant PFIELD_UP_SPLIT(0.3, "INTAKE_PF_UP_SPLIT");
DoubleConstant PFIELD_UP_NEAR(0.375, "INTAKE_PF_UP_NEAR");
DoubleConstant PFIELD_UP_FAR(0.75, "INTAKE_PF_UP_FAR");

DoubleConstant POS_OVER(0.03, "INTAKE_POS_OVER");

IntakePot::IntakePot(uint8_t chan, Intake &i) :
	filter(10), noti(IntakePot::callUpdate, this), pot(chan), intake(i) {
	// configure potentiometer oversampling/avg bit counts.
	// each averages 2 ** n values, but averaging rounds
	// off the partial amounts, while oversampling
	// yields full precision.
	//
	// Each value sample takes 0.1666 ms; 2 average, 3 oversample
	// bits imply 2 ** (2 + 3) = 32 samples, so 5 ms. Robot loop
	// time is ~50 ms.
	watching = false;
	semaphore = semMCreate(SEM_Q_PRIORITY);
	noti.StartPeriodic(kPeriod);
	pot.SetOversampleBits(3);
	pot.SetAverageBits(2);
}

void IntakePot::StartWatching() {
	{
		Synchronized sync(semaphore);
		watching = true;
	}
}

void IntakePot::StopWatching() {
	{
		Synchronized sync(semaphore);
		watching = false;
	}
}

void IntakePot::callUpdate(void* x) {
	((IntakePot*) x)->update();
}

void IntakePot::update() {
	bool wt;
	{
		Synchronized sync(semaphore);
		filter.calc(pot.GetVoltage());
		wt = watching;
	}

	if (wt) {
		double loc = getLocation();
		double pw = intake.getLastLiftOutput();
		double tg = intake.getTargetPos();
		double time = GetTime();
		UDPLog::log("LPT:%f,%f,%f,%f\n", loc, pw, time, tg);
	}
}

double IntakePot::getLocation() {
	return scaleLinear(getVoltage(), LOW_LIMIT, HIGH_LIMIT, 0.0, 1.0);
}
double IntakePot::getVoltage() {
	double v;
	{
		Synchronized sync(semaphore);
		v = filter.recalc();
	}
	return v;
}

Intake::Intake() :
	motor_roller(CANID_INTAKE_ROLLER, true),
			motor_lift(CANID_INTAKE_LIFT, true),
			pot(ANALOG_INTAKE_POT_1, *this),
			altpot(ANALOG_INTAKE_ALT_POT, *this),
			beam_break(DIG_INTAKE_BALL_SENSOR) {

	initialize();
	use_alt = false;
}

void Intake::initialize() {
	target_power = 0.0;
	moveToPosition(getLocation());
	direction = kSpinNot;
	mode = kPower;
	pos_controller.reset(getLocation());
	alt_controller.reset(getLocation());
	target_pos = KICK_POSITION;
}

void Intake::process() {
	switch (mode) {
	case kPosition:
		if (potbroke) {
			setLiftDirect(0.0);
		} else {
			double loc = getLocation();
			double pow = pos_controller.calc(target_pos, loc, last_output);
			if (use_alt) {
				alt_controller.calc(target_pos, loc, last_output);
			}
			if (loc > 1.0 && pow > 0.0) {
				pow = 0.0;
			} else if (loc < 0.0 && pow < 0.0) {
				pow = 0.0;
			}
			setLiftDirect(-SCALE_POSITION_CONTROL * pow);
		}
		break;
	case kPower:
		if (potbroke) {
			setLiftDirect(target_power * SCALE_BROKEN_POWER_CONTROL);
		} else {
			double loc = getLocation();
			double power = target_power;
			if (target_power < 0) { // up
				double dist = HIGH_POWER_STOP - loc;
				if (dist <= 0.0) {
					power = 0;
				} else if (dist < PFIELD_UP_SPLIT) {
					power *= PFIELD_UP_NEAR;
				} else {
					power *= PFIELD_UP_FAR;
				}
			} else { // down
				double dist = loc;
				if (dist <= 0.0) {
					power = 0.0;
				} else if (dist < PFIELD_DOWN_SPLIT) {
					power *= PFIELD_DOWN_NEAR;
				} else {
					power *= PFIELD_DOWN_FAR;
				}
			}
			setLiftDirect(power);
		}
		break;
	}

	switch (direction) {
	case kSpinCustomIn:
		printf("Custom in: %f\n", custom_spin_speed);
		motor_roller.Set(custom_spin_speed);
		break;
	case kSpinCustomOut:
		printf("Custom out: %f\n", custom_spin_speed);
		motor_roller.Set(-custom_spin_speed);
		break;
	case kSpinIn:
		motor_roller.Set(SPIN_IN_POWER);
		break;
	case kSpinOutSlow:
		motor_roller.Set(-SPIN_OUT_SLOW_POWER);
		break;
	case kSpinOut:
		motor_roller.Set(-SPIN_OUT_POWER);
		break;
	default:
	case kSpinNot:
		motor_roller.Set(0.0);
		break;
	}
}

void Intake::debug() {
	LCDWriter d;
	d.line1("V: %6.5f L: %4.3f", pot.getVoltage(), getLocation());
	d.line2("V: %6.5f L: %4.3f", altpot.getVoltage(), getUnsloppedLocation());
	d.line3("high %d low %d broke %d", isRaised(), isLowered(), potbroke);
	d.line4("mode %d ball %d", mode, isBallPresent());
	d.line5("lft %05.2f rol %05.2f", motor_lift.GetIdx(0)->GetOutputCurrent(),
			motor_roller.GetIdx(0)->GetOutputCurrent());
	d.line6("tg: %3.2f sp %d out %05.4f",
			mode == kPower ? target_power : target_pos, direction, last_output);
}

void Intake::spin(SpinMode spin) {
	direction = spin;
}

void Intake::moveArm(double down_power) {
	target_power = down_power;
	mode = kPower;
}
void Intake::goToLocation(Position p) {
	switch (p) {
	case kLocArcKick:
		moveToPosition(ARC_KICK_POSITION);
		return;
	case kLocFeed:
		moveToPosition(0.0);
		return;
	case kLocAutoKick:
		moveToPosition(KICK_POSITION);
		return;
	case kLocTeleopKick:
		moveToPosition(KICK_POS_TELE);
		return;
	default:
		printf("ERROR: Not a position\n");
		return;
	}
}
void Intake::moveToPosition(double pos) {
	target_pos = bound(pos, 0.0, 1.0);
	if (mode != kPosition) {
		pos_controller.reset(getLocation());
		alt_controller.reset(getLocation());
	}
	mode = kPosition;
}

bool Intake::isRaised() {
	double loc = getLocation();
	return (loc > SAFE_PERCENT);
}
bool Intake::isLowered() {
	return getLocation() < 0.2;
}
void Intake::setPotBroken(bool broke) {
	potbroke = broke;
}

bool Intake::isBallPresent() {
	return beam_break.Get();
}

double Intake::getLocation() {
	return pot.getLocation();
}

double Intake::getUnsloppedLocation() {
	return scaleLinear(altpot.getVoltage(), ALT_LOW_LIM, ALT_HIGH_LIM, 0.0, 1.0);
}

double Intake::getTargetPos() {
	return target_pos;
}

void Intake::setLiftDirect(double v) {
	motor_lift.Set(v);
	last_output = v;
}
double Intake::getLastLiftOutput() {
	return last_output;
}

bool Intake::isPosOver() {
	return getLocation() > target_pos + POS_OVER;
}

void Intake::setCustomSpinSpeed(double d) {
	custom_spin_speed = d;
}

void Intake::setModelController(bool alt) {
	use_alt = alt;
	if (alt) {
		pot.StartWatching();
	} else {
		pot.StopWatching();
	}
}

void Intake::useAlternatePot(bool yes_p) {

}
