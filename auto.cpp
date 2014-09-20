#include "auto.h"
#include "SmartDashboard/SmartDashboard.h"

//
// Constants.
// All distances are in inches; all powers fractions of full power;
// all times are seconds. 
//

DoubleConstant AUTO1_KICKING_POWER(0.93, "AUTO_KICK_POWER");
DoubleConstant AUTO1_DRIVE_MAX(0.60, "AUTO_DRIVE_POWER");
DoubleConstant AUTO1_DISTANCE(132.0, "AUTO_DISTANCE");

DoubleConstant AUTO2_DRIVE_MAX(0.60, "AUTO2_DRIVE_MAX");
DoubleConstant AUTO2_KICKING_POWER(1.00, "AUTO2_KICKING_POWER");

DoubleConstant AUTO2_WAIT_PRE_KICK(1.0, "AUTO2_WAIT_PRE_KICK");
DoubleConstant AUTO2_HOLD_MIN_TIME(1.0, "AUTO2_HOLD_MIN_TIME");

DoubleConstant AUTO2_DRIVE_DISTANCE(36.0, "AUTO2_DRIVE_DISTANCE");
DoubleConstant AUTO2_FORWARD_AGAIN(66.0, "AUTO2_SECOND_FORWARD");
DoubleConstant AUTO2_DRIVE_BACK(66.0, "AUTO2_DRIVE_BACK");

DoubleConstant TEST_DISTANCE(80.0, "AUTO_TEST_DISTANCE");
DoubleConstant TEST_DRIVE_MAX(0.6, "AUTO_TEST_POWER");

DoubleConstant SHORT_KICK_WAIT(0.5, "AUTO_WAIT_PRE_SHORT");

DoubleConstant VISION_START(1.0, "AUTO_VISION_START");

Auto::Auto(Drive &d, Intake &i, Kicker &k, Lights &l) :
	choice(kNone), autoNone(d, i, k, l), autoDouble(d, i, k, l),
			autoSingle(d, i, k, l), autoTest(d, i, k, l) {
}
Auto::~Auto() {
}

AutoBase& Auto::getAuto() {
	switch (choice) {
	case kAutoA:
		return autoSingle;
	case kAutoB:
		return autoDouble;
	case kAutoTest:
		return autoTest;
	default:
		printf("Erronous auto choice. Choosing nothing instead\n");
	case kNone:
		return autoNone;
	}
}
void Auto::initialize(AutoRoutine x) {
	choice = x;
	printf("AUTO choice %d\n", x);

	AutoBase &au = getAuto();
	au.setup();
	au.init();
}

void Auto::process() {
	getAuto().process();
}

AutoBase::AutoBase(Drive &d, Intake &i, Kicker &k, Lights &l) :
	drive(d), intake(i), kicker(k), lights(l) {
	match_timer.Start();
	state_timer.Start();
}
AutoBase::~AutoBase() {
}
void AutoBase::setup() {
	match_timer.Reset();
}
int AutoBase::getState() {
	return state;
}
void AutoBase::nextState(int next) {
	state = next;
	state_timer.Reset();
}
double AutoBase::getStateTime() {
	return state_timer.Get();
}
double AutoBase::getMatchTime() {
	return match_timer.Get();
}
void AutoBase::startVisionProcessing() {
	SmartDashboard::PutBoolean("process_img", true);
}
bool AutoBase::isProcessingDone() {
	bool done = false;
	try {
		done = !SmartDashboard::GetBoolean("process_img");
	} catch (std::exception e) {
		printf("Failed on SmartDashboard procimg return\n");
	}
	return done;

}
bool AutoBase::isGoalHot() {
	bool hot = true;
	try {
		hot = SmartDashboard::GetBoolean("is_hot_goal");
	} catch (std::exception e) {
		printf("Failed on SmartDashboard hotgoal return\n");
	}
	return hot;
}

AutoNearSingleBall::AutoNearSingleBall(Drive &d, Intake &i, Kicker &k,
		Lights &l) :
	AutoBase(d, i, k, l) {
}
void AutoNearSingleBall::init() {
	vision_yet = false;
	nextState(sInit);
}
void AutoNearSingleBall::process() {
	if (getMatchTime() > VISION_START && !vision_yet) {
		printf("Vision start\n");
		startVisionProcessing();
		vision_yet = true;
	}

	switch (getState()) {
	case sInit:
		printf("AUTO: init\n");
		intake.goToLocation(Intake::kLocAutoKick);
		drive.autonDrive(AUTO1_DISTANCE, AUTO1_DRIVE_MAX);//move forward 2 feet
		lights.setCameraLight(true);
		nextState(sWaitForIntakeUp);
		break;
	case sWaitForIntakeUp:
		printf("AUTO: wait for raise\n");
		if (intake.isRaised()) {
			nextState(sWaitForDriveForward);
		}
		break;
	case sWaitForDriveForward:
		printf("AUTO: wait for drive\n");
		if (drive.autoDone()) {//check if in position
			//stop moving(done by drive code)
			nextState(sInterpretSensor);
		}
		break;
	case sInterpretSensor:
		printf("AUTO: wait for sensor\n");
		if ((vision_yet && isProcessingDone()) || getMatchTime() > 5.0) {
			lights.setCameraLight(false);
			if (isGoalHot()) {
				printf("AUTO: goal was hot\n");
				nextState(sWaitForEarlyShoot);
			} else {
				printf("AUTO: goal not hot\n");
				nextState(sWaitForHalfMark);
			}
		}
		break;
	case sWaitForEarlyShoot:
		printf("AUTO: waiting for early shoot %f\n", getMatchTime());
		if (getStateTime() > SHORT_KICK_WAIT || getMatchTime() >= 5.0) {
			kicker.startKick(AUTO1_KICKING_POWER);
			nextState(sWaitForKick);
		}
		break;
	case sWaitForHalfMark:
		printf("AUTO: waiting for half mark %f\n", getMatchTime());
		if (getMatchTime() >= 5.0) {
			kicker.startKick(AUTO1_KICKING_POWER);
			nextState(sWaitForKick);
		}
		break;
	case sWaitForKick:
		printf("AUTO: waiting to stop kicking\n");
		if (!kicker.nowKicking()) {//wait for kick to finish
			nextState(sEndAuto);
			printf("AUTO: Done %f\n", getMatchTime());
		}
		break;
	default:
	case sEndAuto:
		printf("AUTO: completed\n");
		break;
	}
}
AutoDoubleBall::AutoDoubleBall(Drive &d, Intake &i, Kicker &k, Lights &l) :
	AutoBase(d, i, k, l) {

}
void AutoDoubleBall::init() {
	nextState(sInit);
}
void AutoDoubleBall::process() {
	printf("AUTO2: %d; %f\n", getState(), getMatchTime());
	switch (getState()) {
	case sInit:
		intake.goToLocation(Intake::kLocAutoKick);
		drive.autonDrive(AUTO2_DRIVE_DISTANCE, AUTO2_DRIVE_MAX);//move forward 2 feet
		nextState(sRaiseIntake);
		break;
	case sRaiseIntake:
		printf("AUTO: wait for raise\n");
		if (intake.isRaised()) {
			nextState(sDriveForward1);
		}
		break;
	case sDriveForward1:
		printf("AUTO: wait for drive\n");
		if (drive.autoDone()) {
			nextState(sPausePreKick1);
		}
		break;
	case sPausePreKick1:
		if (getStateTime() > AUTO2_WAIT_PRE_KICK) {
			kicker.startKick(AUTO2_KICKING_POWER);
			nextState(sKick1);
		}
		break;
	case sKick1:
		if (!kicker.nowKicking()) {
			intake.goToLocation(Intake::kLocFeed);
			drive.autonDrive(-AUTO2_DRIVE_BACK, AUTO2_DRIVE_MAX);
			intake.spin(Intake::kSpinIn);
			nextState(sWaitDriveIntake);
		}
		break;
	case sWaitDriveIntake:
		if (intake.isLowered()) {
			nextState(sWaitDriveBack);
		}
		break;
	case sWaitDriveBack:
		if (drive.autoDone()) {
			nextState(sWaitForPossession);
		}
		break;
	case sWaitForPossession:
		if (getStateTime() > AUTO2_HOLD_MIN_TIME) {
			nextState(sRaiseBall);
			intake.goToLocation(Intake::kLocAutoKick);
		}
		break;
	case sRaiseBall:
		if (intake.isRaised()) {
			intake.spin(Intake::kSpinNot);
			drive.autonDrive(AUTO2_FORWARD_AGAIN, AUTO2_DRIVE_MAX);
			nextState(sDriveForward2);
		}
		break;
	case sDriveForward2:
		if (drive.autoDone()) {
			nextState(sPausePreKick2);
		}
		break;
	case sPausePreKick2:
		if (getStateTime() > AUTO2_WAIT_PRE_KICK) {
			nextState(sKick2);
			kicker.startKick(AUTO2_KICKING_POWER);
		}
		break;
	case sKick2:
		if (!kicker.nowKicking()) {
			nextState(sEndAuto);
		}
		break;
	default:
	case sEndAuto:
		break;
	}
}

AutoNone::AutoNone(Drive &d, Intake &i, Kicker &k, Lights &l) :
	AutoBase(d, i, k, l) {
}
void AutoNone::init() {
}
void AutoNone::process() {
}

AutoTest::AutoTest(Drive &d, Intake &i, Kicker &k, Lights &l) :
	AutoBase(d, i, k, l) {
}
void AutoTest::init() {
	nextState(sInit);
}
void AutoTest::process() {
	switch (getState()) {
	case sInit:
		drive.autonDrive(TEST_DISTANCE, TEST_DRIVE_MAX);
		nextState(sDrive);
		break;
	case sDrive:
		break;
	default:
	case sDone:
		break;
	}
}

