#include "drive.h"
#include "intake.h"
#include "kicker.h"
#include "lights.h"
#include "auto.h"
#include "controls.h"
#include "util.h"
#include "SimpleRobot.h"

/**
 * The robot.
 * 
 * It is divided into three groups of systems:
 * - the Drive, Intake, and Kicker, the functional
 *   portions of the robot
 * - the Lights, a non-mechanically functional system
 * - the Controls and Auto classes, which control
 *   (at a high level) the robot when it is disabled
 *   or in teleop, or in autonomous, respectively.
 * 
 * One operations is very expensive, and is performed before
 * the robot is constructed. (As soon as RobotBase is constructed,
 * the DriverStation class is instantiated; this communicates with
 * the laptop, calling FRC_NetworkCommunication_observeUserProgramDisabled,
 * at which point the robot can be enabled although 
 * 
 * Creating a CAN device: the first call to read takes almost exactly 2.005 seconds,
 *    probably due o authentification protocol and setup details.
 */
class Yolo: public SimpleRobot {
private:
	class PreInitCalls {
	public:
		PreInitCalls() {
			time = GetTime();
			printf("RobotBase UP: %f\n", time);
		}
		double time;
	} pre_construct_;
	Drive drive;
	Intake intake;
	Kicker kicker;
	Lights lights;
	Auto autosel;
	Controls controls;
public:
	const static double ROBOT_PERIOD = 0.050;

	Yolo() :
		pre_construct_(), drive(), intake(), kicker(intake),
				lights(drive, intake, kicker),
				autosel(drive, intake, kicker, lights),
				controls(drive, intake, kicker, lights) {
		printf("Yolo Took %f to create\n", GetTime() - pre_construct_.time);
		printf("Battery Voltage: %2.4f\n", GetBatteryVoltage());
	}

	~Yolo() {
		UDPLog::destroy();
	}

	/**
	 * Should be called at the start of each mode
	 */
	void enterMode(const char* name) {
		// update the semi-fixed constants
		last_time = GetTime();
		Constants::reload();
		printf("\n\n\t\t%s\n\n", name);
	}

	void waitUntilNextPeriod() {
		double newtime = GetTime();
		double delta = newtime - last_time;
		UDPLog::log("Loop:%f\n", delta);
		Wait(0.005);
		double leftover = ROBOT_PERIOD - 0.005 - delta;
		if (leftover > 0) {
			Wait(leftover);
		}
		last_time = GetTime();
	}

	void RobotInit() {
		Constants::load();
		printf("\n\n\t\tROBOT INITIALIZED\n\n");
	}

	void Disabled() {
		enterMode("DISABLED");
		controls.initialize(false);
		lights.initialize();
		drive.measureGyro();
		while (IsDisabled()) {
			controls.process(false);
			lights.process();
			waitUntilNextPeriod();
		}
		drive.finalizeGyro();
	}

	void Autonomous() {
		enterMode("AUTONOMOUS");
		autosel.initialize(controls.getAutoMode());
		lights.initialize();
		intake.initialize();
		kicker.initialize();
		drive.initialize();
		while (IsAutonomous() && IsEnabled()) {
			autosel.process();
			lights.process();
			intake.process();
			kicker.process();
			drive.process();
			waitUntilNextPeriod();
		}
	}

	void OperatorControl() {
		enterMode("TELEOP");
		controls.initialize(true);
		lights.initialize();
		intake.initialize();
		kicker.initialize();
		drive.initialize();
		while (IsOperatorControl() && IsEnabled()) {
			controls.process(true);
			intake.process();
			kicker.process();
			drive.process();
			lights.process();
			waitUntilNextPeriod();
		}
	}

	void Test() {
		enterMode("TEST");
	}
private:
	double last_time;

};

RobotBase *FRC_userClassFactory() {
	//
	// Initialize the UDP log facility.
	//
	UDPLog::setup();
	//
	// The first CANBus message takes roughly 2 seconds.
	// The call UpdateSyncGroup(0) does nothing because:
	//   No jaguars have been commanded with a sync group.
	//   Sync groups mask in strange ways. 
	//
	SafeCANJag::UpdateSyncGroup(0);
	return new Yolo();
}
extern "C" {
int32_t FRC_UserProgram_StartupLibraryInit() {
	RobotBase::startRobotTask((FUNCPTR) FRC_userClassFactory);
	return 0;
}
}
