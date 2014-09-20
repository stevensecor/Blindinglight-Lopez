#ifndef AUTO_H_
#define AUTO_H_

#include "util.h"
#include "drive.h"
#include "intake.h"
#include "kicker.h"
#include "lights.h"

/**
 * Each autonomous routine is representated as a simple state machine
 * associated with a class. All of these state machines have access to
 * helper functions and timers, as well as to all the base-level subsystems.
 * 
 * The intent is that the init() method sets starting state variables and the process method
 * updates the state machine and robot functions.
 * 
 * In autonomous, the Auto class recieves a target autonomous on initialization.
 * That autonomous routine will be initialized and run. It will not coexist with 
 * the controls class: only autonomous will command the systems.
 */

class AutoBase {
public:
	AutoBase(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual ~AutoBase();
	/**
	 * Set the timers for the overlying autonomous mode. 
	 */
	void setup();
	
	/**
	 * Override these functions. init() will be called at the
	 * start of the autonomous period. process() will be called
	 * periodically afterward.
	 * 
	 * Typically, in init() one calls nextState to key the 
	 * state machine; then in process() the state transitions
	 * and side effects (robot actions) are handled.
	 */
	virtual void init() = 0;
	virtual void process() = 0;
protected:
	/**
	 * Get the state machine state
	 */
	int getState();
	/**
	 * Set the state machine state and start the state duration timer
	 */
	void nextState(int);

	/**
	 * Returns time since the last state transition; useful for state transition timeouts. 
	 */
	double getStateTime();
	/**
	 * Returns time since start of autonomous
	 */
	double getMatchTime();

	Drive &drive;
	Intake &intake;
	Kicker &kicker;
	Lights &lights;

	/**
	 * Begins the vision processing sequence
	 */
	void startVisionProcessing();
	/**
	 * Returns true as soon as a response is available.
	 * If startVisionProcessing has not been called, may return anything
	 */
	bool isProcessingDone();
	/**
	 * Returns whether the goal is hot as of startVisionProcessing
	 * having been called. Only gives a good value after isProcessingDone returns true.
	 */
	bool isGoalHot();
private:
	DISALLOW_COPY_AND_ASSIGN(AutoBase);
	
	Timer match_timer;
	Timer state_timer;
	int state;
};

/**
 * Single ball hot high goal autonomous.
 */
class AutoNearSingleBall: public AutoBase {
public:
	AutoNearSingleBall(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual void init();
	virtual void process();
private:
	bool vision_yet;
	enum {
		sInit,
		sWaitForIntakeUp,
		sWaitForDriveForward,
		sInterpretSensor,
		sWaitForEarlyShoot,
		sWaitForHalfMark,
		sWaitForKick,
		sEndAuto
	};
};

/**
 * Double ball high goal autonomous.
 * The robot starts with a ball and drives forward. While driving,
 * it kicks the ball into the high goal (to get enough range).
 * It backs up, pulls in a second ball, and then kicks again.
 * 
 * Has not been well tested.
 */
class AutoDoubleBall: public AutoBase {
public:
	AutoDoubleBall(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual void init();
	virtual void process();
private:
	enum {
		sInit,
		sRaiseIntake,
		sDriveForward1,
		sPausePreKick1,
		sKick1,
		sWaitDriveIntake,
		sWaitDriveBack,
		sWaitForPossession,
		sRaiseBall,
		sDriveForward2,
		sPausePreKick2,
		sKick2,
		sEndAuto
	};
};

/**
 * Placeholder autonomous routine, when no other routines are selected.
 */
class AutoNone: public AutoBase {
public:
	AutoNone(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual void init();
	virtual void process();
};

/**
 * Experimental autonomous; subject to rapid change.
 */
class AutoTest: public AutoBase {
public:
	AutoTest(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual void init();
	virtual void process();
private:
	enum {
		sInit,
		sDrive,
		sDone
	};
};

/**
 * Selector and wrapper for the actual autonomous routines.
 */
class Auto {
public:
	typedef enum {
		kNone, kAutoA, kAutoB, kAutoTest
	} AutoRoutine;

	Auto(Drive &d, Intake &i, Kicker &k, Lights &l);
	virtual ~Auto();
	/**
	 * Reset state at start of a mode.
	 * The chosen auto routine is set.
	 */
	void initialize(AutoRoutine a);
	/**
	 * Update state every cycle.
	 */
	void process();
private:
	AutoRoutine choice;
	AutoBase& getAuto();

	AutoNone autoNone;
	AutoDoubleBall autoDouble;
	AutoNearSingleBall autoSingle;
	AutoTest autoTest;
};

#endif
