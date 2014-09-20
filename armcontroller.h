#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include "util.h"
#include "Timer.h"

class BumpController {
public:
	double calc(double target, double loc,double);
	void reset(double loc);
private:
	TimeDifferentiator diff;
	TimeIntegrator integ;
	double boost;
	bool stalled;
};

class NaiveController {
public:
	double calc(double target, double loc,double);
	void reset(double loc);
private:
	TimeDifferentiator vdiff;
	TimeDifferentiator ediff;
	TimeIntegrator integ;
	bool close;
	double boost;
};

/**
 * Model predictive controller for the intake arm.
 * The controller performs two actions: it measures the net external force
 * acting on the intake, and calculates an output command that comes closest
 * to attaining a defined relationship between positional error and velocity.
 * 
 * Both measurement and control optimize a single scalar parameter over a range
 * of possiblities. For the measurement phase, an F-value is chosen for which
 * a simulation of the past behavior most closely approximates the actual past
 * behavior. The control phase optimizes the proximity to a target trajectory
 * over a short future time interval.
 * 
 * Offline model based testing implies that the control phase is much more
 * effective than the measurement phase due to noise/scale.
 * 
 */
class ModelController {
public:
	ModelController();
	double calc(double target, double loc, double last_out);
	void reset(double loc);
private:
	typedef struct {
		double x;
		double v;
	} State;

	void log_hist(double output);
	double target_speed(double x);

	static double eval_F(ModelController*, double F);
	static double eval_pow(ModelController*, double spd);
	static State predict(State s, double out, double F);

	Timer timer;
	MovingAverageFilter filtF;
	RingBuffer<double> hist_v;
	RingBuffer<double> hist_x;
	RingBuffer<double> hist_out;
	MovingAverageFilter output_smooth;
	double estF;

	double target_x;
	State current;
};

class LimitController {
public:
	double calc(double target, double loc, double last_out, bool over);
	void reset(double loc, bool over);
private:
	TimeDifferentiator derr;
	TimeIntegrator ierr;
};

#endif
