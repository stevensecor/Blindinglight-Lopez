#include "armcontroller.h"
#include "math.h"

// In this realm, positive is UP.

DoubleConstant FIELD_EXTF_LOW_FORCE(0.10, "INTAKE_EXTF_LOW_FORCE");
DoubleConstant FIELD_EXTF_LOW_ZERO_PT(0.50, "INTAKE_EXTF_LOW_ZERO_PT");
DoubleConstant FIELD_EXTF_HIGH_ZERO_PT(0.65, "INTAKE_EXTF_HIGH_ZERO_PT");
DoubleConstant FIELD_EXTF_HIGH_FORCE(-0.07, "INTAKE_EXTF_HIGH_FORCE");

/**
 * Amount to be added to the output to compensate for the
 * external forces on the intake.
 */
double getIntForceField(double loc) {
	typedef struct {
		double loc;
		double extforce;
	} LcPw;
	// Note: force field must be sorted by loc
	LcPw FORCE_FIELD[] = { { 0.0, FIELD_EXTF_LOW_FORCE }, {
			FIELD_EXTF_LOW_ZERO_PT, 0.0 }, { FIELD_EXTF_HIGH_ZERO_PT, 0.0 }, {
			1.0, FIELD_EXTF_HIGH_FORCE } };

	int len = sizeof(FORCE_FIELD) / sizeof(LcPw);
	double extf;
	int i = 1;
	for (; i < len; i++) {
		if (loc <= FORCE_FIELD[i].loc) {
			extf = scaleLinear(loc, FORCE_FIELD[i - 1].loc, FORCE_FIELD[i].loc,
					FORCE_FIELD[i - 1].extforce, FORCE_FIELD[i].extforce);
			break;
		}
	}
	if (loc > FORCE_FIELD[i].loc) {
		i = len - 1;
		extf = scaleLinear(loc, FORCE_FIELD[i - 1].loc, FORCE_FIELD[i].loc,
				FORCE_FIELD[i - 1].extforce, FORCE_FIELD[i].extforce);
	}

	return extf;
}

DoubleConstant UNHAPPY_SPEED_ZONE(0.01, "INTAKE_BOOST_ZONE_UNHAPPY");
DoubleConstant MEH_SPEED_ZONE(0.03, "INTAKE_BOOST_ZONE_MEH");
DoubleConstant BOOST(2.0, "INTAKE_BOOST_INC");
DoubleConstant PVALUE(2.0, "INTAKE_BOOST_P");
DoubleConstant IVALUE(0.2, "INTAKE_BOOST_I");

void BumpController::reset(double loc) {
	diff.reset(loc);
	boost = 0.0;
	stalled = false;
	integ.reset(0);
}
double BumpController::calc(double target, double loc, double) {
	double spd = diff.calc(loc);
	if (fabs(spd) < UNHAPPY_SPEED_ZONE) {
		stalled = true;
		boost += BOOST * diff.lastTimeStep();
	} else if (fabs(spd) < MEH_SPEED_ZONE) {
		// don't change boost
		if (stalled) {

		}
	} else {
		boost = 0.0;
		stalled = false;
	}

	double target_spd;
	double err = target - loc;
	if (fabs(err) < 0.01) {
		target_spd = 0.0;
		stalled = false;
		boost = 0.0;
	} else {

		target_spd = PVALUE * err + IVALUE * integ.calc(err);
	}

//	} else if (dist < 0) {
//		double fd = fabs(dist);
//		target_spd = -1.0;
//		if (fd < 0.5) {
//			target_spd *= fd;
//		}
//	} else {
//		double fd = fabs(dist);
//		target_spd = 1.0;
//		if (fd < 0.5) {
//			target_spd *= fd;
//		}
//	}

	double sign = (target_spd > 0) ? 1.0 : -1.0;
	double pow = target_spd + boost * sign + getIntForceField(loc);
	return bound(pow, -1.0, 1.0);
}

DoubleConstant PID_P(1.5, "INTAKE_PID_P");
DoubleConstant PID_I(2.0, "INTAKE_PID_I");
DoubleConstant PID_D(0.0, "INTAKE_PID_D");

DoubleConstant PID_GROWTH(1.0, "INTAKE_PID_GROWTH"); // unitless

void NaiveController::reset(double loc) {
	vdiff.reset(loc);
	ediff.reset(0);
	integ.reset(0);
}

double NaiveController::calc(double target, double loc, double) {
	double vel = vdiff.calc(loc);
	double err = target - loc;
	double derr = ediff.calc(err);

	// Integral growth during the P rise is bad.
	// Integral growth in steady state is good.
	// Capped integrals are bad, unless well defined via output value, and still delay things.
	// 
	//
	double ierr;
	//	printf("magic: vp: %f, err %f\n", fabs(vel) * PID_GROWTH, err);
	if (fabs(vel) * PID_GROWTH < 1.0) {
		ierr = integ.calc(err);
	} else {
		ierr = integ.calc((PID_GROWTH / vel) * err);
	}

	double out = err * PID_P + ierr * PID_I + derr * PID_D;

	return out + getIntForceField(loc);
}

const int F_SMOOTH_SAMPLES = 1;
const int CONVEX_SEEK_GENERATIONS = 18;
const int MONITOR_HISTORY_LENGTH = 5;
const int CONTROL_LOOKAHEAD_LENGTH = 3;
const int OUTPUT_SMOOTH_SAMPLES = 1;
DoubleConstant PREDICTION_STEP(0.050, "INTAKE_AC_TIMESTEP");
DoubleConstant OUT_SCALE(1.0, "INTAKE_AC_OUT_SCALE");
DoubleConstant STALL_SPEED(0.01, "INTAKE_AC_STALL_SPD");
DoubleConstant FRIC_STATIC(0.2, "INTAKE_AC_FRIC_STATIC");
DoubleConstant FRIC_DYNAMIC(0.1, "INTAKE_AC_FRIC_DYNAMIC");
DoubleConstant K_PROP(2.5, "INTAKE_AC_PROPK");
DoubleConstant START_F(0.0, "INTAKE_AC_F_START");

double seek_convex(ModelController* inst, double min, double max,
		double(*thunk)(ModelController*, double)) {
	//
	// Current cost: 36 calls, eqv. step 0.0012
	// the best way to speed up the controller is to optimize this
	//
	double score_max = (*thunk)(inst, max);
	double score_min = (*thunk)(inst, min);
	for (int gen = 0; gen < CONVEX_SEEK_GENERATIONS; gen++) {
		double low = (min * 2 + max) * 0.333333333333333333333333;
		double high = (max * 2 + min) * 0.333333333333333333333333;
		double score_high = (*thunk)(inst, high);
		double score_low = (*thunk)(inst, low);

		if (score_high >= score_low) {
			max = high;
			score_max = score_high;
		}
		if (score_low >= score_high) {
			min = low;
			score_min = score_low;
		}
	}
	return (max + min) * 0.5;
}

ModelController::ModelController() :
	filtF(F_SMOOTH_SAMPLES), hist_v(MONITOR_HISTORY_LENGTH),
			hist_x(MONITOR_HISTORY_LENGTH), hist_out(MONITOR_HISTORY_LENGTH),
			output_smooth(OUTPUT_SMOOTH_SAMPLES) {
	timer.Start();
	timer.Reset();
}

void ModelController::reset(double loc) {
	estF = 0;
	target_x = 0;
	State s = { loc, 0 };
	current = s;

	filtF.reset(START_F);
	hist_v.clear();
	hist_x.clear();
	hist_out.clear();
	output_smooth.reset(0);
}

void ModelController::log_hist(double output) {
	hist_v.next(current.v);
	hist_x.next(current.x);
	hist_out.next(output);
}

double ModelController::calc(double target, double loc, double last_out) {
	log_hist(last_out);

	double timestep = timer.Get();
	timer.Reset();
	target_x = target;
	State s = { loc, (loc - current.x) / timestep };
	current = s;

	double newF = seek_convex(this, -1.0, 1.0, eval_F);
	estF = filtF.calc(newF);

	double eout = seek_convex(this, -1.0, 1.0, eval_pow);
	double output = output_smooth.calc(eout);
	printf("x %f v %f ==> out %f; force %f\n", current.x, current.v, output,
			estF);
	UDPLog::log("GG:%f,%f,%f,%f\n", current.x, current.v, output, estF);

	return output;
}

double ModelController::eval_F(ModelController* ctr, double F) {
	State state = { ctr->hist_x[0], ctr->hist_v[0] };
	double cost = 0.0;
	int len = ctr->hist_x.size();
	for (int i = 0; i < len - 1; i++) {
		double out = ctr->hist_out[i];
		state = predict(state, out, F);
		double err = (state.x - ctr->hist_x[i + 1]);
		cost += err * err;
	}
	state = predict(state, ctr->hist_out[len - 1], F);
	double err = (state.x - ctr->current.x);
	cost += err * err;
	return cost;
}

double ModelController::eval_pow(ModelController* ctr, double pow) {
	double cost = 0;
	State state = ctr->current;
	for (int i = 0; i < CONTROL_LOOKAHEAD_LENGTH; i++) {
		state = predict(state, pow, ctr->estF);
		double err = ctr->target_speed(state.x) - state.v;
		cost += err * err;
	}
	return cost;
}

double ModelController::target_speed(double loc) {
	return (target_x - loc) * K_PROP;
}

inline bool within(double x, double radius) {
	return (x > -radius) && (x < radius);
}

ModelController::State ModelController::predict(State state, double output,
		double F) {
	double force = (output + F) * OUT_SCALE;
	if (within(state.v, STALL_SPEED) && within(force, FRIC_STATIC)) {
		State res = { state.x, 0 };
		return res;
	}

	double fdyn = state.v > 0 ? FRIC_DYNAMIC : -FRIC_DYNAMIC;
	double acc = force - fdyn;

	// note: it might improve results to match the prediction step
	// size to the historical timestep for the measurement phase
	double v = state.v + acc * PREDICTION_STEP;
	double x = state.x + v * PREDICTION_STEP;

	if (x > 1.0) {
		State res = { 1.0, 0.0 };
		return res;
	} else if (x < 0.0) {
		State res = { 0.0, 0.0 };
		return res;
	}

	State res = { x, v };
	return res;
}

DoubleConstant LIMC_RADIUS(0.025, "INTAKE_LIMC_BACKRAD");
DoubleConstant LIMC_FIELD_RAD(0.050, "INTAKE_LIMC_FLDRAD");
DoubleConstant LIMC_FIELD_POWER(0.20, "INTAKE_LIMC_FIELD");
DoubleConstant LIMC_PID_P(1.0, "INTAKE_LIMC_PP");
DoubleConstant LIMC_PID_I(0.3, "INTAKE_LIMC_PI");
DoubleConstant LIMC_PID_D(0.0, "INTAKE_LIMC_PD");

double LimitController::calc(double target, double loc, double last_out,
		bool over) {
	double err = target - loc;
	if (fabs(err) > LIMC_FIELD_RAD) {
		// we assumed, crudely, that if the controller is above target it should go down
		err = err > 0 ? err - LIMC_RADIUS : err + LIMC_RADIUS;
		double icomp = ierr.calc(err) * LIMC_PID_I;
		double pcomp = err * LIMC_PID_P;
		double dcomp = derr.calc(err) * LIMC_PID_D;

		return icomp + pcomp + dcomp;
	} else {
		if (over) {
			return -LIMC_FIELD_POWER;
		} else {
			return LIMC_FIELD_POWER;
		}
	}
}

void LimitController::reset(double loc, bool over) {
	ierr.reset(0);
	derr.reset(loc);
}
