#include "controllers.h"
#include "Timer.h"

TimeDifferentiator::TimeDifferentiator() {
}

void TimeDifferentiator::reset(double val) {
	last_time = GetTime();
	last = val;
	step = 0.0;
}

double TimeDifferentiator::calc(double next) {
	double time = GetTime();
	step = time - last_time;
	last_time = time;

	double delta = (next - last);
	last = next;
	if (step == 0.0) {
		return 0.0;
	} else {
		return delta / step;
	}
}

double TimeDifferentiator::lastTimeStep() {
	return step;
}

TimeIntegrator::TimeIntegrator() {
}

void TimeIntegrator::reset(double val) {
	last_time = GetTime();
	integral = val;
	step = 0.0;
}

double TimeIntegrator::calc(double next) {
	double time = GetTime();
	step = time - last_time;
	last_time = time;

	integral += next * step;
	return integral;
}

double TimeIntegrator::lastTimeStep() {
	return step;
}

MovingAverageFilter::MovingAverageFilter(int size) :
	buf(size) {
}

void MovingAverageFilter::reset(double val) {
	buf.clear();
	buf.next(val);
}

double MovingAverageFilter::calc(double next) {
	buf.next(next);
	return recalc();
}

double MovingAverageFilter::recalc() {
	int l = buf.size();
	double sum = 0;
	for (int i = 0; i < l; i++) {
		sum += buf[i];
	}
	return sum / ((double) l);
}
