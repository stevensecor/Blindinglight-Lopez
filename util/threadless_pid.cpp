#include "threadless_pid.h"
#include "Timer.h"
#include "udplog.h"

ThreadlessPID::ThreadlessPID(double p, double i, double d, double f) {
	setConstants(p, i, d, f);
	integral = 0.0;
	lerror = 0.0;
	ltime = Timer::GetPPCTimestamp();
}

void ThreadlessPID::setConstants(double p, double i, double d, double f) {
	k_p = p;
	k_i = i;
	k_d = d;
	k_f = f;
}

void ThreadlessPID::reset() {
	integral = 0.0;
	lerror = 0.0;
	ltime = Timer::GetPPCTimestamp();
}

double ThreadlessPID::calc(double target, double measure, const char* name) {
	double time_new = Timer::GetPPCTimestamp();
	double tdelta = time_new - ltime;
	ltime = time_new;

	double error_new = target - measure;
	double diff = (error_new - lerror) / tdelta;
	lerror = error_new;

	integral += error_new * tdelta;

	double p_comp = (k_p * error_new);
	double k_comp = (k_d * diff);
	double i_comp = (k_i * integral);
	double f_comp = (k_f * target);
	double output = p_comp + k_comp + i_comp + f_comp;

	if (name != 0) {
		UDPLog::log("%s:0.0,1.0,%f,%f,%f,%f,%f,%f,%f\n", name, target, measure,
				output, p_comp, k_comp, i_comp, f_comp);
	}

	return output;
}
