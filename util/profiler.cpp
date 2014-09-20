#include "profiler.h"
#include "Timer.h"
#include "udplog.h"

Profiler::Profiler(const char* out, bool udp) :
	stream() {
	key = out;
	use_udp = udp;
	config();
}
void Profiler::config() {
	stream.clear();
	stream << key << ":";
	first = true;
	last_time = GetTime();
}

void Profiler::finish() {
	// don't send empties
	if (first) {
		return;
	}
	stream << std::endl;
	if (use_udp) {
		UDPLog::log(stream.str().c_str());
	} else {
		printf(stream.str().c_str());
	}
}

Profiler::~Profiler() {
	finish();
}

void Profiler::next() {
	finish();
	config();
}

void Profiler::stamp(int) {
	double newtime = GetTime();
	double delta = newtime - last_time;
	last_time = newtime;

	if (first) {
		first = false;
	} else {
		stream << ",";
	}

	stream << delta;
}

double InitTimer::ltime = 0.0;
InitTimer::InitTimer(const char* msg) {
	if (msg != NULL) {
		printf(msg);
	}
	double newtime = GetTime();
	printf("Stamp: %f\n", newtime - ltime);
	ltime = newtime;
}
