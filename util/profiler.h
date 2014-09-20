#ifndef UTIL_PROFILER_H_
#define UTIL_PROFILER_H_

#include <sstream> 

/**
 * Create stack based profiler object.
 * Writes a set of time deltas to UDP port 1140
 * with printf format "%s:%f,%f,%f\n".
 */
class Profiler {
public:
	/**
	 * Create a Profiler with specified key.
	 * On destruction, the Profiler will
	 * send the data.
	 */
	Profiler(const char* key, bool udp = true);
	~Profiler();

	/**
	 * Mark the current time and append the last time
	 * delta to the output string.
	 * 
	 * Optional argument is in case you want to:
	 * >
	 * >  stamp(1);
	 * >  intensive();
	 * >  stamp(2);
	 * >  cheap();
	 * >  stamp(3);
	 */
	void stamp(int whatever = 0);

	/**
	 * Send built up time deltas to UDP port 1140,
	 * and prepare to send another set of time deltas.
	 */
	void next();
private:
	void config();
	void finish();
	std::stringstream stream;
	double last_time;
	bool first;
	bool use_udp;
	const char* key;
};

/**
 * Printf a message on initialization,
 * and printf the elapsed time since the
 * last time an InitTimer was constructed.
 */
class InitTimer {
public:
	InitTimer(const char* msg = NULL);
private:
	static double ltime;
};

#endif
