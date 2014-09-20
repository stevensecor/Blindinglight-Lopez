#ifndef UTIL_CONSTANTS_H_
#define UTIL_CONSTANTS_H_

/**
 * Repository for values that, though they are constants,
 * tend to be changed at a very high rate or with time constraints.
 * Though not strictly variables, these remain unchanged over the
 * duration of a mode. This are very useful for PID tuning
 * and autonomous parameters.
 * 
 * On robot startup, Constants::load() should be called, to read
 * the last values from robot memory. Constants::reload() will then
 * check at the start of a mode if any values have changed; if so, 
 * all constants will assume the new values.
 */
namespace Constants {
void load();
void reload();
}

class IntConstant {
private:
	IntConstant(int val, const char* name);
public:
	~IntConstant();
	operator int() const;
	// never call update
	void update(int val);
	const char* getName();
private:
	// constants may not be moved/copied
	IntConstant(const IntConstant&);             
	void operator=(const IntConstant&);
	
	int value;
	const char* name;
};

/**
 * WARNING: USE only ALL CAPS, NUMBERS (1234), and 
 * UNDERSCORES (_) 
 */
class DoubleConstant {
public:
	DoubleConstant(double val, const char* name);
	~DoubleConstant();
	operator double() const;
	// never call update
	void update(double val);
	const char* getName();
private:
	// constants may not be moved/copied
	DoubleConstant(const DoubleConstant&);             
	void operator=(const DoubleConstant&);
	
	double value;
	const char* name;
};

#endif
