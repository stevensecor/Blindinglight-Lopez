#ifndef UTIL_MULTIMOTOR_H_
#define UTIL_MULTIMOTOR_H_

#include "safecanjag.h"
#include <vector>
#include <set>

/**
 * Preferred wrapper class for motors controlled by Jaguars
 * over CAN. The class permits simple control of a group of
 * motors as one unit, provides logging functions, and can 
 * reset motor settings if any controllers reboot.
 * 
 * Experimental timing indicates that write operations
 * on Jaguars take 0.25 ms, while read operations need
 * 2.5 ms.
 * 
 * This class is NOT threadsafe.
 */
class MultiMotor {
public:
	/**
	 * If `is_break` is true, the motors will be set to
	 * break mode. If false, the motors will be set to coast mode.
	 */
	MultiMotor(uint8_t, bool is_break);
	MultiMotor(uint8_t,uint8_t, bool is_break);
	MultiMotor(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t, bool is_break);
	virtual ~MultiMotor();
	
	/**
	 * Set break mode for all motors.
	 * 
	 * Cost: N WRITES
	 */
	void SetBreakMode(bool is_break);
	
	/**
	 * Printf any faults occuring to the constituent motor
	 * controllers.
	 * 
	 * Cost: N READS
	 */
	void PrintFaults();
	
	/**
	 * Printf log input voltage, current, temperature, and 
	 * output voltage.
	 * 
	 * Cost: 4*N READS
	 */
	void LogState();
	
	/**
	 * Returns true if any Jaguars had rebooted;
	 * if so, resets their configuration.
	 * 
	 * Cost: N READS, K WRITES
	 * (K is the number of reset controllers)
	 */
	bool ConditionalReflash();
	
	/**
	 * Reset the configuration of all motor controllers.
	 * 
	 * Cost: N WRITES
	 */
	void Reflash();
	
	/**
	 * Get a specific motor controller instance.
	 * 
	 * Both functions return NULL if the specified object
	 * is not found.
	 */
	SafeCANJag* GetByID(uint8_t can_id);
	SafeCANJag* GetIdx(int idx);
	
	/**
	 * Set the output of all motors to a given value.
	 * 
	 * Cost: / N > 1 => (N+1) WRITES \
	 *       \ N = 1 => 1 WRITE      /
	 */
	void Set(double setpoint);
	/**
	 * Set the output of all motors to a given value.
	 * Some motor controllers may update a few ms before the others. 
	 * 
	 * Cost: N WRITES
	 */
	void SetUnsynced(double setpoint);
	
	/**
	 * Reflash() all MultiMotors in existence.
	 * 
	 * Cost: J WRITES
	 * (J is the total number of Jaguars)
	 */
	static void ReflashAll();
private:
	void init(bool);
	void reflashIndividual(SafeCANJag*);
	bool break_mode;
	double last_set;
	std::vector<SafeCANJag*> jags;

	static std::set<MultiMotor*> mms;
};


#endif
