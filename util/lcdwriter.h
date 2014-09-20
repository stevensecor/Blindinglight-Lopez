#ifndef UTIL_LCDWRITER_H_
#define UTIL_LCDWRITER_H_

#include "DriverStationLCD.h"

/**
 * Cleanly write to the Driver Station LCD.
 */
class LCDWriter {
public:
	/**
	 * On destruction, the LCDWriter sends the written data
	 * to the driver station.
	 */
	LCDWriter();
	virtual ~LCDWriter();
	
	/**
	 * Clear the "LCD screen" on the dashboard.
	 */
	void clear();

	/**
	 * Write up to 21 characters to each line.
	 * Arguments are like printf().
	 */
	void line1(const char* format, ...);
	void line2(const char* format, ...);
	void line3(const char* format, ...);
	void line4(const char* format, ...);
	void line5(const char* format, ...);
	void line6(const char* format, ...);
private:
	DriverStationLCD *lcd;
};

#endif
