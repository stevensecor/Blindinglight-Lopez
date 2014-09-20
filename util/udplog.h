#ifndef UDPLOGGER_H
#define UDPLOGGER_H

/**
 * Example use:
 * 
 * bool uhOh(int errcode, int level) {
 * 		if (errcode)
 *	 		UDPLog::log(
 *	 			"You got an error! Advance to square %d and recieve %d bucks!\n",
 *	 		 	errcode, level);
 *	 	return errcode != 0;
 * }
 * 
 * This logger sends information over port 1140 to the driver station laptop. 
 * 
 */

namespace UDPLog {
void setup();
void destroy();
void log(const char* fmt, ...);
}

#endif
