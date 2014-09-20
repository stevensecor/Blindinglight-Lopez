#ifndef UTIL_H
#define UTIL_H

/**
 * Collection of utility classes and functions to improve development; these
 * are a robot-agnostic substructure to the robot code proper. 
 * 
 * Do not directly include any of the headers of the `util` folder: instead,
 * #include "util.h"
 */

#include "util/buttonlatch.h"
#include "util/constants.h"
#include "util/calc.h"
#include "util/safecanjag.h"
#include "util/lcdwriter.h"
#include "util/threadless_pid.h"
#include "util/udplog.h"
#include "util/controllers.h"
#include "util/profiler.h"
#include "util/multimotor.h"
#include "util/rollinggyro.h"
#include "util/misc.h"

#endif
