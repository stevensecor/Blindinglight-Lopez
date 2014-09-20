#ifndef UTIL_CALC_H_
#define UTIL_CALC_H_

double scaleLinear(double v, double old_min, double old_max,
		double new_min, double new_max);

double bound(double v, double min, double max);

#endif
