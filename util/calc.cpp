#include "calc.h"

double scaleLinear(double v, double old_min, double old_max,
		double new_min, double new_max) {
	return (v - old_min) / (old_max - old_min) * (new_max - new_min) + new_min;
}

double bound(double v, double min, double max) {
	if (v < min) {
		return min;
	}
	if (v > max) {
		return max;
	}
	return v;
}
