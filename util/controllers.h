#ifndef UTIL_CONTROLLERS_H_
#define UTIL_CONTROLLERS_H_

#include <stdio.h>

/**
 * Utility class; used to differentiate a stream of data based
 * on the time between updates.
 */
class TimeDifferentiator {
public:
	TimeDifferentiator();
	void reset(double val);
	double calc(double next);
	double lastTimeStep();
private:
	double last;
	double step;
	double last_time;
};

/**
 * Like the TimeDifferentiator; integrates a series of doubles, each
 * scaled by the time between updates.
 */
class TimeIntegrator {
public:
	TimeIntegrator();
	void reset(double val);
	double calc(double next);
	double lastTimeStep();
private:
	double integral;
	double step;
	double last_time;
};

/**
 * Ring buffer implementation. Contains a set of doubles.
 */
template<typename T>
class RingBuffer {
public:
	/**
	 * Create a ring buffer with the added capacity.
	 */
	RingBuffer(int capacity) {
		length = capacity + 1;
		buf = new T[length];
		tip = 0;
		tail = 0;
	}
	~RingBuffer() {
		delete[] buf;
	}

	/**
	 * Insert the next element: if `capacity` elements have been
	 * added, overwrites the earliest added element
	 */
	void next(T v) {
		buf[tip] = v;
		tip++;
		if (tip >= length) {
			tip = 0;
		}
		if (tip == tail) {
			tail++;
			if (tail >= length) {
				tail = 0;
			}
		}
	}

	/**
	 * Empty the collection: it will have size() 0.
	 */
	void clear() {
		tip = tail;
	}
	/**
	 * Number of elements in the collection.
	 */
	int size() const {
		if (tip >= tail) {
			return tip - tail;
		} else {
			return tip - tail + length;
		}
	}
	/**
	 * Indices range from 0 to k-1, where k is the number of elements added.
	 * Addition of an element decrements each present element's index
	 */
	T operator[](int i) const {
		if (i >= size() || i < 0) {
			printf("Index out of bounds: %d\n", i);
			return T();
		}
		int addr = i + tail;
		if (addr >= length) {
			return buf[addr - length];
		} else {
			return buf[addr];
		}
	}
private:
	T* buf;
	int length;
	int tip;
	int tail;
};

/**
 * Moving average filter on up to `size` doubles.
 * Averages as many values as have been added
 * up to a maximum size. 
 */
class MovingAverageFilter {
public:
	MovingAverageFilter(int size);
	/**
	 * Clear the filter, seed it with the value.
	 */
	void reset(double val);
	/**
	 * Return the average of the last N values,
	 * where N is the lesser of the number
	 * of added values and the filter size.
	 */
	double calc(double next);
	double recalc();
private:
	RingBuffer<double> buf;
};

#endif
