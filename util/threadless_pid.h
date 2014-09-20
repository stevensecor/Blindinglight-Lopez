#ifndef THREADLESSPID_H_
#define THREADLESSPID_H_

/**
 * Time normalized variant on a PIDController. All time units are seconds; increasing
 * the frequency at which this is called should only improve response time/smoothness,
 * but maintain the general PID profile.
 */
class ThreadlessPID {
public:
	ThreadlessPID(double p, double i=0, double d=0, double f=0);
	void setConstants(double p, double i=0, double d=0, double f=0);

	/**
	 * if logname is nonzero, logs PID data to UDP 1140 as well.
	 * 
	 * Returns output.
	 */
	double calc(double target, double input, const char* logname=0);

	void reset();
private:
	double k_p;
	double k_i;
	double k_d;
	double k_f;

	double integral;
	double lerror;
	double ltime;
};

#endif /* THREADLESSPID_H_ */
