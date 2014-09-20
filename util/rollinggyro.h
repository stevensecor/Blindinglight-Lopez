#ifndef ROLLING_GYRO_H_
#define ROLLING_GYRO_H_

#include "AnalogChannel.h"
#include "Notifier.h"
#include "controllers.h"

/**
 * Alternate 
 */
class RollingGyro {
public:
	RollingGyro(uint8_t channel, double sensitivity = 0.007);
	virtual ~RollingGyro();

	void BeginMeasurement();
	void FinishMeasurement();
	
	bool IsDisconnected(); // only call this at assumed rest.

	double GetRawLevel();
	
	double GetAngle(); // value not to be trusted while calibrating
	void Reset();
	void SetSensitivity(double sensitivity);
private:
	void Calibrate();
	static void callCalibrate(void*);
	typedef struct {
		INT64 value;
		uint32_t count;
	} Reading;
	SEM_ID semaphore;
	Notifier notifier;
	AnalogChannel channel;
	double sensitivity;
	RingBuffer<Reading> buffer;
	double offset;
	bool calibrating;
};

#endif // ROLLING_GYRO_H_
