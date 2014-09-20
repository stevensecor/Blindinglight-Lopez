#include "rollinggyro.h"
#include "AnalogModule.h"

#define LOUD_GYRO 0

//
// Thank you to FRC1538, who inspired me to finally try this.
//

const double PERIOD = 0.5;
const int BUFFER_SIZE = 12;

RollingGyro::RollingGyro(uint8_t chan, double sens) :
	notifier(RollingGyro::callCalibrate, this), channel(chan),
			buffer(BUFFER_SIZE) {

	sensitivity = sens;

	if (!channel.IsAccumulatorChannel()) {
		printf("ERROR: Gyro not on accumulator channel");
		return;
	}

	channel.SetAverageBits(0);
	channel.SetOversampleBits(10);
	channel.InitAccumulator();

	offset = 0.0;

	semaphore = semMCreate(SEM_Q_PRIORITY);
	calibrating = false;
}

RollingGyro::~RollingGyro() {
	notifier.Stop();
	semFlush(semaphore);
}

void RollingGyro::BeginMeasurement() {
#if LOUD_GYRO
	printf("[Gyro] Begin\n");
#endif
	Synchronized sync(semaphore);

	if (calibrating) {
		printf("[Gyro] Already began calibration.");
		return;
	}
	calibrating = true;

	channel.SetAccumulatorCenter(0);
	channel.ResetAccumulator();
	buffer.clear();

	notifier.StartPeriodic(PERIOD);
}

void RollingGyro::FinishMeasurement() {
#if LOUD_GYRO
	printf("[Gyro] Finish\n");
#endif
	Synchronized sync(semaphore);

	if (!calibrating) {
		printf("[Gyro] Calibration has not yet been started.");
		return;
	}
	calibrating = false;

	notifier.Stop();

	// When the measurement period is too short...
	if (buffer.size() == 0) {
		Calibrate();
	}

	Reading total = { 0, 0 };
	for (int i = 0; i < buffer.size(); i++) {
		Reading n = buffer[i];
		total.count += n.count;
		total.value += n.value;
	}

	uint32_t center = (uint32_t) ((float) total.value / (float) total.count
			+ .5);
	offset = ((float) total.value / (float) total.count) - (float) center;

	channel.SetAccumulatorCenter(center);
	channel.SetAccumulatorDeadband(0);
	channel.ResetAccumulator();

}
double RollingGyro::GetAngle() {
	double angle;
	{
		Synchronized sync(semaphore);
		angle = 0;

		// Copied from WPILib implementation

		INT64 rawValue;
		uint32_t count;
		channel.GetAccumulatorOutput(&rawValue, &count);

		INT64 value = rawValue - (INT64) ((float) count * offset);

		double scaledValue = value * 1e-9 * (double) channel.GetLSBWeight()
				* (double) (1 << channel.GetAverageBits())
				/ (channel.GetModule()->GetSampleRate() * sensitivity);

		return (float) scaledValue;
	}
	return angle;
}
void RollingGyro::Reset() {
	Synchronized sync(semaphore);
	channel.ResetAccumulator();
}

void RollingGyro::SetSensitivity(double val) {
	Synchronized sync(semaphore);

	sensitivity = val;
}

void RollingGyro::Calibrate() {
	Synchronized sync(semaphore);
#if LOUD_GYRO
	if (buffer.size() < 12) {
		printf("[Gyro] cal %d\n", buffer.size());
	}
#endif
	Reading r;
	channel.GetAccumulatorOutput(&r.value, &r.count);
	buffer.next(r);
	channel.ResetAccumulator();
}

void RollingGyro::callCalibrate(void* x) {
	((RollingGyro*) x)->Calibrate();
}

bool RollingGyro::IsDisconnected() {
	// when disconnected, the analog module read voltage is (0 + noise).
	// when connected, the ADX#@$%, steady, shows 2.5 V
	double v = channel.GetValue();
	const double NOISE = 0.050;
	return v < NOISE && v > -NOISE;
}

double RollingGyro::GetRawLevel() {
	return channel.GetVoltage();
}
