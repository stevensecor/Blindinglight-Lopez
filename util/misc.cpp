#include "misc.h"
#include "DriverStation.h"

double GetBatteryVoltage() {
	return DriverStation::GetInstance()->GetBatteryVoltage();
}
