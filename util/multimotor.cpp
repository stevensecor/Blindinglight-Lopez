#include "multimotor.h"
#include <stdio.h>

//
// TODO: add voltage ramping (VCOMP_IN, VCOMP_COMP) to this
// improve time efficiency, diagnostics
//
//
//


std::set<MultiMotor*> MultiMotor::mms = std::set<MultiMotor*>();
// 'cause we type it to much
typedef std::vector<SafeCANJag*>::iterator j_t;

MultiMotor::MultiMotor(uint8_t a, bool is_break) {
	jags.push_back(new SafeCANJag(a));
	init(is_break);
}
MultiMotor::MultiMotor(uint8_t a, uint8_t b, bool is_break) {
	jags.push_back(new SafeCANJag(a));
	jags.push_back(new SafeCANJag(b));
	init(is_break);
}
MultiMotor::MultiMotor(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e,
		uint8_t f, bool is_break) {
	jags.push_back(new SafeCANJag(a));
	jags.push_back(new SafeCANJag(b));
	jags.push_back(new SafeCANJag(c));
	jags.push_back(new SafeCANJag(d));
	jags.push_back(new SafeCANJag(e));
	jags.push_back(new SafeCANJag(f));
	init(is_break);
}
void MultiMotor::init(bool is_break) {
	SetBreakMode(is_break);
	last_set = 0.0;
	mms.insert(this);
}
MultiMotor::~MultiMotor() {
	mms.erase(this);
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		delete *i;
	}
}

void MultiMotor::SetBreakMode(bool is_break) {
	break_mode = is_break;
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		(*i)->ConfigNeutralMode(break_mode ? SafeCANJag::kNeutralMode_Brake : SafeCANJag::kNeutralMode_Coast);
		
	}
}
void MultiMotor::reflashIndividual(SafeCANJag* x) {
	x->ConfigNeutralMode(break_mode ? SafeCANJag::kNeutralMode_Brake : SafeCANJag::kNeutralMode_Coast);
}


bool MultiMotor::ConditionalReflash() {
	bool any = false;
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		SafeCANJag* x = *i;
		if (x->GetPowerCycled()) {
			any = true;
			reflashIndividual(x);
		}
	}
	return any;
}


void MultiMotor::Reflash() {
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		reflashIndividual(*i);
	}
}

SafeCANJag* MultiMotor::GetByID(uint8_t can_id) {
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		SafeCANJag* m = *i;
		if (m->getID() == can_id) {
			return m;
		}
	}
	printf("Could not find motor id %d\n",can_id);
	return NULL;
}

SafeCANJag* MultiMotor::GetIdx(int idx) {
	size_t i = idx;
	if (i >= jags.size()) {
		return NULL;
	}
	return jags[idx];
}

void MultiMotor::Set(double setpoint) {
	if (jags.size() == 1) {
		jags[0]->Set(setpoint);
		return;
	}

	const uint8_t sync = 0x01;
	last_set = setpoint;
	for (std::vector<SafeCANJag*>::iterator i = jags.begin(); i != jags.end(); i++) {
		(*i)->Set(setpoint, sync);
	}
	SafeCANJag::UpdateSyncGroup(sync);

}
void MultiMotor::SetUnsynced(double setpoint) {
	last_set = setpoint;
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		(*i)->Set(setpoint);
	}
}

void MultiMotor::PrintFaults() {
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		SafeCANJag* x = *i;
		uint16_t code = x->GetFaults();
		int id = x->getID();

		if (code & SafeCANJag::kCurrentFault) {
			printf("Jaguar Current Fault: %d\n", id);
		}
		if (code & SafeCANJag::kTemperatureFault) {
			printf("Jaguar Temp Fault: %d\n", id);
		}
		if (code & SafeCANJag::kBusVoltageFault) {
			printf("Jaguar Volt Fault: %d\n", id);
		}
		if (code & SafeCANJag::kGateDriverFault) {
			printf("Jaguar Gate Fault: %d\n", id);
		}
	}
}

void MultiMotor::LogState() {
	for (j_t i = jags.begin(); i != jags.end(); i++) {
		SafeCANJag* x = *i;

		printf("id: %d; ivolt %2.6f, curr %2.6f, temp %2.6f, ovolt %2.6f\n",
				x->getID(), x->GetBusVoltage(), x->GetOutputCurrent(),
				x->GetTemperature(), x->GetOutputVoltage());
	}
}

void MultiMotor::ReflashAll() {
	for (std::set<MultiMotor*>::iterator i = mms.begin(); i != mms.end(); i++) {
		(*i)->Reflash();
	}
}
