#ifndef IOMAP_H_
#define IOMAP_H_

// CANID 1 is factory default, reserved to
// quickly replace/add another Jaguar without
// assigning a new id
const int CANID_DRIVE_LEFT_FRONT = 2;
const int CANID_DRIVE_LEFT_REAR = 3;
const int CANID_DRIVE_RIGHT_FRONT = 4;
const int CANID_DRIVE_RIGHT_REAR = 5;
const int CANID_KICKER_LEFT_FRONT = 6;
const int CANID_KICKER_LEFT_MID = 7;
const int CANID_KICKER_LEFT_BACK = 8;
const int CANID_KICKER_RIGHT_FRONT = 9;
const int CANID_KICKER_RIGHT_MID = 10;
const int CANID_KICKER_RIGHT_BACK = 11;
const int CANID_INTAKE_ROLLER = 12;
const int CANID_INTAKE_LIFT = 13;

// PWM CHANNELS
const int PWM_KICKER_GUARD_LEFT = 9;
const int PWM_KICKER_GUARD_RIGHT = 10;

// RELAY OUTPUTS
// No relay outputs yet

// ANALOG INPUTS
const int ANALOG_GYRO = 1;
const int ANALOG_INTAKE_POT_1 = 2;

const int ANALOG_INTAKE_ALT_POT = 6;

const int ANALOG_BATTERY_VOLTAGE = 8;// reserved

// DIGITAL IO
const int DIG_DRIVE_ENCODER_LEFT_A = 1;
const int DIG_DRIVE_ENCODER_LEFT_B = 2;
const int DIG_DRIVE_ENCODER_RIGHT_A = 3;
const int DIG_DRIVE_ENCODER_RIGHT_B = 4;
const int DIG_KICKER_ENCODER_A = 5;
const int DIG_KICKER_ENCODER_B = 6;
const int DIG_KICKER_LOW_FLAG = 7;
const int DIG_INTAKE_POS_LIMIT = 8;
const int DIG_KICKER_HIGH_FLAG = 9;
const int DIG_INTAKE_BALL_SENSOR = 10;

const int DIG_LIGHTS_INDICATE_RED = 11;
const int DIG_LIGHTS_INDICATE_GREEN = 12;
const int DIG_LIGHTS_INDICATE_BLUE = 13;
const int DIG_LIGHTS_CAMERA = 14;

// I2C REGISTERS/DEVICES
// No I2C registers or devices yet

// PNEUMATICS
// No Pneumatics yet

#endif /* IOMAP_H_ */
