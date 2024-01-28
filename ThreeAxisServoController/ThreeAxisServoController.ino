/*
  ThreeAxisServoController

  Controls a system of 3 brushed DC motor servos with potentiometer and
  current sense feedback. Uses Kalman filters to estimate the position and
  external load torque of each servo.

  This sketch was written specifically for Arduino Micro boards.

  Control loop input pinout:
  - analog input 0: servo 1 position potentiometer
  - analog input 1: servo 1 current sense
  - analog input 2: servo 2 position potentiometer
  - analog input 3: servo 2 current sense
  - analog input 4: servo 3 position potentiometer
  - analog input 5: servo 3 current sense
  - analog input 6: servo power voltage sense
*/


//*****************************************************************************
//***************************** INCLUDE LIBRARIES *****************************
//*****************************************************************************

// matrix manipulation library for Kalman filter
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

// fixed point arithmetic library
#define FIXED_POINTS_NO_RANDOM
#include <FixedPoints.h>
#include <FixedPointsCommon.h>

// SPI for communication with host controller
#include <SPI.h>


//*****************************************************************************
//**************************** DEFINE PIN LAYOUT ******************************
//*****************************************************************************

// Analog inputs
#define TASC_PIN_S1_POTENTIOMETER A0
#define TASC_PIN_S1_CURRENT_SENSE A1
#define TASC_PIN_S2_POTENTIOMETER A2
#define TASC_PIN_S2_CURRENT_SENSE A3
#define TASC_PIN_S3_POTENTIOMETER A4
#define TASC_PIN_S3_CURRENT_SENSE A5
#define TASC_PIN_SERVO_POWER_VOLTAGE A6

// Digital I/O
#define TASC_PIN_DRV2_NSLEEP 0
#define TASC_PIN_DRV2_NFAULT 1
#define TASC_PIN_DRV1_NFAULT 2
#define TASC_PIN_DRV1_IN1 3
#define TASC_PIN_DRV1_IN2 5
#define TASC_PIN_DRV2_IN1 6
#define TASC_PIN_DRV3_NFAULT 7
#define TASC_PIN_DRV3_NSLEEP 8
#define TASC_PIN_DRV2_IN2 9
#define TASC_PIN_DRV3_IN1 10
#define TASC_PIN_DRV3_IN2 11
#define TASC_PIN_DRVOFF 12
#define TASC_PIN_SPI_CS 13
#define TASC_PIN_SPI_CIPO 14
#define TASC_PIN_SPI_SCK 15
#define TASC_PIN_SPI_COPI 16
#define TASC_PIN_DRV1_NSLEEP 17


//*****************************************************************************
//************************ KALMAN FILTER DEFINITIONS **************************
//*****************************************************************************

typedef SQ15x16 fixed_point;

typedef struct {
  fixed_point position;
  fixed_point velocity;
  fixed_point load_torque;
} CustomServoState;

typedef struct {
  fixed_point position_variance;
  fixed_point velocity_variance;
  fixed_point load_torque_variance;
} CustomServoStateCovariance;

typedef struct {
  fixed_point moment_of_inertia;
  fixed_point torque_coefficient;
  fixed_point positive_kinetic_friction;
  fixed_point positive_constant_friction;
  fixed_point positive_static_friction;
  fixed_point negative_kinetic_friction;
  fixed_point negative_constant_friction;
  fixed_point negative_static_friction;
} CustomServoPhysicalParameters;

typedef struct {
  fixed_point potentiometer_noise_variance;
  fixed_point current_sense_noise_variance;
  fixed_point position_process_variance;
  fixed_point velocity_process_variance;
  fixed_point load_torque_process_variance;
} CustomServoFilterParameters;


//*****************************************************************************
//********************* MOTOR DRIVER IC COMMS VARIABLES ***********************
//*****************************************************************************

// drive enable bools - set true or false
bool TASC_drv1Enable = false;
bool TASC_drv2Enable = false;
bool TASC_drv3Enable = false;
// drive faulted bools - set false to re-enable drive after fault
bool TASC_drv1Fault = false;
bool TASC_drv2Fault = false;
bool TASC_drv3Fault = false;
// drive enabled bools - true when nFAULT handshake is complete and no new faults have occured
bool TASC_drv1Enabled = false;
bool TASC_drv2Enabled = false;
bool TASC_drv3Enabled = false;

// motor target voltages
fixed_point TASC_drv1Voltage = 0.0;
fixed_point TASC_drv2Voltage = 0.0;
fixed_point TASC_drv3Voltage = 0.0;


//*****************************************************************************
//**************************** INPUT VARIABLES ********************************
//*****************************************************************************

// these are sensor readings that have been preprocessed to the desired units.

// incoming motor power voltage - volts
fixed_point TASC_motorSupplyVoltage = 0.0;
// servo position potentiometer values - radians, centered on zero at mid pot
fixed_point TASC_drv1Potentiometer = 0.0;
fixed_point TASC_drv2Potentiometer = 0.0;
fixed_point TASC_drv3Potentiometer = 0.0;
// motor current sense values - amps
fixed_point TASC_drv1Current = 0.0;
fixed_point TASC_drv2Current = 0.0;
fixed_point TASC_drv3Current = 0.0;


//*****************************************************************************
//********************** FUNCTION FORWARD DECLARATIONS ************************
//*****************************************************************************

void TASC_DRV8243_nsleep_handshake();
void TASC_readSensors();
void TASC_writeMotorVoltages();


//*****************************************************************************
//**************************** MAIN PROGRAM LOGIC *****************************
//*****************************************************************************

void setup() {
  // initialize digital I/O pins
  pinMode(TASC_PIN_DRVOFF, OUTPUT);
  pinMode(TASC_PIN_DRV1_NFAULT, INPUT);
  pinMode(TASC_PIN_DRV1_NSLEEP, OUTPUT);
  pinMode(TASC_PIN_DRV1_IN1, OUTPUT);
  pinMode(TASC_PIN_DRV1_IN2, OUTPUT);
  pinMode(TASC_PIN_DRV2_NFAULT, INPUT);
  pinMode(TASC_PIN_DRV2_NSLEEP, OUTPUT);
  pinMode(TASC_PIN_DRV3_NFAULT, INPUT);
  pinMode(TASC_PIN_DRV3_NSLEEP, OUTPUT);
  pinMode(TASC_PIN_SPI_CS, INPUT);

  // turn all motor drives off when starting up
  digitalWrite(TASC_PIN_DRVOFF, HIGH);
  digitalWrite(TASC_PIN_DRV1_NSLEEP, !TASC_drv1Enable);
  digitalWrite(TASC_PIN_DRV2_NSLEEP, !TASC_drv2Enable);
  digitalWrite(TASC_PIN_DRV3_NSLEEP, !TASC_drv3Enable);

  SPI.begin(); // TODO SPI comms between arduino and host controller

  // enable drive 1
  TASC_drv1Enable = true;
  digitalWrite(TASC_PIN_DRVOFF, LOW);
}

void loop() {
  TASC_DRV8243_nsleep_handshake();

  if (TASC_drv1Fault) { // drive is faulted
    // naively re-enable the drive
    TASC_drv1Fault = false;
  }

  TASC_readSensors();

  // TODO state estimation

  // TODO servo motion control

  // write target motor voltages to drives
  TASC_writeMotorVoltages();
}


//*****************************************************************************
//************************** FUNCTION DEFINITIONS *****************************
//*****************************************************************************

/*
 * TASC_DRV8243_nsleep_handshake
 *
 * Handles basic communication with the motor drivers.
 */
void TASC_DRV8243_nsleep_handshake() {
  bool TASC_drv1Nfault = digitalRead(TASC_PIN_DRV1_NFAULT);
  bool TASC_drv2Nfault = digitalRead(TASC_PIN_DRV2_NFAULT);
  bool TASC_drv3Nfault = digitalRead(TASC_PIN_DRV3_NFAULT);
  TASC_drv1Fault = TASC_drv1Fault || (TASC_drv1Enable && TASC_drv1Enabled && !TASC_drv1Nfault);
  TASC_drv2Fault = TASC_drv2Fault || (TASC_drv2Enable && TASC_drv2Enabled && !TASC_drv2Nfault);
  TASC_drv3Fault = TASC_drv3Fault || (TASC_drv3Enable && TASC_drv3Enabled && !TASC_drv3Nfault);
  TASC_drv1Enabled = TASC_drv1Enabled && TASC_drv1Nfault;
  TASC_drv2Enabled = TASC_drv2Enabled && TASC_drv2Nfault;
  TASC_drv3Enabled = TASC_drv3Enabled && TASC_drv3Nfault;

  if (TASC_drv1Enable && !TASC_drv1Enabled && !TASC_drv1Fault) {
    if (!TASC_drv1Nfault) {
      digitalWrite(TASC_PIN_DRV1_NSLEEP, LOW);
    } else {
      digitalWrite(TASC_PIN_DRV1_NSLEEP, HIGH);
      TASC_drv1Enabled = true;
    }
  } else if (!TASC_drv1Enable && TASC_drv1Enabled) {
    digitalWrite(TASC_PIN_DRV1_NSLEEP, LOW);
  }
  if (TASC_drv2Enable && !TASC_drv2Enabled && !TASC_drv2Fault) {
    if (!TASC_drv2Nfault) {
      digitalWrite(TASC_PIN_DRV2_NSLEEP, LOW);
    } else {
      digitalWrite(TASC_PIN_DRV2_NSLEEP, HIGH);
      TASC_drv2Enabled = true;
    }
  } else if (!TASC_drv2Enable && TASC_drv2Enabled) {
    digitalWrite(TASC_PIN_DRV2_NSLEEP, LOW);
  }
  if (TASC_drv3Enable && !TASC_drv3Enabled && !TASC_drv3Fault) {
    if (!TASC_drv3Nfault) {
      digitalWrite(TASC_PIN_DRV3_NSLEEP, LOW);
    } else {
      digitalWrite(TASC_PIN_DRV3_NSLEEP, HIGH);
      TASC_drv3Enabled = true;
    }
  } else if (!TASC_drv3Enable && TASC_drv3Enabled) {
    digitalWrite(TASC_PIN_DRV3_NSLEEP, LOW);
  }
}

/*
 * TASC_readSensors
 *
 * Reads sensor values, preprocesses them, and writes them to global variables.
 */
void TASC_readSensors() {
  TASC_motorSupplyVoltage = float(analogRead(TASC_PIN_SERVO_POWER_VOLTAGE)) / 1023.0 * 5.0 * (4530.0 + 2200.0) / 2200.0;
  TASC_drv1Potentiometer = (float(analogRead(TASC_PIN_S1_POTENTIOMETER)) / 1023.0 * 2.0 - 1.0) * 3.1415926; // TODO incorporate pot range calibration
  TASC_drv2Potentiometer = (float(analogRead(TASC_PIN_S2_POTENTIOMETER)) / 1023.0 * 2.0 - 1.0) * 3.1415926; // TODO incorporate pot range calibration
  TASC_drv3Potentiometer = (float(analogRead(TASC_PIN_S3_POTENTIOMETER)) / 1023.0 * 2.0 - 1.0) * 3.1415926; // TODO incorporate pot range calibration
  TASC_drv1Current = (float(analogRead(TASC_PIN_S1_CURRENT_SENSE)) / 1023.0 * 5.0 - 2.5) / 0.185; // TODO correct nonstationary bias
  TASC_drv2Current = (float(analogRead(TASC_PIN_S2_CURRENT_SENSE)) / 1023.0 * 5.0 - 2.5) / 0.185; // TODO correct nonstationary bias
  TASC_drv3Current = (float(analogRead(TASC_PIN_S3_CURRENT_SENSE)) / 1023.0 * 5.0 - 2.5) / 0.185; // TODO correct nonstationary bias
}

/*
 * TASC_writeMotorVoltages
 *
 * Maps target motor voltages to PWM outputs.
 */
void TASC_writeMotorVoltages() {
  if (TASC_drv1Enabled) {
    if (TASC_drv1Voltage >= 0.0) {
      digitalWrite(TASC_PIN_DRV1_IN2, LOW);
      analogWrite(
        TASC_PIN_DRV1_IN1,
        constrain(int(TASC_drv1Voltage / TASC_motorSupplyVoltage * 255.0), 0, 255)
      );
    } else {
      digitalWrite(TASC_PIN_DRV1_IN1, LOW);
      analogWrite(
        TASC_PIN_DRV1_IN2,
        constrain(int(-TASC_drv1Voltage / TASC_motorSupplyVoltage * 255.0), 0, 255)
      );
    }
  } else {
    digitalWrite(TASC_PIN_DRV1_IN1, LOW);
    digitalWrite(TASC_PIN_DRV1_IN2, LOW);
  }
  if (TASC_drv2Enabled) {
    if (TASC_drv2Voltage >= 0.0) {
      digitalWrite(TASC_PIN_DRV2_IN2, LOW);
      analogWrite(
        TASC_PIN_DRV2_IN1,
        constrain(int(TASC_drv2Voltage / TASC_motorSupplyVoltage * 255.0), 0, 255)
      );
    } else {
      digitalWrite(TASC_PIN_DRV2_IN1, LOW);
      analogWrite(
        TASC_PIN_DRV2_IN2,
        constrain(int(-TASC_drv2Voltage / TASC_motorSupplyVoltage * 255.0), 0, 255)
      );
    }
  } else {
    digitalWrite(TASC_PIN_DRV2_IN1, LOW);
    digitalWrite(TASC_PIN_DRV2_IN2, LOW);
  }
  if (TASC_drv3Enabled) {
    if (TASC_drv3Voltage >= 0.0) {
      analogWrite(TASC_PIN_DRV3_IN2, 0);
      analogWrite(
        TASC_PIN_DRV3_IN1,
        constrain(int(float(TASC_drv3Voltage / TASC_motorSupplyVoltage * 255.0)), 0, 255)
      );
    } else {
      analogWrite(TASC_PIN_DRV3_IN1, 0);
      analogWrite(
        TASC_PIN_DRV3_IN2,
        constrain(int(float(-TASC_drv3Voltage / TASC_motorSupplyVoltage * 255.0)), 0, 255)
      );
    }
  } else {
    digitalWrite(TASC_PIN_DRV3_IN1, LOW);
    digitalWrite(TASC_PIN_DRV3_IN2, LOW);
  }
}
