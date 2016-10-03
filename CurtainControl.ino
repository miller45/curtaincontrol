#define DRMODE
#define TESTMODE
// FabScan - http://hci.rwth-aachen.de/fabscan
//
// R. Bohne 30.12.2013
// This sketch tests all four stepper drivers on the FabScan-Shield V1.1
// It also turns on the light and the laser
// This sketch is not the real FabScan firmware, but just a test script for people who want to test their hardware!
// at startup, the sketch blinks all leds, the light and the motor a few times and after that, it starts to spin the motors.
#include <AccelStepper.h>

#define STEP_US 10

#define LIGHT_PIN 13
#define LASER_PIN 18
#define MS_PIN    19

//Stepper 1 as labeled on Shield, Turntable
#define ENABLE_PIN_0  2
#define STEP_PIN_0    3
#define DIR_PIN_0     4
#define CWPIN 8
#define CCWPIN 9

#define STDSPEED 200

int stepMul = 1;


bool lastDir = LOW;

void stepForward() {
  if (lastDir != LOW) {
    digitalWrite(DIR_PIN_0, LOW);
    lastDir = LOW;
  }
  digitalWrite(STEP_PIN_0, HIGH);
  delayMicroseconds(STEP_US);
  digitalWrite(STEP_PIN_0, LOW);
  delayMicroseconds(STEP_US);
}


void stepBackward() {
  if (lastDir != HIGH) {
    digitalWrite(DIR_PIN_0, HIGH);
    lastDir = HIGH;
  }
  digitalWrite(STEP_PIN_0, HIGH);
  delayMicroseconds(STEP_US);
  digitalWrite(STEP_PIN_0, LOW);
  delayMicroseconds(STEP_US);

}

#ifdef DRMODE
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN_0, DIR_PIN_0);
#else
AccelStepper stepper(stepForward, stepForward);
#endif


void setup()
{
  Serial.begin(9600);
  pinMode(CWPIN, INPUT_PULLUP);
  pinMode(CCWPIN, INPUT_PULLUP);
  //pinMode(LASER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  //  pinMode(MS_PIN, OUTPUT);
  //digitalWrite(MS_PIN, HIGH);  //HIGH for 16microstepping, LOW for no microstepping

  //is done by enable stepper pinMode(ENABLE_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(STEP_PIN_0, OUTPUT);

  enableStepper();

#ifdef DRMODE
  stepper.setEnablePin(ENABLE_PIN_0);
#else
  
#endif
  digitalWrite(ENABLE_PIN_0, LOW);  //HIGH to turn off

  //blink all leds, lights ans the laser 10 times
  for (int i = 0; i < 10; i++)
  {
    //   digitalWrite(ENABLE_PIN_0, HIGH);  //HIGH to turn off

    digitalWrite(LIGHT_PIN, 0); //turn light off
    digitalWrite(LASER_PIN, 0); //turn laser off
    delay(120);
    //   digitalWrite(ENABLE_PIN_0, LOW);  //HIGH to turn off

    digitalWrite(LIGHT_PIN, 1); //turn light on
    digitalWrite(LASER_PIN, 1); //turn laser on
    delay(120);
  }

  stepper.setMaxSpeed(500 * stepMul);
  stepper.setAcceleration(1000.0 * stepMul);
  //stepper.setSpeed(1500);
}

int lldir = 0;

#ifdef TESTMODE

void loop(){
  
}

#else

void loop() {
  int dir = 1 - digitalRead(CWPIN) - (1 - digitalRead(CCWPIN));
  //stepper.runSpeed();
  if (dir != lldir) {
    lldir = dir;
    if (dir != 0) {
      enableStepper();
      stepper.setSpeed(dir * STDSPEED * stepMul);
    } else {      
      stepper.setSpeed(0);
    }
  }
  if (dir != 0) {    
    stepper.runSpeed();
  } else {
    standbyStepper();
  }

}

#endif


int stepperState = -2;

void enableStepper() {
  if (stepperState != 2) {
    stepperState = 2;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, LOW);
  }
}
void disableStepper() {
  if (stepperState != 0) {
    stepperState = 0;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, HIGH);
  }
}

void standbyStepper() {
  if (stepperState != 1) {
    stepperState = 1;
    digitalWrite(ENABLE_PIN_0, LOW);
    pinMode(ENABLE_PIN_0, INPUT);
  }
}



