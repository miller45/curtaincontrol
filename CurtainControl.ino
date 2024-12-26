//#define LOGINPUT
//#define SWEEP
// FabScan - http://hci.rwth-aachen.de/fabscan
//
// R. Bohne 30.12.2013
// This sketch tests all four stepper drivers on the FabScan-Shield V1.1
// It also turns on the light and the laser
// This sketch is not the real FabScan firmware, but just a test script for people who want to test their hardware!
// at startup, the sketch blinks all leds, the light and the motor a few times and after that, it starts to spin the motors.
#include <AccelStepper.h>

#define STEP_US 100

#define LIGHT_PIN 13

//Stepper 1 as labeled on Shield, Turntable
#define ENABLE_PIN_0  4
#define STEP_PIN_0    3
#define DIR_PIN_0     2
#define CWPIN 10
#define CCWPIN 11

#define FAN_PIN 9

#define STDSPEED 20f

#define MODE_IDLE 0
#define MODE_STANDBY 1
#define SPEED1 2
#define SPEED2 3
#define SPEED3 4

int stepMul = 1;

int mode = MODE_IDLE;


bool lastDir = LOW;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN_0, DIR_PIN_0);

void setup()
{
  digitalWrite(ENABLE_PIN_0, HIGH);
  Serial.begin(9600);
#ifdef SWEEP
  Serial.println("SWEEP");
#endif
  pinMode(CWPIN, INPUT_PULLUP);
  pinMode(CCWPIN, INPUT_PULLUP);

  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(STEP_PIN_0, OUTPUT);


  stepper.setEnablePin(ENABLE_PIN_0);

  stepper.setMaxSpeed(500 * stepMul);
  stepper.setAcceleration(1000.0 * stepMul);
}

int lldir = 0;


void loop() {

#ifdef LOGINPUT
  logLoop();
#elifdef SWEEP
  sweepLoop();
#else
  productiveLoop();
#endif


}

void sweepLoop() {
  stepper.move(199 * stepMul);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(500);
  stepper.move(-199 * stepMul);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(500);
}

void logLoop() {
  int dir = 1 - digitalRead(CWPIN) - (1 - digitalRead(CCWPIN));

  Serial.println(dir);
  delay(500);
}

long startmillis = 0;
long stopmillis = 0;

void productiveLoop() {
  int dir = 1 - digitalRead(CWPIN) - (1 - digitalRead(CCWPIN));

  //condition changed block
  if (dir != lldir) {
    lldir = dir;
    startmillis = millis();
    if (dir != 0) {
      enableStepper();
      stepper.setSpeed(dir * STDSPEED * stepMul);
     // mode = SPEED1;
    } else {
      stepper.setSpeed(0);
    }
  }

  //run always block

  if (dir != 0) {
    stepper.runSpeed();
    //after 2 sec go 2x faster
    long milldiff = millis() - startmillis;

    if (milldiff > 2000 && milldiff < 2010) {
      stepper.setSpeed(dir * STDSPEED * 2 * stepMul);
    }

  } else { // if no dir transition to standby and then idle
    standbyStepper();
  }
}


int stepperState = -2;

void enableStepper() {
  if (stepperState != 2) {
    stepperState = 2;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, LOW);
    digitalWrite(LIGHT_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
  }
}
void disableStepper() {
  if (stepperState != 0) {
    stepperState = 0;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, HIGH);
    digitalWrite(LIGHT_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
  }
}

void standbyStepper() {
  if (stepperState != 1) {
    stepperState = 1;
    digitalWrite(ENABLE_PIN_0, LOW);
    pinMode(ENABLE_PIN_0, INPUT);
    digitalWrite(LIGHT_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
  }
}



