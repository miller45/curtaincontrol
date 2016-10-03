//#define TESTMODE
//#define LOGINPUT
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
#define CWPIN 9
#define CCWPIN 10

#define STDSPEED 100

int stepMul = 1;


bool lastDir = LOW;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN_0, DIR_PIN_0);

void setup()
{
  digitalWrite(ENABLE_PIN_0, INPUT);  //STDBY
  Serial.begin(9600);
  pinMode(CWPIN, INPUT_PULLUP);
  pinMode(CCWPIN, INPUT_PULLUP);

  pinMode(LIGHT_PIN, OUTPUT);
  
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(STEP_PIN_0, OUTPUT);

  enableStepper();

  stepper.setEnablePin(ENABLE_PIN_0);

  stepper.setMaxSpeed(500 * stepMul);
  stepper.setAcceleration(1000.0 * stepMul); 
}

int lldir = 0;

#ifdef TESTMODE

void loop(){
   //blink all leds, lights ans the laser 10 times
  for (int i = 0; i < 10000; i++)
  {
    digitalWrite(ENABLE_PIN_0, LOW);  
    digitalWrite(STEP_PIN_0, HIGH);  
    digitalWrite(DIR_PIN_0, LOW);     
    digitalWrite(LIGHT_PIN, HIGH);     
    delay(1000);
    digitalWrite(ENABLE_PIN_0, HIGH);  
    digitalWrite(STEP_PIN_0, LOW);  
    digitalWrite(DIR_PIN_0, HIGH);     
    digitalWrite(LIGHT_PIN, LOW);     
    delay(1000);
  }

}

#else

void loop() {
  int dir = 1 - digitalRead(CWPIN) - (1 - digitalRead(CCWPIN));

#ifdef LOGINPUT  
  Serial.println(dir);
  delay(500);
#else
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
    disableStepper();
  }
#endif

}

#endif


int stepperState = -2;

void enableStepper() {
  if (stepperState != 2) { 
    stepperState = 2;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, LOW);
    digitalWrite(LIGHT_PIN,HIGH);
  }
}
void disableStepper() {
  if (stepperState != 0) {
    stepperState = 0;
    pinMode(ENABLE_PIN_0, OUTPUT);
    digitalWrite(ENABLE_PIN_0, HIGH);
    digitalWrite(LIGHT_PIN,LOW);
  }
}

void standbyStepper() {
  if (stepperState != 1) {
    stepperState = 1;
    digitalWrite(ENABLE_PIN_0, LOW);
    pinMode(ENABLE_PIN_0, INPUT);
    digitalWrite(LIGHT_PIN,HIGH);
  }
}



