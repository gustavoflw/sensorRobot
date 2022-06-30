#include "Motor.h"

// Motor
float wheelL = 0, wheelR = 0;
const int pin_motorL = 2, pin_motorR = 1;
AF_DCMotor motorL(pin_motorL);
AF_DCMotor motorR(pin_motorR);

/* MILLIS */
unsigned long millis_now      = millis();
unsigned long millis_lastLoop = millis_now;
float         loopRate        = 30.0;
unsigned long loopInterval    = (unsigned long)(1000.0 * 1.0/loopRate); // T = 1/f

unsigned long millis_lastMotor = millis_now;
unsigned long millis_motorInterval = 1000;
float defaultWheel = 150;

void setup() {
  Serial.begin(57600);
}

void loop() {
  millis_now = millis();

  if (millis_now - millis_lastLoop < loopInterval)
    return;
  else
    millis_lastLoop = millis();

  if (millis_now - millis_lastMotor > millis_motorInterval) {
    millis_lastMotor = millis_now;

    wheelL = defaultWheel+30;
    wheelR = defaultWheel;

//    if (wheelL == 0.0) {
//      wheelL = defaultWheel;
//      wheelR = defaultWheel;
//    }
//    else if (wheelL < 0.0) {
//      wheelL = defaultWheel;
//      wheelR = defaultWheel;
//    }
//    else {
//      wheelL = -defaultWheel;
//      wheelR = -defaultWheel;
//    }

    activateMotor(&motorL, wheelL);
    activateMotor(&motorR, wheelR);

    Serial.println();
  }
}
