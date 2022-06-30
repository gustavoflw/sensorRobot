#include <Ultrasonic.h> // Ultrasonic by Erick Simões
#include "Motor.h"
#include "SerialCom.h"

// IR
int pinSensorAnalogico = A0; // módulo conectado no pino analógico A0

// Ultrasonic
const int pin_trig = PIN_A2, pin_echo = PIN_A3;
Ultrasonic ultrasonic(pin_trig, pin_echo);

// Motor
float wheelL = 0.0, wheelR = 0.0;
const int pin_motorL = 2, pin_motorR = 3;
AF_DCMotor motorL(pin_motorL);
AF_DCMotor motorR(pin_motorR);

// SoftwareSerial
const int pin_RX = A4, pin_TX = A5;
SoftwareSerial mySerial(pin_RX, pin_TX); // RX, TX
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char messageFromESP[numChars] = {0};
boolean newData = false;

/* MILLIS */
unsigned long millis_now      = millis();
unsigned long millis_lastLoop = millis_now;
float         loopRate        = 30.0;
unsigned long loopInterval    = (unsigned long)(1000.0 * 1.0/loopRate); // T = 1/f
unsigned long ultrasonicInterval = 100;
unsigned long millis_lastUltrasonic = millis_now;

void setup() {
  Serial.begin(57600);
  mySerial.begin(57600);

  // Velocidade dos motores
  const int speed = 95;
  motorL.setSpeed(speed);
  motorR.setSpeed(speed);
}

void loop() {
  millis_now = millis();

  if (millis_now - millis_lastLoop < loopInterval)
    return;
  else
    millis_lastLoop = millis();
  
  // Recebe mensagem pela SoftwareSerial
  swSerialReceiveWithMarkers(&mySerial, &newData, receivedChars, numChars);
  
  if (newData == true) {
    // Trata a mensagem da SoftwareSerial
    processNewData(tempChars, receivedChars, messageFromESP,
      &wheelL, &wheelR, &newData);
  }

  // Aciona motores
  activateMotor(&motorL, wheelL, 0);
  activateMotor(&motorR, wheelR, 0);
//  Serial.println("...");

  // Ultrassom e IR
  if (millis_now - millis_lastUltrasonic > ultrasonicInterval) {
    millis_lastUltrasonic = millis_now;
    
    int distance = ultrasonic.read();

    int irColor = 0;
    irColor = analogRead(pinSensorAnalogico);
    
    serialSendWithMarkers(distance, irColor);
    swSerialSendWithMarkers(&mySerial, distance, irColor);
  }
}

// Printa os dados na SERIAL NORMAL
void showParsedData() {
    Serial.print("Message ");
    Serial.println(messageFromESP);
    Serial.print("wheelL ");
    Serial.println(wheelL);
    Serial.print("wheelR ");
    Serial.println(wheelR);
}

// Manda dados do ultrassom na SERIAL NORMAL
void serialSendWithMarkers(int distance, int color)
{
  char buff[32] = {0};
  snprintf(buff, 32, "<Distance, %d, %d>\n", distance, color);
  Serial.write(buff, 32);
}
