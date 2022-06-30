#include <Ultrasonic.h> // Ultrasonic by Erick Simões

// IR
int pinSensorAnalogico = A0; // módulo conectado no pino analógico A0

/* MILLIS */
unsigned long millis_now      = millis();
unsigned long millis_lastLoop = millis_now;
float         loopRate        = 30.0;
unsigned long loopInterval    = (unsigned long)(1000.0 * 1.0/loopRate); // T = 1/f
unsigned long ultrasonicInterval = 100;
unsigned long millis_lastUltrasonic = millis_now;

int distance = 3;

void setup() {
  Serial.begin(57600);

  pinMode(pinSensorAnalogico, OUTPUT);
  digitalWrite(pinSensorAnalogico, HIGH);
}

void loop() {
  millis_now = millis();

  if (millis_now - millis_lastLoop < loopInterval)
    return;
  else
    millis_lastLoop = millis();

  // Ultrassom e IR
  if (millis_now - millis_lastUltrasonic > ultrasonicInterval) {
    millis_lastUltrasonic = millis_now;

    int irAnalog = analogRead(pinSensorAnalogico);
    char irColor;
    if (irAnalog < 500)
      irColor = 'w';
    else
      irColor = 'b'; 

    Serial.println(irAnalog);
    serialSendWithMarkers(distance, irColor);
  }
}

// Manda dados do ultrassom na SERIAL NORMAL
void serialSendWithMarkers(int distance, char color)
{
  char buff[32] = {0};
  snprintf(buff, 32, "<Distance, %d>, %c\n", distance, color);
  Serial.write(buff, 32);
}
