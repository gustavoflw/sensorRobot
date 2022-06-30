#ifndef MOTOR_H
#define MOTOR_H

#include <AFMotor.h> // Adafruit Motor shield library (af_motor)

/* ATUAÇÃO DO MOTOR
 *  - wheelSpeed é o valor de wheelL ou wheelR, no intervalo [-255, 255] */
void activateMotor(AF_DCMotor* motor, float wheelSpeed)
{
  // Corrige para ficar no maximo com módulo de 255
  if (wheelSpeed > 255.0)
    wheelSpeed = 255.0;
  else if (wheelSpeed < -255.0)
    wheelSpeed = -255.0;
  
  // Calcula módulo
  int intWheelSpeed = (int)wheelSpeed;
  int positiveWheelSpeed = intWheelSpeed;
  if (positiveWheelSpeed < 0)
    positiveWheelSpeed = -positiveWheelSpeed;

  Serial.print("intWheelSpeed, positiveWheelSpeed: ");
  Serial.print(intWheelSpeed);
  Serial.print(", ");
  Serial.println(positiveWheelSpeed);
  
  // Define a velocidade do motor
  motor->setSpeed(positiveWheelSpeed);

  // Determina se é para ir para frente, trás ou parar
  if (intWheelSpeed == 0)
    motor->run(RELEASE);
  else if (wheelSpeed < 0)
    motor->run(BACKWARD);
  else
    motor->run(FORWARD);
}

#endif
