#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>                         // O giroscópio e acelerômetro usa isso
#include <Adafruit_Sensor.h>              // Sensores em geral
#include <Adafruit_MPU6050.h>             // Giroscópio e acelerômetro
#include <geometry_msgs/Vector3.h>        // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
#include <sensor_msgs/Temperature.h>      // https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html

void setupGyro(Adafruit_MPU6050* sensor_gyro)
{
  unsigned long millis_start = millis();
  boolean ok = false;
  
  Serial.print("Gyro: ");
  while (ok == false) {
    if (millis() - millis_start > 5000)
      break;
    Serial.print(".");
    ok = (boolean)sensor_gyro->begin();
    delay(100);
  }
  Serial.print("[");
  Serial.print(ok);
  Serial.println("]");
  
  sensor_gyro->setAccelerometerRange(MPU6050_RANGE_2_G);
  sensor_gyro->setGyroRange(MPU6050_RANGE_250_DEG);
  sensor_gyro->setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void gyroToMessages(geometry_msgs::Vector3* msg_rotation, geometry_msgs::Vector3* msg_acceleration, 
  sensor_msgs::Temperature* msg_temperature, 
  sensors_event_t* offset_rotation, sensors_event_t* offset_acceleration,
  sensors_event_t* rotation, sensors_event_t* acceleration,
  sensors_event_t* temperature)
{
  // x é o "z", y tá certo, z é o "x",
  msg_rotation->x = rotation->gyro.z - offset_rotation->gyro.z;
  msg_rotation->y = rotation->gyro.y - offset_rotation->gyro.y;
  msg_rotation->z = rotation->gyro.x - offset_rotation->gyro.x;
  msg_acceleration->x = acceleration->acceleration.z - offset_acceleration->acceleration.z;
  msg_acceleration->y = acceleration->acceleration.y - offset_acceleration->acceleration.y;
  msg_acceleration->z = acceleration->acceleration.x - offset_acceleration->acceleration.x;
  msg_temperature->temperature = temperature->temperature;

//  msg_rotation->x = rotation->gyro.z;
//  msg_rotation->y = rotation->gyro.y;
//  msg_rotation->z = rotation->gyro.x;
//  msg_acceleration->x = acceleration->acceleration.z;
//  msg_acceleration->y = acceleration->acceleration.y;
//  msg_acceleration->z = acceleration->acceleration.x;
//  msg_temperature->temperature = temperature->temperature;
}

#endif
