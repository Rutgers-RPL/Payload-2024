#include <Arduino.h>
#include<MS5607.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
extern TwoWire Wire1;
MS5607 P_Sens;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() {
  
  Serial.begin(9600); //Baud rate: 9600
 if(!P_Sens.begin()){
    Serial.println("Error in Communicating with sensor, check your connections!");
 }else{
    Serial.println("MS5607 initialization successful!");
  }
    if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
   Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  configureSensor();
}
float P_val,T_val,H_val;

void loop() {
  int sensorValue1 = analogRead(A17);// read the input on analog pin 0:
  float voltage1 = sensorValue1 * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  
  int sensorValue2 = analogRead(A16);
  float voltage2 = sensorValue2*(5.0/1024.0);

  int sensorValue3 = analogRead(A15);
  float voltage3 = sensorValue3*(5.0/1024.0);

    T_val = P_Sens.getTemperature();
    P_val = P_Sens.getPressure();
    H_val = P_Sens.getAltitude();

   /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \trad/s");

  Serial.println("**********************\n");

  Serial.print("Temperature :  ");
  Serial.print(T_val);
  Serial.println(" C");
  Serial.print("Pressure    :  ");
  Serial.print(P_val);
  Serial.println(" mBar");
  Serial.print("Turbidity Sensor 1:" );
  Serial.println(voltage1);
  Serial.print("Turbidity Sensor 2:" );
  Serial.println(voltage2); // print out the value you read:
  Serial.println(" ");
  Serial.print("Turbidity Sensor 3:");
  Serial.println(voltage3); // print out the value you read:
  Serial.println(" ");
  delay(1000);
}