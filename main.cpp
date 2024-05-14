#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <SD.h>

File dataFile;

char fileName[13]={0};  //8.3 format uses 12 bytes+zero terminator

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

float count = 0;

void configureSensor(void)
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() {
  
  Serial.begin(9600); //Baud rate: 9600

  Serial.print("Initializing SD Card...");
  if (!SD.begin(BUILTIN_SDCARD)){
    Serial.print("Initialization Failed");
  }
  Serial.print("Initialization Done");

int fileNumber = 1;

     if(!lsm.begin())
   {
  /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
     while(1);
   }
   Serial.println(F("Found LSM9DS0 9DOF"));

   configureSensor();

  Serial.println("CLEARDATA"); //clears up any data left from previous projects
  Serial.println("T+ \tTurb 1 \tTurb 2 \tTurb 3 \tTemp \tAccel X \tAccel Y \tAccel Z \tGyro X \tGyro Y \tGyro Z"); //always write LABEL, to indicate it as first line
  Serial.println("RESETTIMER");

   do
  {
    // check if we've used all filenumbers; if so, display message and halt forever
    if(fileNumber == 255)
    {
      Serial.println("Out of file numbers");
      for(;;);
    }

    // create a file name logNNN.txt
    sprintf(fileName,"log%03d.txt", fileNumber++);

    // and check
  } while(SD.exists(fileName));

  dataFile = SD.open(fileName, FILE_WRITE);

   //if the file opened ok, write to it:
    if (dataFile) {
    Serial.println("File opened ok");
      // print the headings for our data
   dataFile.println("T+ \tTurb 1 \tTurb 2 \tTurb 3 \tTemp \tX \tY \tZ \tG X \tG Y \tG Z");
    }
   dataFile.close();
}


void loop() {

  int sensorValue1 = analogRead(A17);// read the input on analog pin 0:
  float voltage1 = sensorValue1 * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  
  int sensorValue2 = analogRead(A16);
  float voltage2 = sensorValue2*(5.0/1024.0);

  int sensorValue3 = analogRead(A15);
  float voltage3 = sensorValue3*(5.0/1024.0);

  
    /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  dataFile = SD.open(fileName, FILE_WRITE);

   // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(count); //Store date on SD card
    dataFile.print("\t"); //Move to next column using a ","

    dataFile.print(voltage1); //Store date on SD card
    dataFile.print("\t"); //Move to next column using a ","

    dataFile.print(voltage2);
    dataFile.print("\t");

    dataFile.print(voltage3);
    dataFile.print("\t");

    dataFile.print(temp.temperature);
    dataFile.print("\t");

    dataFile.print(accel.acceleration.x);
    dataFile.print("\t");

    dataFile.print(accel.acceleration.y);
    dataFile.print("\t");

    dataFile.print(accel.acceleration.z);
    dataFile.print("\t");

    dataFile.print(gyro.gyro.x);
    dataFile.print("\t");

    dataFile.print(gyro.gyro.y);
    dataFile.print("\t");

    dataFile.print(gyro.gyro.z);
    dataFile.print("\t");

    dataFile.println(); //End of Row move to next row
    dataFile.close(); //Close the file
  }
  else
    Serial.println("OOPS!! SD card writing failed");


  // print out acceleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \trad/s");

  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  Serial.print("Turbidity Sensor 1:" );
  Serial.println(voltage1);
  Serial.print("Turbidity Sensor 2:" );
  Serial.println(voltage2); // print out the value you read:
  Serial.println(" ");
  Serial.print("Turbidity Sensor 3:");
  Serial.println(voltage3); // print out the value you read:
  Serial.println(" ");

  
delay(500);

count = count + 0.5;

}