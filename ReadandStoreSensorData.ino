// last worked on March. 26, 2019 2:00pm by David Onak
//UPDATE: March. 27, 2019 8:00pm by David Onak

#define LED 9
#define SWITCH 8
#define CARD 10
#define SEALEVELPRESSURE_HPA (1017.0)

//bme sensor libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;

//library for sd card
#include <SD.h>
File sensorData;

int lastSwitchState = 0;
int startTime = 0;
int timeOff = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(CARD, OUTPUT);

  //serial print sensor status
  while (!Serial); // wait for serial to connect
  Serial.println(F("Sensor Readings Commencing"));

  //SD card status
  Serial.print("Initializing SD card...");
  while (!SD.begin(CARD)) { // will make rapid blinks for SDcard connection error
    Serial.println("initialization failed!");
    //LED indicator
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
  Serial.println("initialization done.");

  // Initialize BME680
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println(F("Sensor connected"));
}

void loop() {
  int switchState = digitalRead(SWITCH);

  if (switchState) { // while removable wire is pulled out (enabled) 

    //read for SD card file
    sensorData = SD.open("data.txt", FILE_WRITE);

    if (sensorData) { // file could be opened

      //check if sensor is picking up readings
      if (bme.performReading()) {
        Serial.println(bme.temperature);
        } else {
        // Could not perform reading
        // Will make rapid blinks for sensor connection error
        Serial.println("Failed to perform reading :(");
        //LED indicator
        digitalWrite(LED, HIGH);
        delay(200);
        digitalWrite(LED, LOW);
        delay(200);
        }
        
      //process to add a key
      if (lastSwitchState != switchState) {
        // If switch state has changed

        if (switchState) {
          Serial.print("Writing key to data.txt...");
          sensorData.println("Temp (degC), Humidity (%), Pressure (hPa), Gas (kOhm), Time(s)");
          Serial.println("done.");
          startTime = millis() / 1000;
        }
      }

      bme.performReading();
      Serial.print("Writing to data.txt...");
      sensorData.print(bme.temperature);
      sensorData.print(",");
      sensorData.print(bme.humidity);
      sensorData.print(",");
      sensorData.print(bme.pressure / 100.0);
      sensorData.print(",");
      sensorData.print(bme.gas_resistance / 1000);
      sensorData.print(",");
      sensorData.println((millis() / 1000) - startTime);
      // close the file:
      sensorData.close();
      Serial.println("done.");
    } else {
      //if the file didn't open
      Serial.println("error opening data.txt");
    }
    delay(1000);
  } else { // while removable wire is not pulled out (disabled)
    Serial.println("Switch has been turned off");
    //LED indicator
    //if less than 5 seconds passed by
    if (((millis() / 1000) - timeOff) < 5) {
      //do nothing
    } else {
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
      timeOff = millis() / 1000;
    }
  }

  lastSwitchState = switchState;
  
  // serial monitor print of read data from sensors
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
}
