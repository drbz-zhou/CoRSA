
/*
  BLE IMU sketch on projecthub
  -----------------------------
  Arduino MKR 1010 + IMU Shield

  FOR USE WITH WEB APP DEMO AT :
  https://8bitkick.github.io/ArduinoBLE-IMU.html

  IMPORTANT - if required update MKR 1010 fw for bluetooth support first
  See https://forum.arduino.cc/index.php?topic=579306.0

*/

#include <ArduinoBLE.h>
#include "Arduino.h"
#include "Nano33BLEAccelerometer.h"
#include "Nano33BLEPressure.h"
#include <Wire.h>
#include "MAX30105.h"
#include "Adafruit_BME680.h"


// BLE Service
BLEService imuService("917649A0-D98E-11E5-9EEC-0002A5D5C51B"); // Custom UUID

// BLE Characteristic
BLECharacteristic imuCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLENotify, 28); // lengh of packet * 4

long previousMillis = 0;  // last timechecked, in ms

Nano33BLEAccelerometerData accelerometerData;
Nano33BLEPressureData nanoPressureData;

Adafruit_BME680 bme; 

// PulseOximeter
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

float datapacket[7] = {0};
  
void setup() {
  Serial.begin(115200);    // initialize serial communication

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  Accelerometer.begin();
  Pressure.begin();

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // setup bluetooth
  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuCharacteristic);
  BLE.addService(imuService);

  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  // setup bme680
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_1);
  bme.setGasHeater(320, 10); // 320*C for 150 ms
  
  // setup PulseOximeter
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}


// send IMU data
void sendSensorData() {
  long tic = millis();
  

  // read orientation x, y and z eulers
  //IMU.readEulerAngles(eulers[0], eulers[1], eulers[2]);

  Accelerometer.pop(accelerometerData);
  Pressure.pop(nanoPressureData);
  bme.endReading();

  // heartrate
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // calculating beats and spo2 should be carried out at receiver javascript, because sampling rate here isn't ideal

  datapacket[0] = accelerometerData.x;
  datapacket[1] = accelerometerData.y;
  datapacket[2] = accelerometerData.z;
  datapacket[3] = irValue;
  datapacket[4] = redValue;
  datapacket[5] = bme.pressure / 1000.0;
  datapacket[6] = nanoPressureData.barometricPressure;
  bme.beginReading();


  // Send 3x eulers over bluetooth as 1x byte array
  imuCharacteristic.setValue((byte *) &datapacket, 28); // lengh of packet * 4
  long toc = millis() - tic;
  for(int i = 0; i<7; i++){
    Serial.print(datapacket[i]);
    Serial.print(",");
  }
  Serial.println(toc);
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a BLE central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      //sendSensorData();
      // define speed
      if (currentMillis - previousMillis >= 40) {
        previousMillis = currentMillis;
        sendSensorData();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
