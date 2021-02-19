#include <Arduino.h>
// #include <RBDdimmer.h>
#include <dimmable_light.h>
#include <ESP32Encoder.h>

// https://www.arduino.cc/en/Reference/ArduinoBLE

// https://github.com/nkolban/ESP32_BLE_Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
/**
 * GATT Characteristic and Object Type 0x2A6D Pressure (spec says unit is pascals)
 * GATT Unit 0x2780 bar 
 * GATT Unit 0x272F Celsius temperature (degree Celsius)
 * GATT Characteristic and Object Type 0x2A1C Temperature Measurement
 * GATT Characteristic and Object Type 0x2A1D Temperature Type
 */
#define SERVICE_UUID "8fc1ceca-b162-4401-9607-c8ac21383e4e"
#define PRESSURE_CHARACTERISTIC_ID "c14f18ef-4797-439e-a54f-498ba680291d" // standard characteristic for pressure in bars
#define BARS_UNIT_ID "2780"

// Pins (set for ESP32 Devkit-c; see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
const int encoderPin1 = 34;      // clk
const int encoderPin2 = 35;      // dt
const int encoderSwitchPin = 32; //push button switch (sw)
const int pumpZeroCrossPin = 27;
const int pumpControlPin = 14;
// ADC Channel 1 must be used when WiFi or BT is active
const int pressureSensorPin = 33;

/**
 * Pressure Sensor Globals - TODO - check if max V OUT is 4.5 or 5
 * Sensor range is .5 to 4.5V
 * Scaled to 3.3 that .3 to 3V
 * 4095/3.3 = 1240.9 steps per volt
*/
volatile int rawPressure = 0;
volatile float barPressure = 0;
const int minRawPressure = 0;  // 1bar
const int maxRawPressure = 3300; // ~10bar
int rawPressureRange = maxRawPressure - minRawPressure;

volatile int pumpLevel = 0;
volatile int lastPumpLevel = 0;
const int pumpMin = 140;
const int pumpRange = 254 - pumpMin;

volatile int encoderInterruptCount = 0;
const int encoderPulsesPerRev = 40;
const int encoderFullRangeRevs = 1;

bool deviceConnected = false;
bool oldDeviceConnected = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

ESP32Encoder encoder;

//dimmerLamp pumpControl(pumpControlPin, pumpZeroCrossPin); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
DimmableLight light(pumpControlPin);

BLEServer *pServer = NULL;
BLECharacteristic *pPressureCharacteristic = NULL;

// https://github.com/nkolban/ESP32_BLE_Arduino/blob/master/examples/BLE_notify/BLE_notify.ino
class ComGndServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("BLE Server connected");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("BLE Server Disconnected");
    deviceConnected = false;
  }
};
//https://learn.sparkfun.com/tutorials/esp32-thing-plus-hookup-guide/arduino-example-esp32-ble
class PressureCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.println("*********");
      Serial.print("received value: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);

      Serial.println();
    }
    pCharacteristic->setValue("received " + value);
  }
};

void setup()
{
  // put your setup code here, to run once:
  //pumpControl.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  Serial.begin(115200);
  Serial.println("Setup start");

  DimmableLight::setSyncPin(pumpZeroCrossPin);
  DimmableLight::begin();

  Serial.println("Pump Control Configured");

  encoder.attachHalfQuad(encoderPin1, encoderPin2);
  encoder.clearCount();

  Serial.println("Encoder Configured");

  BLEDevice::init("COM-GND Espresso");
  Serial.println("BLE Device Initialized");
  pServer = BLEDevice::createServer();
  Serial.println("BLE Server Initialized");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLE Service Initialized");
  pServer->setCallbacks(new ComGndServerCallbacks());
    Serial.println("BLE Server Callback Initialized");


  // https://www.arduino.cc/en/Reference/ArduinoBLEBLECharacteristicBLECharacteristic
  pPressureCharacteristic = pService->createCharacteristic(
      PRESSURE_CHARACTERISTIC_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  Serial.println("BLE Pressure Characteristic Created");

  pPressureCharacteristic->setCallbacks(new PressureCallbacks());

  Serial.println("BLE Pressure Characteristic Callback Initialized");

  pPressureCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  Serial.println("BLE Service Started");

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  Serial.println("BLE BLEAdvertising Initialized");

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  Serial.println("BLE BLEAdvertising Setup Complete");

  BLEDevice::startAdvertising();
   Serial.println("BLE Advertizing Started");
  Serial.println("BLE Setup Complete");
}

void loop()
{
  int oldCount = encoderInterruptCount;
  encoderInterruptCount = encoder.getCount();

  if (encoderInterruptCount < 0)
  {
    encoder.clearCount();
    encoderInterruptCount = 0;
  }
  else if (encoderInterruptCount > encoderFullRangeRevs * encoderPulsesPerRev)
  {
    encoder.setCount(encoderFullRangeRevs * encoderPulsesPerRev);
    encoderInterruptCount = encoderFullRangeRevs * encoderPulsesPerRev;
  }

  if (encoderInterruptCount != oldCount)
  {
    // convert encoder value to pump PMW range
    float encoderPerc = encoderInterruptCount / (float)(encoderFullRangeRevs * encoderPulsesPerRev);
    pumpLevel = (int)((encoderPerc * (float)pumpRange) + pumpMin);
    //Serial.println(String(encoderInterruptCount) + " - " + String(pumpLevel));
  }

  if (lastPumpLevel != pumpLevel)
  {
    Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    // pumpControl.setPower((pumpLevel));
    light.setBrightness(pumpLevel);
    // TriacDimmer::setBrightness(channel_1, percVal);
    // delay(500);
  }
 
  int lastRawPressure = rawPressure;
  int rawPressure = analogRead(pressureSensorPin);
  if (lastRawPressure != rawPressure)
  {
    int normalizeRawPressure = rawPressure - minRawPressure;
    float rawPressurePerc = (float)((float)normalizeRawPressure / (float)rawPressureRange);
    float barPressure = (rawPressurePerc * 10.0);
    Serial.println(String(barPressure) + " - " + String(rawPressure) );
    delay(250);
    if (deviceConnected)
    {
      pPressureCharacteristic->setValue(barPressure);
      pPressureCharacteristic->notify();
      delay(3);
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  //Serial.println("Pressure: " + String(rawPressure));
  //delay(250);
  // float oldPressure = pressure;
}
