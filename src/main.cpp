#include <Arduino.h>
#include <dimmable_light.h>
#include <ESP32Encoder.h>

#include <PID_v1.h>

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

#define ENC_PRESSURE_MODE 0
#define ENC_POWER_MODE 1

// Pins (set for ESP32 Devkit-c; see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
const int encoderPin1 = 34;      // clk
const int encoderPin2 = 35;      // dt
const int encoderSwitchPin = 32; // push button switch (sw)
const int pumpZeroCrossPin = 27; // zc
const int pumpControlPin = 14;
// ADC Channel 1 must be used when Esp32 WiFi or BT is active
const int pressureSensorPin = 33;

/**
 * Pressure Sensor Globals
 * Sensor range is .5 to 4.5V
 * TODO - Work out the scaling from the Resistor Divider
*/
volatile int rawPressure = 0;
volatile float barPressure = 0;
const int minRawPressure = 0;    // 1bar
const int maxRawPressure = 3300; // ~10bar
int rawPressureRange = maxRawPressure - minRawPressure;

/**
 * Pump Variac Globals
 */
// Initial pump power
volatile int pumpLevel = 0;
volatile int lastPumpLevel = 0;
// The minimum value below which the pump cuts-out
const int pumpMin = 140;
// DimmableLight value range is 0 - 255 - but Robotdyn seems to cut out at 100% (255)
const int pumpMax = 254;
const int pumpRange = pumpMax - pumpMin;
DimmableLight pump(pumpControlPin);

/**
 * Rotary Encoder Globals
 */
// The numder of pulses produced by one full revolution
const int encoderPulsesPerRev = 40;
// The number of revolutions required for full output range
const int encoderFullRangeRevs = 2;
// initial encoder value - set to max for initializing at full pressure
volatile int encoderCount = encoderFullRangeRevs * encoderPulsesPerRev;
// Mode flag to set if the encoder controls the target Pressure or directly sets the pump power output
const int encoderMode = ENC_PRESSURE_MODE;

ESP32Encoder encoder;

/**
 * Bluetooth Globals
 */
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
BLECharacteristic *pPressureCharacteristic = NULL;

/**
 * Pressure PID Globals
 * https://playground.arduino.cc/Code/PIDLibrary/
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
double Setpoint, Input, Output;

PID pressurePID(&Input, &Output, &Setpoint, 3.75, 6, 1, P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used


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

  // PID
  Setpoint = 10; // bars
  pressurePID.SetOutputLimits(pumpMin, pumpMax);
  pressurePID.SetMode(AUTOMATIC);
}

void loop()
{
  int oldEncoderCount = encoderCount;
  encoderCount = encoder.getCount();

  if (encoderCount < 0)
  {
    encoder.clearCount();
    encoderCount = 0;
  }
  else if (encoderCount > encoderFullRangeRevs * encoderPulsesPerRev)
  {
    encoder.setCount(encoderFullRangeRevs * encoderPulsesPerRev);
    encoderCount = encoderFullRangeRevs * encoderPulsesPerRev;
  }

  if (encoderCount != oldEncoderCount)
  {
    // convert encoder value to percentage
    float encoderPerc = encoderCount / (float)(encoderFullRangeRevs * encoderPulsesPerRev);
    
    if(encoderMode == ENC_POWER_MODE) {
      // in power mode, the encoder set's the pump power directly
      pumpLevel = (int)((encoderPerc * (float)pumpRange) + pumpMin);
    } else {
      // In Pressure mode, the encoder sets the target Pressure for the PID
      Setpoint = encoderPerc * 10.0;
    }
    //Serial.println(String(encoderCount) + " - " + String(pumpLevel));
  }

  if(encoderMode == ENC_PRESSURE_MODE) {
    pumpLevel = Output;
  }

  if (lastPumpLevel != pumpLevel)
  {
    // Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    pump.setBrightness(pumpLevel);
  }

  int lastRawPressure = rawPressure;
  int rawPressure = analogRead(pressureSensorPin);
  if (lastRawPressure != rawPressure)
  {
    int normalizeRawPressure = rawPressure - minRawPressure;
    float rawPressurePerc = (float)((float)normalizeRawPressure / (float)rawPressureRange);
    float barPressure = (rawPressurePerc * 10.0);
    Serial.println("Sp: " + String(Setpoint) + " O: " + String(Output) + " I: " + String(Input));
     delay(50);
    if (deviceConnected)
    {
      pPressureCharacteristic->setValue(barPressure);
      pPressureCharacteristic->notify();
      delay(3);
    }
    Input = barPressure;

  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
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

  pressurePID.Compute();
  
}
