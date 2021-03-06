#include <Arduino.h>
/**
 * NOTE: The mains frequency must be set in ./lib/Dimmable-Light-Arduino-master/src/thyristor.h
 * It has been set 60hz for this applicatio.
 */
#include <dimmable_light.h>

#include <PID_v1.h>

// https://github.com/nkolban/ESP32_BLE_Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "pump-module.h"
#include "rotary-encoder-module.h"

// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
/**
 * GATT Characteristic and Object Type 0x2A6D Pressure (spec says unit is pascals)
 * GATT Unit 0x2780 bar 
 * GATT Unit 0x272F Celsius temperature (degree Celsius)
 * GATT Characteristic and Object Type 0x2A1C Temperature Measurement
 * GATT Characteristic and Object Type 0x2A1D Temperature Type
 */
#define SERVICE_UUID "8fc1ceca-b162-4401-9607-c8ac21383e4e"
#define PRESSURE_SENSOR_CHAR_ID "c14f18ef-4797-439e-a54f-498ba680291d" // BT read-only characteristic for pressure sensor value in bars
#define PRESSURE_TARGET_CHAR_ID "34c242f1-8b5f-4d99-8238-4538eb0b5764" // BT read/write characteristic for target pressure in bars
#define PUMP_POWER_CHAR_ID "d8ad3645-50ad-4f7a-a79d-af0a59469455" // BT read-onlt characteristic for pumps power level

#define BARS_UNIT_ID "2780"

#define ENC_PRESSURE_MODE 0
#define ENC_POWER_MODE 1

/** 
 * Pins 
 * set-up for ESP32 Devkit-c
 * see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
 */
const unsigned char encoderPin1 = 34;      // clk
const unsigned char encoderPin2 = 35;      // dt
const unsigned char encoderSwitchPin = 32; // push button switch (sw)
const unsigned char pumpZeroCrossPin = 27; // zc
const unsigned char pumpControlPin = 14;
// ADC Channel 1 must be used when Esp32 WiFi or BT is active
const unsigned char pressureSensorPin = 33;

/**
 * Pressure Sensor Globals
 * Sensor range is .5 to 4.5V
 * TODO - Work out the scaling from the Resistor Divider
*/
int rawPressure = 0;
float lastBarPressure = 0;
float barPressure = 0;
const int minRawPressure = 50;   // 1bar
const int maxRawPressure = 3300; // ~10bar
int rawPressureRange = maxRawPressure - minRawPressure;

/**
 * Pump Variac Globals
 */
// Initial pump power
int pumpLevel = 0;
int lastPumpLevel = 0;
bool pumpPowerIsOn = false;
// The minimum value below which the pump cuts-out
const int pumpMin = 125;
// DimmableLight value range is 0 - 255 - but Robotdyn seems to cut out at 100% (255)
const int pumpMax = 254;
const int pumpRange = pumpMax - pumpMin;

PumpModule pump(pumpZeroCrossPin, pumpControlPin);

/** 
 * Encoder Globals
 */
float encoderPosition = 0;
float lastEncoderPosition = 0;
int encoderMode = ENC_PRESSURE_MODE;

RotaryEncoderModule rotaryEncoder(encoderPin1, encoderPin2, encoderSwitchPin);

/**
 * Bluetooth Globals
 */
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
BLECharacteristic *pPressureSensorBLEChar = NULL;
BLECharacteristic *pPressureTargetBLEChar = NULL;
BLECharacteristic *pPumpPowerBLEChar = NULL;

/**
 * Pressure PID Globals
 * https://playground.arduino.cc/Code/PIDLibrary/
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */
double PidSetpoint = 0;
double PidInput = 0;
double PidOutput = 0;

PID pressurePID(&PidInput, &PidOutput, &PidSetpoint, 3.75, 6, 1, P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

class RotartEncodeCallbacks : public RotaryEncoderModuleCallbacks
{
  void onButtonDown(RotaryEncoderModule *module)
  {
    Serial.println("onButtonDown");
  }
  void onButtonUp(RotaryEncoderModule *module)
  {
    Serial.println("onButtonUp");
  }
  void onEncoderChange(RotaryEncoderModule *module)
  {
    encoderPosition = module->getPercent();
    // float perc = module->getPercent();
    // Serial.println("onEncoderChange: " + String(raw) + " - " + String(perc));
    // PidSetpoint = perc * 10.0;
    // pPressureTargetBLEChar->setValue(PidSetpoint);
    // pPressureTargetBLEChar->notify();
    // delay(3);
  }
};

class PumpCallbacks : public PumpModuleCallbacks
{
  void onPowerOn(PumpModule *module)
  {
    Serial.println("onPowerOn");
    pumpPowerIsOn = true;
    // rotaryEncoder.setPercent(1);
    PidOutput = pumpMax;
    pressurePID.SetMode(AUTOMATIC);
  }

  void onPowerOff(PumpModule *module)
  {
    Serial.println("onPowerOff");
    pumpPowerIsOn = false;
    PidOutput = 0;
    // rotaryEncoder.setPercent(0);
    pressurePID.SetMode(MANUAL);
    // pPumpPowerBLEChar->setValue(0);
    // pPressureTargetBLEChar->notify();
  }
};

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
class PressureSensorBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.print("received value: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);

      Serial.println();
    }
    // pCharacteristic->setValue("received " + value);
  }
};
class PressureTargetBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.print("Write Target Pressure: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);

      Serial.println();
    }
    // pCharacteristic->setValue("received " + value);
  }
};

void setEncoderMode(int mode)
{
  if (mode == ENC_POWER_MODE)
  {
    encoderMode = ENC_POWER_MODE;
    pressurePID.SetMode(MANUAL);
  }
  else
  {
    encoderMode = ENC_PRESSURE_MODE;
    pressurePID.SetMode(AUTOMATIC);
  }
}
void setup()
{

  Serial.begin(115200);
  Serial.println("Setup start");

  pump.setCallbacks(new PumpCallbacks());
  pump.begin();
  pump.setPumpPercent(1);

  Serial.println("Pump Control Configured");

  BLEDevice::init("COM-GND Espresso");
  Serial.println("BLE Device Initialized");
  pServer = BLEDevice::createServer();
  Serial.println("BLE Server Initialized");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLE Service Initialized");
  pServer->setCallbacks(new ComGndServerCallbacks());
  Serial.println("BLE Server Callback Initialized");

  // https://www.arduino.cc/en/Reference/ArduinoBLEBLECharacteristicBLECharacteristic
  pPressureSensorBLEChar = pService->createCharacteristic(
      PRESSURE_SENSOR_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Pressure Sensor Characteristic Created");
  // pPressureSensorBLEChar->setCallbacks(new PressureSensorBLECharCallbacks());
  Serial.println("BLE Pressure Sensor Characteristic Callback Initialized");
  pPressureSensorBLEChar->addDescriptor(new BLE2902());

  pPressureTargetBLEChar = pService->createCharacteristic(
      PRESSURE_TARGET_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Pressure Target Characteristic Created");
  pPressureTargetBLEChar->setCallbacks(new PressureTargetBLECharCallbacks());
  Serial.println("BLE Pressure Target Characteristic Callback Initialized");
  pPressureTargetBLEChar->addDescriptor(new BLE2902());

  pPumpPowerBLEChar = pService->createCharacteristic(
      PUMP_POWER_CHAR_ID,
      BLECharacteristic::PROPERTY_READ);
  Serial.println("BLE Pump Power Characteristic Created");
  pPumpPowerBLEChar->addDescriptor(new BLE2902());

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

  // Rotary Encoder
  rotaryEncoder.setCallbacks(new RotartEncodeCallbacks());
  rotaryEncoder.setPercent(1);
  rotaryEncoder.begin();

  // PID
  PidSetpoint = 10; // bars
  pressurePID.SetOutputLimits(pumpMin, pumpMax);
  pressurePID.SetMode(pump.getPowerIsOn() ? AUTOMATIC : MANUAL);
}

/**
 * Main Loop
 */
void loop()
{

  // Handle Encoder
  bool bleNotified = false;
  if(lastEncoderPosition != encoderPosition) {
    // Note: encoderPosition is set in RotartEncodeCallbacks
    lastEncoderPosition = encoderPosition;
    Serial.println("encoder: " + String(encoderPosition));
    float scaledTargetPressure = encoderPosition * 10.0;
    PidSetpoint = scaledTargetPressure;
    pPressureTargetBLEChar->setValue(scaledTargetPressure);
    pPressureTargetBLEChar->notify();
    bleNotified = true;
  }

  // Handle Pressure Sensor

  int rawPressure = analogRead(pressureSensorPin);
  int normalizeRawPressure = rawPressure - minRawPressure;
  float rawPressurePerc = (float)((float)normalizeRawPressure / (float)rawPressureRange);
  lastBarPressure = barPressure;

  // Calculate the bar pressure, rounding two 2 decimals
  // TODO: this seems to result in a float like 12.1200000012345
  barPressure = roundf(rawPressurePerc * 10.0 * 100.0) / 100.0;

  if (lastBarPressure != barPressure)
  {
    if (deviceConnected)
    {
      pPressureSensorBLEChar->setValue(barPressure);
      pPressureSensorBLEChar->notify();
      bleNotified = true;
    }
    // Pressure is not a useful input value when there is no resistance to the pump.
    // e.g. when the portafilter is empty, the pressure will be approaching 0, as flow approaches pump maximum.
    // ideally the system would model flow rate rather than pressure to solve this.
    // For now, when the system pressure is below a set point, we *very* roughly estimate the flow rate
    // The pump has specifications for flow at a given pressure, but not for a variable power supply
    // TODO: characterize pump flow at various power levels - this can be done once a scale
    // is integrated to measure water output accross different power levels.
    const float transitionPressure = .2;
    if (rawPressurePerc < transitionPressure)
    {
      /**
       * Note that the PID is in P_ON_M mode
       * see: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
       * (p is a resistive force)
       * TODO: only setTuning on initial transition below .2
       */
      // pressurePID.SetTunings(3.85, 5.6, .55);
      pressurePID.SetTunings(3, 8, .55);

      // under 2 bars, transition to a roughly estimated flow-rate
      // flow will be estimated as the inverse of pressure * the pump power %
      float pumpPerc = pump.getPowerIsOn() ? pump.getPumpPercent() : 0;
      float estFlow = (1 - rawPressurePerc) * pumpPerc;
      // create a differential to increase weight of flow vs pressure on PID PidInput, as pressure drops
      float delta = transitionPressure - rawPressurePerc;
      float nomalizedDelta = 1 / transitionPressure * delta; // scale delta to 0 - 1;
      float blendedInput = (nomalizedDelta * estFlow) + ((1 - nomalizedDelta) * rawPressurePerc);
      //Serial.println("out: " + String(PidOutput) + " bar: " + String(barPressure) + " flow: " + estFlow + " pump: " + String(pumpPerc) + " delta: " + String(nomalizedDelta) + " input: " + blendedInput);

      PidInput = blendedInput * 10.0;
    }
    else
    {
      // when pressure is above set point, let the PID act on pressure alone
      //Serial.println("Sp: " + String(PidSetpoint) + " O: " + String(PidOutput) + " I: " + String(PidInput) + " B: " + String(rawPressurePerc));
      pressurePID.SetTunings(3.75, 6, 1);
      PidInput = barPressure;
    }

    Serial.println("pSp: " + String(PidSetpoint) + " pOut: " + String(PidOutput) + " pIn: " + String(PidInput) + " Bar: " + String(rawPressurePerc));

  }
  
  // Handle Pump

  if (encoderMode == ENC_PRESSURE_MODE)
  {
    if(pumpPowerIsOn) {
      pumpLevel = PidOutput;
    } else {
      pumpLevel = 0;
    }
  }
  if (lastPumpLevel != pumpLevel)
  {
    // Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    pump.setPumpLevel(pumpLevel);
    float pumpPerc = pump.getPumpPercent();
    pPumpPowerBLEChar->setValue(pumpPerc);
    pPressureTargetBLEChar->notify();
     bleNotified = true;
  }

  // Handle BT disconnecting

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

  // loop as fast as possible, but give ble change to catchup if anything changed
  // TODO - consider using a xTask to notify ble characteristics at a slower rate
  if(bleNotified) {
    delay(50);
  }
  bleNotified = false;
  
}
