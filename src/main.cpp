#include <Arduino.h>
/**
 * NOTE: The mains frequency must be set in ./lib/Dimmable-Light-Arduino-master/src/thyristor.h
 * It has been set 60hz for this application.
 */
#include <dimmable_light.h>
#include <QuickPID.h>
#include <ESP32AnalogRead.h>

// https://github.com/nkolban/ESP32_BLE_Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "pump-module.h"
#include "rotary-encoder-module.h"
#include "pressure-m323-module.h"
#include "temperature-ntc-module.h"

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
#define PUMP_POWER_CHAR_ID "d8ad3645-50ad-4f7a-a79d-af0a59469455"      // BT read-onlt characteristic for pumps power level

#define BARS_UNIT_ID "2780"

#define ENC_PRESSURE_MODE 0
#define ENC_POWER_MODE 1

/** 
 * Pins 
 * set-up for ESP32 Devkit-c
 * see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
 */
const unsigned char encoderPin1 = 25;      // clk
const unsigned char encoderPin2 = 26;      // dt
const unsigned char encoderSwitchPin = 27; // push button switch (sw)
const unsigned char pumpZeroCrossPin = 14; // zc
const unsigned char pumpControlPin = 12;
// ADC Channel 1 must be used when Esp32 WiFi or BT is active
const unsigned char pressureSensorPin = 33;

const unsigned char boilerTempSensorPin = 32;
/**
 * Pressure Sensor Globals
*/
PressureM323Module pressureSensor(pressureSensorPin, 60);

/**
 * Boiler External NTC Temperature Sensor
 */
TemperatureNtcModule externalBoilerTempSensor(boilerTempSensorPin);

// ESP32AnalogRead adc;
// int rawPressure = 0;
float lastBarPressure = 0;
float barPressure = 0;
// const int minRawPressure = 37;   // .37v = 0bar
// const int maxRawPressure = 3300; // 3.3v = ~10bar
// int rawPressureRange = maxRawPressure - minRawPressure;

/**
 * Pump Variac Globals
 */
// Initial pump power

PumpModule pump(pumpZeroCrossPin, pumpControlPin);

int pumpLevel = 0;
int lastPumpLevel = 0;
bool pumpPowerIsOn = false;
// The minimum value below which the pump cuts-out
// const int pumpMin = 100;
// DimmableLight value range is 0 - 255 - but Robotdyn seems to cut out at 100% (255)
// const int pumpMax = 254;
// const int pumpRange = pumpMax - pumpMin;

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
float blePressureTarget = 0;
float lastBlePressureTarget = 0;
bool blePressureSensorNotifyFlag = false;
bool blePressureTargetNotifyFlag = false;
bool blePumpPowerNotifyFlag = false;
/**
 * Pressure PID Globals
 * https://playground.arduino.cc/Code/PIDLibrary/
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * https://github.com/Dlloydev/QuickPID
 * https://www.crossco.com/resources/technical/how-to-tune-pid-loops/
 */
int pressurePidSetpoint = 0;
int pressurePidInput = 0;
int pressurePidOutput = 0;
// Proportional Gain - Dependant on pOn value. A mix of proportional response to measurement vs error
// PoM: higher value increases conservativeness
float lowPressureP = 4.0;
float highPressureP = 5.0; // testing: 1.5;
                           // Integral Gain
float lowPressureI = 1;
float highPressureI = 12.0; // testing: 2.0
// Derivitative Gain
float lowPressureD = 1;
float highPressureD = 1; // testing: 0
// Ratio of Proportional on Measurement vs Proportional on Error
// 0 = 100% PoM, 1 = 100% PoE
// see: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
float lowPressurePon = 0.2;
float highPressurePon = 0.1; // testing: .25

QuickPID pressurePID(
    &pressurePidInput,
    &pressurePidOutput,
    &pressurePidSetpoint,
    highPressureP,   // Kp
    highPressureI,   // Ki
    highPressureD,   // Kd
    highPressurePon, // POn - Proportional on Error weighting. O = 100% Proportional on Measurement, 1 = 100% Proportional on Error
    (uint8_t)DIRECT);

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
    // pressurePidSetpoint = perc * 10.0;
    // pPressureTargetBLEChar->setValue(pressurePidSetpoint);
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
    // pressurePidOutput = pumpMax;
    pressurePID.SetMode(AUTOMATIC);
  }

  void onPowerOff(PumpModule *module)
  {
    Serial.println("onPowerOff");
    pumpPowerIsOn = false;
    // pressurePidOutput = 0;
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
  void onRead(BLECharacteristic *pCharacteristic)
  {
  }
  // void onWrite(BLECharacteristic *pCharacteristic)
  // {
  //   std::string value = pCharacteristic->getValue();

  //   if (value.length() > 0)
  //   {
  //     Serial.print("received value: ");
  //     for (int i = 0; i < value.length(); i++)
  //       Serial.print(value[i]);

  //     Serial.println();
  //   }
  // pCharacteristic->setValue("received " + value);
  //}
};
class PressureTargetBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    String strValue;

    if (value.length() > 0)
    {
      Serial.print("Recieved BT Target Pressure (" + String(value.length()) + "): ");

      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(value[i]);
        strValue += (char)value[i];
      }
      Serial.println("---");
      Serial.println(strValue);
    }

    // convert the string to a float value
    float fValue = strtof(strValue.c_str(), NULL);
    // Serial.println("float: " + String(fValue));

    blePressureTarget = fValue;

    // pCharacteristic->setValue("received " + value);
  }
};

void bleNotifyTask(void *params)
{
  for (;;)
  {
    if (blePressureSensorNotifyFlag)
    {
      pPressureSensorBLEChar->notify();
      blePressureSensorNotifyFlag = false;
    }
    if (blePressureTargetNotifyFlag)
    {
      pPressureTargetBLEChar->notify();
      blePressureTargetNotifyFlag = false;
    }
    if (blePumpPowerNotifyFlag)
    {
      pPumpPowerBLEChar->notify();
      blePumpPowerNotifyFlag = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

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

/**
 * Main Setup
 */
void setup()
{

  Serial.begin(115200);
  Serial.println("Setup start");
  Serial.begin(115200);

  // adc.attach(pressureSensorPin);

  pump.setCallbacks(new PumpCallbacks());
  pump.begin();
  pump.setPumpPercent(1);

  Serial.println("Pump Module Configured");

  pressureSensor.begin();
  Serial.println("Pressure Sensor Module Configured");

  // External Boiler Temperature
  externalBoilerTempSensor.begin();
  Serial.println("Temperature Sensor Module Configured");

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
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
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

  // Pressure PID
  pressurePidSetpoint = 10 * 100; // bars
  pressurePID.SetOutputLimits(pump.getPumpMin() * 100, pump.getPumpMax() * 100);
  pressurePID.SetMode(pump.getPowerIsOn() ? AUTOMATIC : MANUAL);

  xTaskCreate(bleNotifyTask, "bleNotify", 5000, NULL, 1, NULL);
}

/**
 * Main Loop
 */
void loop()
{

  // Handle Encoder
  if (lastEncoderPosition != encoderPosition)
  {
    // Note: encoderPosition is set in RotartEncodeCallbacks
    lastEncoderPosition = encoderPosition;
    Serial.println("encoder: " + String(encoderPosition));
    float scaledTargetPressure = encoderPosition * 10.0;
    pressurePidSetpoint = (int)(scaledTargetPressure * 100.0);
    pPressureTargetBLEChar->setValue(scaledTargetPressure);
    // pPressureTargetBLEChar->notify();
    blePressureTargetNotifyFlag = true;
  }

  // Handle Ble Target Pressure Changes
  if (lastBlePressureTarget != blePressureTarget)
  {
    lastBlePressureTarget = blePressureTarget;
    pressurePidSetpoint = (int)(lastBlePressureTarget * 100);
    rotaryEncoder.setPercent(lastBlePressureTarget / 10.0);
    pPressureTargetBLEChar->setValue(lastBlePressureTarget);
    // pPressureTargetBLEChar->notify();
    blePressureTargetNotifyFlag = true;
  }
  // Handle Pressure Sensor

  // int rawPressure = analogRead(pressureSensorPin);
  // int rawPressure = readPressureMultisample();
  // int normalizeRawPressure = rawPressure - minRawPressure;
  // float rawPressurePerc = (float)((float)normalizeRawPressure / (float)rawPressureRange);

  lastBarPressure = barPressure;

  // Calculate the bar pressure, rounding two 3 decimals
  // TODO: this seems to result in a float like 12.1200000012345
  // barPressure = roundf(rawPressurePerc * 10.0 * 1000.0) / 1000.0;
  barPressure = pressureSensor.getPressureBars();

  if (lastBarPressure != barPressure)
  {
    if (deviceConnected)
    {
      pPressureSensorBLEChar->setValue(barPressure);
      // pPressureSensorBLEChar->notify();
      blePressureSensorNotifyFlag = true;
    }
    // Pressure is not a useful input value when there is no resistance to the pump.
    // e.g. when the portafilter is empty, the pressure will be approaching 0, as flow approaches pump maximum.
    // ideally the system would model flow rate rather than pressure to solve this.
    // For now, when the system pressure is below a set point, we *very* roughly estimate the flow rate
    // The pump has specifications for flow at a given pressure, but not for a variable power supply
    // TODO: characterize pump flow at various power levels - this can be done once a scale
    // is integrated to measure water output accross different power levels.

    const float transitionPressure = .25; // 1/4 bar
    if (barPressure < transitionPressure)
    {
      /**
       * Note that the PID is in P_ON_M mode
       * see: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
       * (p is a resistive force)
       * TODO: only setTuning on initial transition below .2
       */
      // float pressurePerc = pressureSensor.getPressurePercent();

      // pressurePID.SetTunings(3, 8, .55, 0);

      // // flow will be estimated as the inverse of pressure * the pump power %
      // float pumpPerc = pump.getPowerIsOn() ? pump.getPumpPercent() : 0;

      // float estFlow = (1 - pressurePerc) * pumpPerc;
      // // create a differential to increase weight of flow vs pressure on PID pressurePidInput, as pressure drops
      // float delta = transitionPressure - pressurePerc;
      // float nomalizedDelta = 1 / transitionPressure * delta; // scale delta to 0 - 1;
      // float blendedInput = (nomalizedDelta * estFlow) + ((1 - nomalizedDelta) * pressurePerc);
      // //Serial.println("out: " + String(pressurePidOutput) + " bar: " + String(barPressure) + " flow: " + estFlow + " pump: " + String(pumpPerc) + " delta: " + String(nomalizedDelta) + " input: " + blendedInput);

      // pressurePidInput = (int)(blendedInput * 10.0 * 100.0);

      pressurePID.SetMode(MANUAL);
      pressurePidInput = (int)(barPressure * 100.0);
    }
    else if (barPressure < 5)
    {
      pressurePID.SetMode(AUTOMATIC);
      pressurePID.SetTunings(lowPressureP, lowPressureI, lowPressureD, lowPressurePon);
      pressurePidInput = (int)(barPressure * 100.0);
    }
    else
    {
      // when pressure is above set point, let the PID act on pressure alone
      //Serial.println("Sp: " + String(pressurePidSetpoint) + " O: " + String(pressurePidOutput) + " I: " + String(pressurePidInput) + " B: " + String(rawPressurePerc));
      pressurePID.SetMode(AUTOMATIC);
      pressurePID.SetTunings(highPressureP, highPressureI, highPressureD, highPressurePon);
      pressurePidInput = (int)(barPressure * 100.0);
    }
  }

  float temperature = externalBoilerTempSensor.getTemperatureC();
  int tempMv = externalBoilerTempSensor.getTemperatureMv();
  float tempR = externalBoilerTempSensor.getTemperatureResistance();

  // Serial.println(
  //   "pSp: " + String(pressurePidSetpoint) +
  //   " pOut: " + String(pressurePidOutput) +
  //   " pIn: " + String(pressurePidInput) +
  //   " pressure: " + String(barPressure) +
  //   " Pump: " + String(pumpLevel) +
  //   " C: " + String(temperature)
  // );

  Serial.println(
      "mv: " + String(tempMv) +
      " r: " + String(tempR) +
      " c: " + String(temperature));

  // Handle Pump

  if (pressurePID.GetMode() == AUTOMATIC)
  {
    // use PID output when pid is on
    pumpLevel = (float)(pressurePidOutput / 100.0);
  }
  else
  {
    // use encoder position when pid is off.

    pumpLevel = (int)(encoderPosition * (float)pump.getPumpRange()) + pump.getPumpMin();
    if (pumpPowerIsOn)
    {
      pressurePidOutput = (int)(pumpLevel * 100.0);
    }
    else
    {
      pressurePidOutput = pump.getPumpMin() * 100.0;
    }
  }

  // set pump leve to 0 when pump is off.
  if (!pumpPowerIsOn)
  {
    pumpLevel = -1;
  }

  if (lastPumpLevel != pumpLevel)
  {
    // Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    pump.setPumpLevel(pumpLevel);
    if (pumpPowerIsOn)
    {
      float pumpPerc = pump.getPumpPercent();
      pPumpPowerBLEChar->setValue(pumpPerc);
    }
    else
    {
      float powerOff = -1.0;
      pPumpPowerBLEChar->setValue(powerOff);
    }
    blePumpPowerNotifyFlag = true;
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
  // if (bleNotified)
  // {
  //   delay(50);
  // }
  // bleNotified = false;
}
