#define INCLUDE_uxTaskGetStackHighWaterMark 1
#ifdef DEBUG_FREERTOS
#include "freertos-debug-utils.h"
#endif

#include <Arduino.h>
#include <Wire.h>
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
#include "ssr-heater-module.h"
#include "flow-fs2012-module.h"

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
#define PUMP_POWER_CHAR_ID "d8ad3645-50ad-4f7a-a79d-af0a59469455"      // BT read-only characteristic for pumps power level
#define TEMP_SENSOR_CHAR_ID 0x2A1C                                     // BT read-only characteristic for boiler external temperature - "Temperature Measurement" standard id
#define TEMP_TARGET_CHAR_ID "14da7f12-2faa-47c2-89af-1752098c281b"     // BT read/write characteristic for target boiler temperature
#define FLOW_SENSOR_CHAR_ID "f5ec47c3-b240-49e8-a7c8-1b5fe5537cde"     // BT read-only characteristic for in-flow rate
#define BARS_UNIT_ID "2780"

#define ENC_PRESSURE_MODE 0
#define ENC_POWER_MODE 1

// The cycle time of the boiler Heater SSR
#define HEATER_SSR_PERIOD_MS 1000

/** 
 * Pins 
 * set-up for ESP32 Devkit-c
 * see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
 */
const unsigned char encoderPin1 = 25;      // clk
const unsigned char encoderPin2 = 26;      // dt
const unsigned char encoderSwitchPin = 27; // push button switch (sw)
const unsigned char pumpZeroCrossPin = 18; // zc
const unsigned char pumpControlPin = 19;   // pwm
// ADC Channel 1 must be used when Esp32 WiFi or BT is active
// NOTE: Pins 34 to 39 are input only
const unsigned char pressureSensorPin = 33;    // pressure sensor analog output -> adc
const unsigned char boilerTempSensorPin = 32;  // thermistor -> adc
const unsigned char boilerTempControlPin = 17; // GPIO to SSR

// I2C bus
// SDA / SCL Require Pull-up resistors to VDD
// Typical pullup value is 4.7k for 5V
// note that the 5V <-> 3.3v level shifter has 10K pull-up resistor installed
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
// https://learn.sparkfun.com/tutorials/bi-directional-logic-level-converter-hookup-guide
// https://www.arduino.cc/en/reference/wire

const unsigned char i2cSda = 23; // i2c data line
const unsigned char i2cScl = 22; // i2c clock line

/**
 * I2C Globals
 */
TwoWire I2C = TwoWire(0);

/**
 * Flow Sensor Globals
 */
FlowFs2012Module flowSensor(&I2C);
float lastFlow = 0.0;

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

float pumpLevel = 0.0;
float lastPumpLevel = 0.0;
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
BLECharacteristic *pTemperatureSensorBLEChar = NULL;
BLECharacteristic *pTemperatureTargetBLEChar = NULL;
BLECharacteristic *pFlowSensorBLEChar = NULL;

float blePressureTarget = 0;
float lastBlePressureTarget = 0;
bool blePressureSensorNotifyFlag = false;
bool blePressureTargetNotifyFlag = false;
bool blePumpPowerNotifyFlag = false;

float bleTemperatureTarget = 93.0;
float lastBleTemperatureTarget = bleTemperatureTarget;
bool bleTemperatureTargetNotifyFlag = false;

/**
 * Pressure PID Globals
 * https://playground.arduino.cc/Code/PIDLibrary/
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 * https://github.com/Dlloydev/QuickPID
 * https://www.crossco.com/resources/technical/how-to-tune-pid-loops/
 */
float pressurePidSetpoint = 0;
float pressurePidInput = 0;
float pressurePidOutput = 0;
// Proportional Gain - Dependant on pOn value. A mix of proportional response to measurement vs error
// PoM: higher value increases conservativeness. As pressure increases, P decreases.
float lowPressureP = .1;
float highPressureP = .25;
// Integral Gain (per sample period)
float lowPressureI = 1.0;
float highPressureI = 10.0;
// Derivative Gain (per second)
float lowPressureD = 0;
float highPressureD = 0;
// Ratio of Proportional on Measurement vs Proportional on Error
// 0 = 100% PoM, 1 = 100% PoE
// see: http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
float lowPressurePon = 1.0;
float highPressurePon = 1.0;

QuickPID pressurePID(
    &pressurePidInput,
    &pressurePidOutput,
    &pressurePidSetpoint,
    lowPressureP,   // Kp
    lowPressureI,   // Ki
    lowPressureD,   // Kd
    lowPressurePon, // POn - Proportional on Error weighting. O = 100% Proportional on Measurement, 1 = 100% Proportional on Error
    0,              // DOn
    QuickPID::DIRECT);

/**
 * Temperature PID Globals
 */

SsrHeaterModule ssrHeater(boilerTempControlPin, HEATER_SSR_PERIOD_MS);

float tempPidSetpoint = bleTemperatureTarget; /* celsius */
float tempPidInput = 0;
float tempPidOutput = 0;

QuickPID tempPID(
    &tempPidInput,
    &tempPidOutput,
    &tempPidSetpoint,
    1.6,   // Kp
    0.025, // Ki
    0.25,  // Kd
    0.5,   // POn - Proportional on Error weighting. O = 100% Proportional on Measurement, 1 = 100% Proportional on Error
    0,     // DOn,
    QuickPID::DIRECT);

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
    pressurePID.SetMode(QuickPID::AUTOMATIC);
  }

  void onPowerOff(PumpModule *module)
  {
    Serial.println("onPowerOff");
    pumpPowerIsOn = false;
    // pressurePidOutput = 0;
    // rotaryEncoder.setPercent(0);
    pressurePID.SetMode(QuickPID::MANUAL);
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
      Serial.print("Received BT Target Pressure (" + String(value.length()) + "): ");

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

class TemperatureTargetBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    String strValue;

    if (value.length() > 0)
    {
      Serial.print("Received BT Target Temperature (" + String(value.length()) + "): ");

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

    bleTemperatureTarget = fValue;

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
    if (bleTemperatureTargetNotifyFlag)
    {
      pPressureTargetBLEChar->notify();
      blePressureTargetNotifyFlag = false;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setEncoderMode(int mode)
{
  if (mode == ENC_POWER_MODE)
  {
    encoderMode = ENC_POWER_MODE;
    pressurePID.SetMode(QuickPID::MANUAL);
  }
  else
  {
    encoderMode = ENC_PRESSURE_MODE;
    pressurePID.SetMode(QuickPID::AUTOMATIC);
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

  // flow sensor support 100k and 400k freq
  I2C.begin(i2cSda, i2cScl, 100000);

  flowSensor.begin();

  pump.setCallbacks(new PumpCallbacks());
  pump.begin();
  pump.setPumpPercent(1);

  Serial.println("Pump Module Configured");

  pressureSensor.begin();
  Serial.println("Pressure Sensor Module Configured");

  // External Boiler Temperature
  externalBoilerTempSensor.begin();
  Serial.println("Temperature Sensor Module Configured");

  // Boiler Heater SSR Control
  // ssrHeater.setDutyCyclePercent(.5));
  ssrHeater.begin();
  Serial.println("Boiler Heater SSR Module Configured");

  tempPID.SetOutputLimits(0.0, 100.0);
  tempPID.SetSampleTimeUs(1000000); // 1 second
  tempPID.SetMode(QuickPID::AUTOMATIC);

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

  /**
   * Pump Power
   */
  pPumpPowerBLEChar = pService->createCharacteristic(
      PUMP_POWER_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Pump Power Characteristic Created");
  pPumpPowerBLEChar->addDescriptor(new BLE2902());

  pTemperatureSensorBLEChar = pService->createCharacteristic(
      (uint16_t)TEMP_SENSOR_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Temperature Sensor Characteristic Created");
  pTemperatureSensorBLEChar->addDescriptor(new BLE2902());

  /**
   * Target Temperature
   */
  pTemperatureTargetBLEChar = pService->createCharacteristic(
      TEMP_TARGET_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  Serial.println("BLE Temperature Target Characteristic Created");
  pPressureTargetBLEChar->setCallbacks(new TemperatureTargetBLECharCallbacks());
  Serial.println("BLE Temperature Target Characteristic Callback Initialized");
  pPressureTargetBLEChar->addDescriptor(new BLE2902());

  pFlowSensorBLEChar = pService->createCharacteristic(
      FLOW_SENSOR_CHAR_ID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("BLE Temperature Sensor Characteristic Created");
  pFlowSensorBLEChar->addDescriptor(new BLE2902());

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
  pressurePidSetpoint = 90.0;        // bars * 10
  pressurePID.SetSampleTimeUs(1000); // 1 ms
  pressurePID.SetOutputLimits(0.0, 100.0);
  pressurePID.SetMode(pump.getPowerIsOn() ? QuickPID::AUTOMATIC : QuickPID::MANUAL);

  //pFlowSensorBLEChar->setValue(0);

  xTaskCreate(bleNotifyTask, "bleNotify", 5000, NULL, 1, NULL);
}

/**
 * Main Loop
 */
void loop()
{

  float flow = flowSensor.getFlowRateMlPerMin();
  int rawFlow = flowSensor.getRawFlowRate();

  Serial.println("rawFlow: " + String(rawFlow));
  if (flow != lastFlow)
  {
    pFlowSensorBLEChar->setValue(flow);
    lastFlow = flow;
  }

  // Handle Encoder
  if (lastEncoderPosition != encoderPosition)
  {
    // Note: encoderPosition is set in RotartEncodeCallbacks
    lastEncoderPosition = encoderPosition;
    Serial.println("encoder: " + String(encoderPosition));
    float scaledTargetPressure = encoderPosition * 10.0;
    pressurePidSetpoint = (float)(encoderPosition * 100.0);
    pPressureTargetBLEChar->setValue(scaledTargetPressure);
    blePressureTargetNotifyFlag = true;
  }

  // Handle Ble Target Pressure Changes
  if (lastBlePressureTarget != blePressureTarget)
  {
    lastBlePressureTarget = blePressureTarget;
    pressurePidSetpoint = (float)(lastBlePressureTarget * 10.0);
    rotaryEncoder.setPercent(lastBlePressureTarget / 10.0);
    pPressureTargetBLEChar->setValue(lastBlePressureTarget);
    blePressureTargetNotifyFlag = true;
  }

  lastBarPressure = barPressure;
  barPressure = pressureSensor.getPressureBars();

  if (lastBarPressure != barPressure)
  {
    pressurePidInput = barPressure * 10.0;

    if (deviceConnected)
    {
      pPressureSensorBLEChar->setValue(barPressure);
      blePressureSensorNotifyFlag = true;
    }
    // Pressure is not a useful input value when there is no resistance to the pump.
    // e.g. when the portafilter is empty, the pressure will be approaching 0, as flow approaches pump maximum.
    // ideally the system would model flow rate rather than pressure to solve this.
    // Intead, we switch off the PID when there is no pressurization.
    if (barPressure < .25)
    {
      pressurePID.SetMode(QuickPID::MANUAL);
      //tempPID.SetMode(QuickPID::AUTOMATIC);
    }
    else if (barPressure < 4)
    {
      // use low pressure PID vals
      pressurePID.SetTunings(lowPressureP, lowPressureI, lowPressureD, lowPressurePon, 0);
      pressurePID.SetMode(QuickPID::AUTOMATIC);
      //tempPID.SetMode(QuickPID::AUTOMATIC);
    }
    else
    {
      // use high pressure PID vals
      pressurePID.SetTunings(highPressureP, highPressureI, highPressureD, highPressurePon, 0);
      pressurePID.SetMode(QuickPID::AUTOMATIC);

      // Force the heater to 100% duty cycle when pump is running at high rate
      // We do this to reduce electrical noise when their is a high power draw within
      // It will smooth out the pressure and head-off water temperature drops during
      // high flow rates.
      // However, for safety, do not exceed the temperature Setpoint
      // if (tempPidInput < (tempPidSetpoint) + 1.0 && pumpLevel > .75)
      // {
      //   tempPID.SetMode(QuickPID::MANUAL);
      //   tempPidOutput = 100.0;
      // }
    }
  }

  /**
   * Handle BLE Target Temperature change
   */
  if (lastBleTemperatureTarget != bleTemperatureTarget)
  {
    lastBleTemperatureTarget = bleTemperatureTarget;
    tempPidSetpoint = bleTemperatureTarget;
    pTemperatureTargetBLEChar->setValue(bleTemperatureTarget);
    bleTemperatureTargetNotifyFlag = true;
  }

  /**
   * Hanlde Boilder Temp control
   */
  float temperature = externalBoilerTempSensor.getTemperatureC();
  if (temperature > 50)
  {
    // Force the heater to 100% duty cycle when pump is running at high rate
    // We do this to reduce electrical noise when their is a high power draw.
    // This smooths out the pressure and heads-off water temperature drops during
    // high flow rates. For safety, do not want to exceed the temperature Setpoint
    if (tempPidInput < (tempPidSetpoint) + 1.0 && pumpLevel > .75 && barPressure >= 4)
    {
      tempPID.SetMode(QuickPID::MANUAL);
      tempPidOutput = 100.0;
    }
    else if (tempPidInput > tempPidSetpoint + .5)
    {
      // we want to reset the PID outputSum to remove an accumulated integral quickly
      // (setting PID to Manual resets the integral and PoM)
      tempPID.SetMode(QuickPID::MANUAL);
      tempPidOutput = 0.0;
    }
    else
    {
      tempPID.SetMode(QuickPID::AUTOMATIC);
    }
  }
  else
  {
    // if the temp is below 50c, the heater power is either off and we don't want to run the pid calc,
    // or it is on but just warming up. In either case, we want to drive the heater at 100%.
    Serial.println(
        "tSp: " + String(tempPidSetpoint) +
        " tOut: " + String(tempPidOutput) +
        " tIn: " + String(tempPidInput) +
        " C " + String(temperature));
    tempPID.SetMode(QuickPID::MANUAL);
    tempPidOutput = 100.0;
  }

  tempPidInput = temperature;
  ssrHeater.setDutyCyclePercent((float)(tempPidOutput / 100.0));

  // int tempMv = externalBoilerTempSensor.getTemperatureMv();
  // float tempR = externalBoilerTempSensor.getTemperatureResistance();

  // Serial.println(
  //     "tSp: " + String(tempPidSetpoint) +
  //     " tOut: " + String(tempPidOutput) +
  //     " tIn: " + String(tempPidInput) +
  //     " C " + String(temperature));
  // Serial.println(
  //     "pSp: " + String(pressurePidSetpoint) +
  //     " pOut: " + String(pressurePidOutput) +
  //     " pIn: " + String(pressurePidInput) +
  //     " pressure: " + String(barPressure) +
  //     " Pump: " + String(pumpLevel) +
  //     " C: " + String(temperature));

  // Serial.println(
  //     "mv: " + String(tempMv) +
  //     " r: " + String(tempR) +
  //     " c: " + String(temperature));

  // Serial.println("flow" + String(flowSensor.getRawFlowRate());

  pTemperatureSensorBLEChar->setValue(temperature);

  // Handle Pump

  if (pressurePID.GetMode() == QuickPID::AUTOMATIC)
  {
    // use PID output when pid is on
    pumpLevel = (float)(pressurePidOutput / 100.0);
  }
  else
  {
    // use encoder position when pid is off.
    pumpLevel = encoderPosition;
    if (pumpPowerIsOn)
    {
      pressurePidOutput = pumpLevel * 100.0;
    }
    else
    {
      pressurePidOutput = 0;
    }
  }

  // set pump level to 0 when pump is off.
  if (!pumpPowerIsOn)
  {
    pumpLevel = -1;
  }

  if (lastPumpLevel != pumpLevel)
  {
    Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    pump.setPumpPercent(pumpLevel);
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
  tempPID.Compute();

#ifdef DEBUG_FREERTOS
  char *rtos_debug_buff;
  vTaskGetRunTimeStats(rtos_debug_buff);
  // rtos_debug_buff += '\0';
  Serial.print(rtos_debug_buff);
#endif
}
