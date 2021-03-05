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
#define PRESSURE_CHARACTERISTIC_ID "c14f18ef-4797-439e-a54f-498ba680291d" // standard characteristic for pressure in bars
#define BARS_UNIT_ID "2780"

#define ENC_PRESSURE_MODE 0
#define ENC_POWER_MODE 1

// Pins (set for ESP32 Devkit-c; see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
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
volatile int pumpLevel = 0;
volatile int lastPumpLevel = 0;
// The minimum value below which the pump cuts-out
const int pumpMin = 130;
// DimmableLight value range is 0 - 255 - but Robotdyn seems to cut out at 100% (255)
const int pumpMax = 254;
const int pumpRange = pumpMax - pumpMin;
// DimmableLight pump(pumpControlPin);

PumpModule pump(pumpZeroCrossPin, pumpControlPin);


int encoderMode = ENC_PRESSURE_MODE;

RotaryEncoderModule rotaryEncoder(encoderPin1, encoderPin2, encoderSwitchPin);


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
double Setpoint = 0;
double Input = 0;
double Output = 0;

PID pressurePID(&Input, &Output, &Setpoint, 3.75, 6, 1, P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used

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
    float perc = module->getPercent();
    Serial.println("onEncoderChange: " + String(perc));
    Setpoint = perc * 10.0;
  }
};

class PumpCallbacks : public PumpModuleCallbacks
{
  void onPowerOn(PumpModule *module)
  {
    Serial.println("onPowerOn");
    Output = 0;
    pressurePID.SetMode(AUTOMATIC);
  }

  void onPowerOff(PumpModule *module)
  {
    Serial.println("onPowerOff");
    Output = 0;
    pressurePID.SetMode(MANUAL);
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

  // Rotary Encoder
  rotaryEncoder.setCallbacks(new RotartEncodeCallbacks());
  rotaryEncoder.begin();

  // PID
  Setpoint = 10; // bars
  pressurePID.SetOutputLimits(pumpMin, pumpMax);
  pressurePID.SetMode(pump.getPowerIsOn() ? AUTOMATIC : MANUAL);
}

/**
 * Main Loop
 */
void loop()
{

  if (encoderMode == ENC_PRESSURE_MODE)
  {
    pumpLevel = Output;
  }

  int lastRawPressure = rawPressure;
  int rawPressure = analogRead(pressureSensorPin);
  int normalizeRawPressure = rawPressure - minRawPressure;
  float rawPressurePerc = (float)((float)normalizeRawPressure / (float)rawPressureRange);
  lastBarPressure = barPressure;
  // calculate the bar pressure, rounding two 2 decimals
  // TODO: this seems to result in a float like 12.1200000012345
  barPressure = roundf(rawPressurePerc * 10.0 * 100.0) / 100.0;

  if (lastBarPressure != barPressure)
  {
    delay(100);
    if (deviceConnected)
    {
      pPressureCharacteristic->setValue(barPressure);
      pPressureCharacteristic->notify();
      delay(3);
    }

    // Pressure is not a useful input value when there is no resistance to the pump.
    // e.g. when the portafilter is empty, the pressure will be approaching 0, as flow approaches pump maximum.
    // ideally the system would model flow rate rather than pressure to solve this.
    // For now, when the system pressure is below a set point, we *very* roughly estimate the flow rate
    // The pump has specifications for flow at a given pressure, but not for a variable power supply
    // TODO: characterize pump flow at various power levels - this can be done once a scale
    // is integrated to measure water output accross different power levels.

    const float setPoint = .2;
    if (rawPressurePerc < setPoint)
    {

      pressurePID.SetTunings(3.75, 5.5, .5);
      // under 2 bars, transition to a roughly estimated flow-rate
      // flow will be estimated as the inverse of pressure * the pump power %
      // float pumpPerc = (float)(pumpLevel - pumpMin) / (float)pumpRange;
      float pumpPerc = pump.getPowerIsOn() ? pump.getPumpPercent() : 0;
      float estFlow = (1 - rawPressurePerc) * pumpPerc;
      // create a differential to increase weight of flow vs pressure on PID Input, as pressure drops
      float delta = setPoint - rawPressurePerc;
      float nomalizedDelta = 1 / setPoint * delta; // scale delta to 0 - 1;
      float blendedInput = (nomalizedDelta * estFlow) + ((1 - nomalizedDelta) * rawPressurePerc);
      Serial.println("out: " + String(Output) + " p: " + String(barPressure) + " flow: " + estFlow + " pump: " + String(pumpPerc) + " delta: " + String(nomalizedDelta) + " input: " + blendedInput);

      Input = blendedInput * 10.0;
    }
    else
    {
      // when pressure is above set point, let the PID act on pressure alone
      Serial.println("Sp: " + String(Setpoint) + " O: " + String(Output) + " I: " + String(Input) + " B: " + String(rawPressurePerc));
      pressurePID.SetTunings(3.75, 6, 1);
      Input = barPressure;
    }
  }

  if (lastPumpLevel != pumpLevel)
  {
    // Serial.println(pumpLevel);
    lastPumpLevel = pumpLevel;
    pump.setPumpLevel(pumpLevel);
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
