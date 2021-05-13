
/**
 * NOTE: The mains frequency must be set in ./lib/Dimmable-Light-Arduino-master/src/thyristor.h
 * It has been set 60hz for this application.
 */

#include "pump-module.h"

int gZcPin;
int gPowerOn = false;

PumpModule::PumpModule(int zcPin, int cntrlPin) : pump(cntrlPin)
{
  xHandle = NULL;
  pumpMin = 120;
  pumpMax = 254;
  pumpRange = pumpMax - pumpMin;
  pumpLevel = pumpMax;
  oldPowerIsOn = false;
  powerIsOn = false;
  Serial.println("New PumpModule - zc pin: " + String(zcPin) + " cntrl pin: " + String(cntrlPin));
  setZeroCrossPin(zcPin);
  Serial.println("New Pump Modules");
}

PumpModule::~PumpModule()
{
  if (xHandle != NULL)
  {
    vTaskDelete(xHandle);
  }
}

/**
     * FreeRTOS callback to monitor the ZC pin to see if the pump is recieving power
     * we don't user interupts becuase the DimmableLight library has already attached one
     */
void PumpModule::watchPumpPowerTask(void *pump)
{
  PumpModule *myself = (PumpModule *)pump;
  Serial.println("Watching Pump Power on pin " + String(myself->getZeroCrossPin()));
  int counter = 0;
  // TODO: we need to reset this loop if the ZC pin changes
  int pin = myself->getZeroCrossPin();
  int pinValue = 0;

  for (;;)
  {

    pinValue = digitalRead(pin);
    // pin is pulled high when power is off and is ac wave when power is on
    // Therefor is pin holds at high for a certain number of cycles, we can be sure the power is off.
    // Serial.print(String(pinValue));
    if (pinValue == 1)
    {
      counter++;
      if (counter > 10)
      {
        counter = 0;
        myself->_setPowerIsOn(false);
      }
    }
    else
    {
      counter = 0;
      myself->_setPowerIsOn(true);
    }

    // note that the delay value must not be an even multiple of the ac frequency,
    // otherwise the digital read value could always read high or low
    // 60hz = ~16ms per cycle, 50hz - 20ms per cycle
    vTaskDelay(35 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void PumpModule::setCallbacks(PumpModuleCallbacks *pCallbacks)
{
  callbacks = pCallbacks;
}

void PumpModule::setZeroCrossPin(int zcPin)
{
  zeroCrossPin = zcPin;
  gZcPin = zcPin;
  DimmableLightLinearized::setSyncPin(zcPin);
}

int PumpModule::getZeroCrossPin()
{
  return zeroCrossPin;
}

void PumpModule::begin()
{
  Serial.println("PumpModule Begin");

  xTaskCreate(
    &PumpModule::watchPumpPowerTask,
    "pumpMon",
    15000,
    this,
    10, 
    &xHandle
  );

  DimmableLightLinearized::begin();
}

bool PumpModule::getPowerIsOn()
{
  return powerIsOn;
}

void PumpModule::_setPowerIsOn(bool isOn)
{
  oldPowerIsOn = powerIsOn;
  powerIsOn = isOn;
  if (oldPowerIsOn != powerIsOn)
  {
    if (powerIsOn)
    {
      callbacks->onPowerOn(this);
    }
    else
    {
      callbacks->onPowerOff(this);
    }

    Serial.println("Pump power: " + String(isOn));
  }
}

void PumpModule::setPumpMin(int min)
{
  pumpMin = min;
  pumpRange = pumpMax - pumpMin;
}

int PumpModule::getPumpMin()
{
  return pumpMin;
}

void PumpModule::setPumpMax(int max)
{
  pumpMax = max;
  pumpRange = pumpMax - pumpMin;
}

int PumpModule::getPumpMax()
{
  return pumpMax;
}

int PumpModule::getPumpRange()
{
  return pumpRange;
}

/**
     * Set the percentage pump level 
     * (normalized between the pump min and max)
     */
void PumpModule::setPumpPercent(float perc)
{
  Serial.println("setPumpPercent: " + String(perc));

  if (perc < 0)
  {
    perc = 0;
  }
  else if (perc > 1)
  {
    perc = 1;
  }
  pumpLevel = pumpMin + round(pumpRange * perc);
  pump.setBrightness(pumpLevel);
}

float PumpModule::getPumpPercent()
{
  float pumpPerc = (float)(pumpLevel - pumpMin) / (float)pumpRange;
  return pumpPerc;
}

/**
 * Set the absolute pump value 
 * Value range is between 0 and 255
 * but cannot be less than pumpMin or larger than pumpMax
 */
void PumpModule::setPumpLevel(int level)
{
  if (level < pumpMin)
  {
    level = pumpMin;
  }
  else if (level > pumpMax)
  {
    level = pumpMax;
  }
  pumpLevel = level;
  pump.setBrightness(pumpLevel);
}

int PumpModule::getPumpLevel()
{
  return pumpLevel;
}
