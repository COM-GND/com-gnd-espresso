
/**
 * NOTE: The mains frequency must be set in ./lib/Dimmable-Light-Arduino-master/src/thyristor.h
 * It has been set 60hz for this applicatio.
 */

#include <dimmable_light.h>
#include "pump-module.h"
// #include "pump-module-callbacks.h"

int gZcPin;
int gPowerOn = false;

PumpModule::PumpModule(int zcPin, int cntrlPin) : pump(cntrlPin)
{
  pumpMin = 130;
  pumpMax = 254;
  pumpRange = pumpMax - pumpMin;
  pumpLevel = pumpMax;
  oldPowerIsOn = false;
  powerIsOn = false;
  Serial.println("New PumpModule - zc pin: " + String(zcPin) + " cntrl pin: " + String(cntrlPin));
  setZeroCrossPin(zcPin);
  Serial.println("New Pump Modules");
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
    // pin is pulled low when power is on
    // Serial.print(String(pinValue));
    if (pinValue == 1)
    {
      counter++;
      if (counter > 4)
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

    delay(100);
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
  DimmableLight::setSyncPin(zcPin);
}

int PumpModule::getZeroCrossPin()
{
  return zeroCrossPin;
}

void PumpModule::begin()
{
  Serial.println("PumpModule Begin");

  xTaskCreate(&PumpModule::watchPumpPowerTask, "pumpMon", 10000, this, 1, NULL);

  DimmableLight::begin();
}

bool PumpModule::getPowerIsOn()
{
  return powerIsOn;
}

void PumpModule::_setPowerIsOn(bool isOn)
{
  oldPowerIsOn = powerIsOn;
  powerIsOn = isOn;
  if(oldPowerIsOn != powerIsOn) {
    if(powerIsOn) {
      callbacks->onPowerOn(this);
    } else {
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
