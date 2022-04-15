
#include "pump-module.h"

int gZcPin;
int gPowerOn = false;

PumpModule *_pumpModule;

/**
 * The Zero Cross Interupt Handler
 */
void IRAM_ATTR handleZeroCrossIsr()
{
  _pumpModule->doVariablePeriodPsm();
}

PumpModule::PumpModule(uint8_t zcPin, uint8_t _ctrlPin)
{
  _pumpModule = this;
  xHandle = NULL;
  pumpMin = 0;
  pumpMax = 99;
  pumpRange = pumpMax - pumpMin;
  pumpLevel = pumpMax;
  oldPowerIsOn = false;
  powerIsOn = false;
  ctrlPin = _ctrlPin;
  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, LOW);
  Serial.println("New PumpModule - zc pin: " + String(zcPin) + " ctrl pin: " + String(ctrlPin));
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
 * FreeRTOS callback to monitor the ZC pin to see if the pump is receiving power
 * TODO: refactor to use interupt along with PSM code.
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
  // DimmableLightLinearized::setSyncPin(zcPin);
}

int PumpModule::getZeroCrossPin()
{
  return zeroCrossPin;
}

void PumpModule::begin()
{
  Serial.println("PumpModule Begin");

  attachInterrupt(zeroCrossPin, handleZeroCrossIsr, RISING);

  xTaskCreate(
      &PumpModule::watchPumpPowerTask,
      "pumpMon",
      15000,
      this,
      10,
      &xHandle);

  // DimmableLightLinearized::begin();
}

/**
 * Variable PSM Interupt handler.
 * NOTE: this is called from the ISR. It cannot contain float values
 * see: https://www.reddit.com/r/esp32/comments/lj2nkx/just_discovered_that_you_cant_use_floats_in_isr/
 */
void IRAM_ATTR PumpModule::doVariablePeriodPsm()
{

  if (psmIndex < psmMaxPeriodCounts)
  {
    // The previous PSM period is still active
    if (psmIndex < currPsmOnCounts)
    {
      // set pump control pin high
      digitalWrite(ctrlPin, HIGH);
    }
    else
    {
      // set pump control pin low
      digitalWrite(ctrlPin, LOW);
    }
  }

  psmIndex++;

  if (psmIndex >= currPsmPeriodCounts)
  {
    psmIndex = 0;
    currPsmPeriodCounts = nextPsmPeriodCounts;
    currPsmOnCounts = nextPsmOnCounts;
  }
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
  // Serial.println("setPumpPercent: " + String(perc));

  if (perc < 0)
  {
    perc = 0;
  }
  else if (perc > 1)
  {
    perc = 1;
  }
  pumpLevel = pumpMin + round(pumpRange * perc);
  computeVPsm();
}

/**
 * Current pump modulation level as percentage.
 * @returns {float} - a value between 0 and 1
 */
float PumpModule::getPumpPercent()
{
  float pumpPerc = (float)(pumpLevel - pumpMin) / (float)pumpRange;
  Serial.println("p%: " + String(pumpLevel) + " - " + String(pumpMin) + " / " + String(pumpRange));
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
  computeVPsm();
}

int PumpModule::getPumpLevel()
{
  return pumpLevel;
}

/**
 * Calculate the next set of Variable PSM values.
 * This approach finds the lowest number of AC cycles that can be used to represent
 * the desired power level. EG. 10% = 1:10 (1 cycle on, 9 off); 20% = 1:5 (1 cycle on, 4 off).
 * The maximum cycle period is set with psmMaxPeriodCounts value.
 * The current PSM period will finish and then these values will be used in the next cycle.
 * This function is called from setPumpLevel and setPumpPerc so that the VPSM values
 * are updated as needed. This async pattern is used to avoid unnecessary calculation in the ISR,
 * and to work around the limitations on float values in the ISR.
 */
void PumpModule::computeVPsm()
{
  // calculate the next PSM period
  Serial.println("p%: " + String(pumpLevel) + " - " + String(pumpMin) + " / " + String(pumpRange));

  float pumpPerc = (float)(pumpLevel - pumpMin) / (float)pumpRange;
  Serial.println("%: " + String(pumpPerc));

  uint16_t rawCounts = round(pumpPerc * (float)psmMaxPeriodCounts);
  Serial.println("raw: " + String(rawCounts));

  uint16_t gcdCounts = gcd(rawCounts, psmMaxPeriodCounts);
  Serial.println("gcd: " + String(gcdCounts));

  nextPsmPeriodCounts = gcdCounts > 0 ? psmMaxPeriodCounts / gcdCounts : 0;
  nextPsmOnCounts = rawCounts > 0 ? rawCounts / gcdCounts : 0;

  Serial.println("psm: " + String(nextPsmOnCounts) + " : " + String(nextPsmPeriodCounts));
}